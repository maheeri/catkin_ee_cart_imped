// Separate node for the optimization portion of the MPC controller. This
// lets it run outside of a thread in the realtime controller, which
// a) Lets it run faster (can get its own core)
// b) Avoids interference with the realtime controller
//
// The controller and optimization (this file) use shared memory to communicate.
// The controller will write robot state to this memory and read out optimized
// forces put there by the optimization process.
//
// Some things here might work a little weirdly (hopefully not inefficiently)
// because the code was originally designed to work in a thread inside
// the arm controller process.

#include "ee_cart_imped_control/mpc_defs.hpp"
#include "ee_cart_imped_control/mpc_cart_opt.hpp"

void MPCCartOptClass::talk(std::string str)
{
   std::cout << str;
}

/*
 * Read a weight matrix from a CSV file. Assumes the file matches the given
 * numbers of rows/cols so set them right OK? (otw. they get 0's)
 */
MatrixXd MPCCartOptClass::readWeightFile(std::string filename, int nR, int nC)
{
    MatrixXd mat(nR,nC);
    std::ifstream infile(filename.c_str());
    std::string line;

    for(int r = 0; r < nR; r++)
    {
        getline(infile,line);
        std::stringstream strstr(line);
        std::string word = "";

        for(int c = 0; c < nC; c++)
        {
            getline(strstr,word,',');
            mat(r,c) = std::atof(word.c_str());
        }
    }
    
    infile.close();
    return mat;
}

MatrixXd MPCCartOptClass::readWeightsFromDir(const std::string dirName, const std::string filename, int nR, int nC)
{
    char fnBuf[200];
    sprintf(fnBuf,"%s/%s",dirName.c_str(),filename.c_str());

    MatrixXd mat = readWeightFile(fnBuf,nR,nC);
    return mat;
}

/*
 * Helper function which loads all the weights needed for the predicitve deep
 * model. Makes it easier to change the structure.
 */
void MPCCartOptClass::loadDeepWeights()
{
    WP_L = readWeightsFromDir(WEIGHT_DIR,"WP_L.csv",FEAT_BLOCK_SZ*2+1,NUM_FAC_L).transpose();
    WC_L = readWeightsFromDir(WEIGHT_DIR,"WC_L.csv",FEAT_BLOCK_SZ*2+1,NUM_FAC_L).transpose();
    WL_1 = readWeightsFromDir(WEIGHT_DIR,"WL_1.csv",NUM_FAC_L,NUM_LATENT).transpose();
    WL_2 = readWeightsFromDir(WEIGHT_DIR,"WL_2.csv",NUM_LATENT+1,NUM_LATENT).transpose();
    WL_L = readWeightsFromDir(WEIGHT_DIR,"WL_L.csv",NUM_LATENT+1,NUM_LATENT).transpose();
    WL_O = readWeightsFromDir(WEIGHT_DIR,"WL_O.csv",NUM_LATENT+1,NUM_FAC_O).transpose();
    WC_O = readWeightsFromDir(WEIGHT_DIR,"WC_O.csv",FEAT_BLOCK_SZ*2+1,NUM_FAC_O).transpose();
    WF_O = readWeightsFromDir(WEIGHT_DIR,"WF_O.csv",FEAT_BLOCK_SZ+1,NUM_FAC_O).transpose();
    WO = readWeightsFromDir(WEIGHT_DIR,"WO.csv",NUM_FAC_O,FEAT_BLOCK_SZ).transpose();
    LInit = readWeightsFromDir(WEIGHT_DIR,"LInit.csv",NUM_LATENT,1);

    WL_L_Fu = WL_L.block<NUM_LATENT-NUM_FIX_LATENT,NUM_LATENT+1>(NUM_FIX_LATENT,0);
    WL_2_Fu = WL_2.block<NUM_LATENT-NUM_FIX_LATENT,NUM_LATENT+1>(NUM_FIX_LATENT,0);
}

// Sigmoids all values in the input. Maybe change this to a #define?
MatrixXd sigmoidAll(MatrixXd vals)
{
    return ((-1*vals).array().exp()+1).inverse().matrix();
}

// Gradient for a non-symmetric smoothed absolute value function, which
// has different slopes on the left and right and smooths out around 0.
MatrixXd unSymAbsGrad(MatrixXd vals, double offset, double posWt, double negWt)
{
    vals = vals.array() - offset;

    MatrixXd geZ = (vals.array() >= 0).cast<double>();

    MatrixXd gradScale = geZ.array()*posWt + (1-geZ.array())*negWt;

    return (gradScale.array()*(vals.array()/(vals.array().square() + ABS_EPS).sqrt().array())).matrix();
}

// Execution thread for the opt thread
// Variable name conventions:
// opt* - global state, synched from the control thread - won't change in
// the optimization loop, only when synching
// a* - local state for the opt loop - will be updated in-loop
void MPCCartOptClass::optThreadFunc()
{
    // When this function starts, it means we just mode-switched to either
    // MPC warmup or full MPC. This thread doesn't care which, since it has
    // the same job either way (just ctrl won't actually use the forces if
    // it's in warmup)
    // First thing is to set up our locals, which will hold copies of the global
    // state, synching every few optimization iters
    
    int optBlockNo;
    int optIter = 1;
    int optStepInBlock;

    int prevTick = 0;
    
    ROS_INFO("Started opt");  
    
    VectorXd optPrevFeat(FEAT_BLOCK_SZ*2+1);
    VectorXd optCurFeat(FEAT_BLOCK_SZ*2+1);
    VectorXd optFutPFeat(FEAT_BLOCK_SZ);
    VectorXd optFutFFeat(FEAT_BLOCK_SZ);

    optCurFeat(optCurFeat.size()-1) = 1.0;
    optPrevFeat(optPrevFeat.size()-1) = 1.0;

    double optXBarrPos, optXBarrNeg, optYHoldPos, optCutDist;
    int optStepped = 0;

    /* Simple net stuff
    VectorXd optInFeat(FEAT_BLOCK_SZ*3+1);
    VectorXd optFutPFeat(FEAT_BLOCK_SZ);
    VectorXd optFutFFeat(FEAT_BLOCK_SZ);
    */

    // Pre-declate some variables to be used by the opt loop
    MatrixXd optFutL1(NUM_LATENT+1,MPC_HORIZON_BLOCKS);
    MatrixXd optFutL2(NUM_LATENT,MPC_HORIZON_BLOCKS);
    MatrixXd optFutPos(FEAT_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    MatrixXd optFutPosRel(FEAT_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    MatrixXd optDiff(FEAT_BLOCK_SZ,MPC_HORIZON_BLOCKS);

    optFutL1.block<1,MPC_HORIZON_BLOCKS>(NUM_LATENT,0).fill(1.0);

    //VectorXd aCurFeat(FEAT_BLOCK_SZ*3 + 1);
    //aCurFeat(aCurFeat.size()-1) = 1.0;

    VectorXd aCurL(NUM_LATENT + 1);
    VectorXd aCurL1(NUM_LATENT);
    Vector3d optPrevPos;
    Vector3d aPrevPos;
    VectorXd aFutF(FEAT_BLOCK_SZ+1);
    VectorXd aCurFeat(FEAT_BLOCK_SZ*2+1);
    VectorXd aPrevFeat(FEAT_BLOCK_SZ*2+1);
    VectorXd outderv;
    VectorXd outdervBack1(FEAT_BLOCK_SZ);
    VectorXd outdervBack2(FEAT_BLOCK_SZ);
    VectorXd outdervBackL(NUM_FIX_LATENT);
    VectorXd aCurR;
    VectorXd aFGrad(FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    Vector3d aAccumDiff;
    VectorXd aF_ODerv;
    VectorXd aSmGradX, aSmGradY, aSmGradZ;

    VectorXd aHP_L, aHC_L, aHL_O, aHC_O, aHF_O, aHDerv, aHC_ODerv, aHL_ODerv, aCurDynL, aHC_LDerv, aHP_LDerv;

    VectorXd gradScale(FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    gradScale.fill(0.0);

    // Add biases
    aCurL(aCurL.size()-1) = 1.0;
    aCurFeat(aCurFeat.size()-1) = 1.0;
    aPrevFeat(aPrevFeat.size()-1) = 1.0;
    aFutF(aFutF.size()-1) = 1.0;

    //optInFeat(optInFeat.size()-1) = 1.0;

    // Initialize optimal forces. Might change this - just 0's right now
    VectorXd optForces = VectorXd::Zero(FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);

    
    // Some convenience maps for if we need to access specific blocks of data.
    // If things are too slow, maybe consider refactoring back to feat-major
    // blocking
    Map<MatrixXd,0,Stride<30,3> > optXDiff(optDiff.data(),MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    Map<MatrixXd,0,Stride<30,3> > optYDiff(&optDiff.data()[1],MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    Map<MatrixXd,0,Stride<30,3> > optZDiff(&optDiff.data()[2],MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);

    Map<MatrixXd,0,Stride<30,3> > optXPos(optFutPos.data(),MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    Map<MatrixXd,0,Stride<30,3> > optYPos(&optFutPos.data()[1],MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);
    Map<MatrixXd,0,Stride<30,3> > optZPos(&optFutPos.data()[2],MPC_BLOCK_SZ,MPC_HORIZON_BLOCKS);

    Map<VectorXd,0,InnerStride<3> > optXF(optForces.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    Map<VectorXd,0,InnerStride<3> > optYF(&optForces.data()[1],MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    Map<VectorXd,0,InnerStride<3> > optZF(&optForces.data()[2],MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);

    Map<VectorXd,0,InnerStride<3> > aFGradX(aFGrad.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    Map<VectorXd,0,InnerStride<3> > aFGradY(&aFGrad.data()[1],MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    Map<VectorXd,0,InnerStride<3> > aFGradZ(&aFGrad.data()[2],MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS);

    Map<MatrixXd,0,Stride<1,3> > aFMat(optForces.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS,3);
    Map<MatrixXd,0,Stride<1,3> > aFGMat(aFGrad.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS,3);
    Map<MatrixXd,0,Stride<1,3> > gsMat(gradScale.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS,3);
    Map<MatrixXd,0,Stride<1,3> > optFutPosMat(optFutPos.data(),MPC_BLOCK_SZ*MPC_HORIZON_BLOCKS,3);

    // Following is for debug & should be removed later
    Map<VectorXd> optPosVec(optFutPos.data(),FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);

    ROS_INFO("Done initing vars");

    // Take the lock and make initial load of global state
    { // TAKE LOCK
    scoped_lock<interprocess_mutex> sLock(mpcShMem->stateMtx);
    optStepInBlock = mpcShMem->curStepInBlock;

    // Populate the "future" forces with actual applied forces where we can
    // and the opt forces otherwise. The applied ones will be clamped to those
    // vals and not optimized.
    optFutFFeat.head(optStepInBlock*3) = futFFeat.head(optStepInBlock*3);
    optFutFFeat.tail((MPC_BLOCK_SZ-optStepInBlock)*3) = optForces.segment(optStepInBlock*3,(MPC_BLOCK_SZ-optStepInBlock)*3);
    
    // Similarly populate the pose features for known steps. Just take
    // all, we'll block the ones we want in the loop.
    optFutPFeat = futPFeat;

    /*
    optInFeat.head(FEAT_BLOCK_SZ*2) = curFeat;
    optInFeat.segment(FEAT_BLOCK_SZ*2,FEAT_BLOCK_SZ) = optFutFFeat;
    */

    optPrevFeat.head(FEAT_BLOCK_SZ*2) = prevFeat;
    optCurFeat.head(FEAT_BLOCK_SZ*2) = curFeat;

    optXBarrPos = mpcShMem->xBarrPos;
    optXBarrNeg = mpcShMem->xBarrNeg;
    optYHoldPos = mpcShMem->yHoldPos;
    optCutDist = mpcShMem->cutDist;

    optPrevPos = lastEndP;

    mpcShMem->xBarrPos = optXBarrPos + optPrevPos(0);
    mpcShMem->xBarrNeg = optXBarrNeg + optPrevPos(0);
    mpcShMem->cutDist =  optPrevPos(2) - optCutDist;
    mpcShMem->yHoldPos = optPrevPos(1);

    optXBarrPos = mpcShMem->xBarrPos;
    optXBarrNeg = mpcShMem->xBarrNeg;
    optYHoldPos = mpcShMem->yHoldPos;
    optCutDist = mpcShMem->cutDist;

    // Load other params from the shared memory. Keep local copies to
    // avoid having to lock every time we touch these.
    // We might consider moving these to params on this node
    downCutCost = mpcShMem->downCutCost;
    xBarrCost = mpcShMem->xBarrCost;
    yStabCost = mpcShMem->yStabCost;
    zStopCost = mpcShMem->zStopCost;
    smCost = mpcShMem->smCost;
    enCostX = mpcShMem->enCostX;
    enCostY = mpcShMem->enCostY;
    enCostZPos = mpcShMem->enCostZPos;
    enCostZNeg = mpcShMem->enCostZNeg;
    optRate = mpcShMem->optRate;
    gradCool = mpcShMem->gradCool;
    xBarrScale = mpcShMem->xBarrScale;



    ROS_INFO("DONE WITH PARAMS");

    // OK, got all the state info we need. Unlock.
    sLock.unlock();
    } // RELEASE LOCK

    // Initialize the latents
    //curL = sigmoidAll(optInFeat*WI + LInit*WL_L);
    curL = LInit;

    #ifdef TEST_GRAD
    curL = readWeightsFromDir(WEIGHT_DIR,"latFeat.csv",NUM_LATENT,1);
    optCurFeat.head(FEAT_BLOCK_SZ*2) = readWeightsFromDir(WEIGHT_DIR,"curFeat.csv",FEAT_BLOCK_SZ*2,1);
    optPrevFeat.head(FEAT_BLOCK_SZ*2) = readWeightsFromDir(WEIGHT_DIR,"prevFeat.csv",FEAT_BLOCK_SZ*2,1);
    optPrevPos.fill(0);
    optCutDist = 0.05;
    node_.param("cutDist",optCutDist,0.05);
    node_.param("xBarrPos",optXBarrPos,0.02);
    node_.param("xBarrNeg",optXBarrNeg,-0.02);
    optYHoldPos = 0;
    #endif

    double initF;
    node_.param("initF",initF,0.0);
    gradScale.fill(0.0);
    optForces.fill(initF);
    optFutFFeat.fill(initF);

    int checkIter;
    node_.param("checkIter",checkIter,200);

    optIter = 0;
    prevL = curL;
    aCurL.head(NUM_LATENT) = prevL;

    ROS_INFO("Init CP 1");

    // Load in the fixed latents
    optFutL2.block<NUM_FIX_LATENT,MPC_HORIZON_BLOCKS>(0,0) = prevL.head(NUM_FIX_LATENT).replicate(1,MPC_HORIZON_BLOCKS);

    // Compute initial non-fixed latents
    aHP_L = sigmoidAll(WP_L*optPrevFeat);
    aHC_L = sigmoidAll(WC_L*optCurFeat);

    ROS_INFO("Init CP 2");

    optFutL1.block<NUM_LATENT,1>(0,0) = sigmoidAll(WL_1*((aHP_L.array()*aHC_L.array()).matrix()));

    ROS_INFO("Init CP 3");
    
    // CurL stores all the latents, including fixed ones even though we'll
    // replace some with fixed for this block.
    curL = sigmoidAll(WL_2*optFutL1.block<NUM_LATENT+1,1>(0,0) + WL_L*aCurL);

    ROS_INFO("Init CP 4");

    // Some of the recurrent (L2) latents will stay fixed for all future
    // TS, fortunately this is pretty easy to do.
    optFutL2.block<NUM_LATENT-NUM_FIX_LATENT,1>(NUM_FIX_LATENT,0) = curL.bottomRows(NUM_LATENT-NUM_FIX_LATENT);

    ROS_INFO("Ready to loop");

    //std::cout << "CurFeat:\n" << optCurFeat << "\n";

    // Continually run optimization. We're actually not going to compute
    // exact cost in this loop to save time
    while(true)
    {
        #ifndef DONT_OPT
        if(!(optIter%50))
        {
            std::stringstream ssT;
            //ssT << "MODE: " << curMode << "XBP :" << optXBarrPos << "XP: " << optXPos(0,0) << "XG: " << optXDiff(0,0) << "XFG: " << aFGrad(0) << "XFGS: " << sqrt(gradScale(0)) <<  " F: " << optForces.segment(0,6).transpose();
            ssT << "XF: " << optXF.transpose() << "\n XP: " << optXPos.transpose() << "\n XG: " << aFGradX.transpose() << "\n";
            talk(ssT.str());
        }

        optFutFFeat.tail((MPC_BLOCK_SZ-optStepInBlock)*3) = optForces.segment(optStepInBlock*3,(MPC_BLOCK_SZ-optStepInBlock)*3);
        aCurFeat = optCurFeat;
        aPrevFeat = optPrevFeat;
        aCurL.head(NUM_LATENT) = curL;
        aPrevPos = optPrevPos;
        
        // Fwd-prop/prediction
        for(int i = 0; i < MPC_HORIZON_BLOCKS; i++)
        {
            // Special case for 1st step since we might already know some
            // of the forces and should clamp those.
            if(i == 0)
            {
                aFutF.head(FEAT_BLOCK_SZ) = optFutFFeat;
            }
            else
            {
                aFutF.head(FEAT_BLOCK_SZ) = optForces.segment(FEAT_BLOCK_SZ*i,FEAT_BLOCK_SZ);
            }

            // Since the latents for the 1st TS don't depend on anything
            // we're optimizing, we don't need to re-compute them every time.
            // L1/L2 for these will be computed once when we synch
            if(i > 0)
            {
                aHP_L = sigmoidAll(WP_L*aPrevFeat);
                aHC_L = sigmoidAll(WC_L*aCurFeat);

                optFutL1.block<NUM_LATENT,1>(0,i) = sigmoidAll(WL_1*((aHP_L.array()*aHC_L.array()).matrix()));

                // Some of the recurrent (L2) latents will stay fixed for all future
                // TS, fortunately this is pretty easy to do.
                optFutL2.block<NUM_LATENT-NUM_FIX_LATENT,1>(NUM_FIX_LATENT,i) = sigmoidAll(WL_2_Fu*optFutL1.block<NUM_LATENT+1,1>(0,i) + WL_L_Fu*aCurL);

                aCurL.head(NUM_LATENT) = optFutL2.block<NUM_LATENT,1>(0,i);
            }

            aHL_O = sigmoidAll(WL_O*aCurL);
            aHC_O = sigmoidAll(WC_O*aCurFeat);
            aHF_O = sigmoidAll(WF_O*aFutF);

            // Get predicted pos w/r/t prev
            aCurR = WO*(aHL_O.array()*aHC_O.array()*aHF_O.array()).matrix();

            if(i == 0)
            {
                aCurR.head(optStepInBlock*3) = optFutPFeat.head(optStepInBlock*3);
            }

            aPrevFeat = aCurFeat;

            // Make new input feats based on prediction and current applied
            // force
            aCurFeat.head(FEAT_BLOCK_SZ) = aCurR;
            aCurFeat.segment(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ) = aFutF.head(FEAT_BLOCK_SZ);
            optFutPosRel.block<FEAT_BLOCK_SZ,1>(0,i) = aCurR;

            // Shift to rel. to current. 
            aCurR += aPrevPos.replicate(MPC_BLOCK_SZ,1);

            // Do we actually need to store this?
            optFutPos.block<FEAT_BLOCK_SZ,1>(0,i) = aCurR;
            aPrevPos = aCurR.tail(3);

            // Compute gradients for pose-based cost functions, to backprop
            // later. Some of these might not be used for step 1.

            // Add in sawing "barrier" gradients
            optXDiff.block<MPC_BLOCK_SZ,1>(0,i) = xBarrCost*(sigmoidAll(xBarrScale*(optXPos.block<MPC_BLOCK_SZ,1>(0,i).array() - optXBarrPos)) - sigmoidAll(xBarrScale*(optXBarrNeg - optXPos.block<MPC_BLOCK_SZ,1>(0,i).array())));

            // Y-stabilization
            optYDiff.block<MPC_BLOCK_SZ,1>(0,i) = yStabCost*(optYPos.block<MPC_BLOCK_SZ,1>(0,i).array() - optYHoldPos);

            // Z-stopping
            optZDiff.block<MPC_BLOCK_SZ,1>(0,i) = unSymAbsGrad(optZPos.block<MPC_BLOCK_SZ,1>(0,i),optCutDist,downCutCost,zStopCost);
            
            // End fwd-prop. Now we have all the latents computed, and all
            // gradients w/r/t outputs. 
        }

        //std::cout << optDiff;

        //std::cout << (optXPos.array() - optXBarrPos);
        //std::cout << "\n Neg: \n" << std::cout << (optXBarrNeg - optXPos.array());

        // Now on to backprop. The fun part.
        aAccumDiff.fill(0);
        aPrevFeat.head(FEAT_BLOCK_SZ) = optFutPosRel.block<FEAT_BLOCK_SZ,1>(0,MPC_HORIZON_BLOCKS-2);
        aPrevFeat.segment(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ) = optForces.segment(FEAT_BLOCK_SZ*(MPC_HORIZON_BLOCKS-2),FEAT_BLOCK_SZ);
        aFGrad.fill(0);
        outdervBack1.fill(0);
        outdervBack2.fill(0);
        outdervBackL.fill(0);

        //ROS_INFO("Finished pred");

        for(int i = MPC_HORIZON_BLOCKS - 1; i >= 0; i--)
        {
            optDiff.block<3,1>(FEAT_BLOCK_SZ-3,i) += aAccumDiff;

            if(i > 0)
            {
                aAccumDiff(0) = optXDiff.block<MPC_BLOCK_SZ,1>(0,i).sum();
                aAccumDiff(1) = optYDiff.block<MPC_BLOCK_SZ,1>(0,i).sum();
                aAccumDiff(2) = optZDiff.block<MPC_BLOCK_SZ,1>(0,i).sum();
            }
            
            outderv = optDiff.block<FEAT_BLOCK_SZ,1>(0,i) + outdervBack1;
            outdervBack1 = outdervBack2;
            aCurFeat = aPrevFeat;

            if(i > 1)
            {
                aPrevFeat.head(FEAT_BLOCK_SZ) = optFutPosRel.block<FEAT_BLOCK_SZ,1>(0,i-2);
                aPrevFeat.segment(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ) = optForces.segment(FEAT_BLOCK_SZ*(i-2),FEAT_BLOCK_SZ);
            }
            else if(i > 0)
            {
                aPrevFeat = optCurFeat;
            }
            else
            {
                aPrevFeat = optPrevFeat;
            }

            outderv = WO.transpose()*outderv;

            

            aCurL.head(NUM_LATENT) = optFutL2.block<NUM_LATENT,1>(0,i);
            aCurL1 = optFutL1.block<NUM_LATENT,1>(0,i);

            // Compute hiddens (factor inputs) - maybe switch to remembering
            // these, but re-computing was cheaper in MATLAB
            aHP_L = sigmoidAll(WP_L*aPrevFeat);
            aHC_L = sigmoidAll(WC_L*aCurFeat);

            aHL_O = sigmoidAll(WL_O*aCurL);
            aHC_O = sigmoidAll(WC_O*aCurFeat);
            aHF_O = sigmoidAll(WF_O*aFutF);

            // Can reuse this quantity and save some ops
            aHDerv = outderv.array()*aHC_O.array()*aHL_O.array()*aHF_O.array();
            aHC_ODerv = (aHDerv.array()*(1-aHC_O.array())).matrix();

            //aF_ODerv = WF_O*(aHDerv.array()*(1-aHF_O.array())).matrix();
            aF_ODerv = (WF_O.leftCols(WF_O.cols()-1).transpose())*((aHDerv.array()*(1-aHF_O.array())).matrix());

            aFGrad.segment(FEAT_BLOCK_SZ*i,FEAT_BLOCK_SZ) += aF_ODerv;

            outdervBack1 += WC_O.leftCols(FEAT_BLOCK_SZ).transpose()*aHC_ODerv;

            if(i > 0)
            {
                aFGrad.segment(FEAT_BLOCK_SZ*(i-1),FEAT_BLOCK_SZ) += WC_O.middleCols(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ).transpose()*aHC_ODerv;
            }
            
            // Don't need to backprop any more if we're on the 1st TS
            // - if so, we already hit all the changing forces
            if(i > 0)
            {
                // Only need to backprop error to the non-fixed latents
                outderv = WL_O.middleCols(NUM_FIX_LATENT,WL_O.cols()-1-NUM_FIX_LATENT).transpose()*(aHDerv.array()*(1-aHL_O.array())).matrix();

                outderv = outderv + outdervBackL;

                aCurDynL = aCurL.segment(NUM_LATENT-NUM_FIX_LATENT,NUM_FIX_LATENT);
                outderv = (outderv.array()*aCurDynL.array()*(1-aCurDynL.array())).matrix();

                outdervBackL = WL_L_Fu.middleCols(NUM_FIX_LATENT,WL_L_Fu.cols()-NUM_FIX_LATENT-1).transpose()*outderv;

                outderv = WL_1.transpose()*((WL_2_Fu.leftCols(WL_2_Fu.cols()-1).transpose()*outderv).array()*aCurL1.array()*(1-aCurL1.array())).matrix();

                aHDerv = outderv.array()*aHC_L.array()*aHP_L.array();

                aHC_LDerv = (aHDerv.array()*(1-aHC_L.array())).matrix();
                outdervBack1 += WC_L.leftCols(FEAT_BLOCK_SZ).transpose()*aHC_LDerv;
                aFGrad.segment(FEAT_BLOCK_SZ*(i-1),FEAT_BLOCK_SZ) += WC_L.middleCols(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ).transpose()*aHC_LDerv;

                if(i > 1)
                {
                    aHP_LDerv = (aHDerv.array()*(1-aHP_L.array())).matrix();
                    outdervBack2 = WP_L.leftCols(FEAT_BLOCK_SZ).transpose()*aHP_LDerv;
    
                    aFGrad.segment(FEAT_BLOCK_SZ*(i-2),FEAT_BLOCK_SZ) += WP_L.middleCols(FEAT_BLOCK_SZ,FEAT_BLOCK_SZ).transpose()*aHP_LDerv;
                    
                }
            }

            
        }

        //std::cout << aFGrad.segment(0,FEAT_BLOCK_SZ);

        aSmGradX = optXF.tail(optXF.size()-1) - optXF.head(optXF.size()-1);
        aSmGradY = optYF.tail(optYF.size()-1) - optYF.head(optYF.size()-1);
        aSmGradZ = optZF.tail(optZF.size()-1) - optZF.head(optZF.size()-1);

        aFGradX.head(aFGradX.size()-1) -= aSmGradX*smCost;
        aFGradX.tail(aFGradX.size()-1) += -aSmGradX*smCost;

        aFGradY.head(aFGradY.size()-1) -= aSmGradY*smCost;
        aFGradY.tail(aFGradY.size()-1) += -aSmGradY*smCost;

        aFGradZ.head(aFGradZ.size()-1) -= aSmGradZ*smCost;
        aFGradZ.tail(aFGradZ.size()-1) += -aSmGradZ*smCost;

        // Also smooth to the previous applied force
        aFGradX(0) += smCost*(optXF(0) - optCurFeat(optCurFeat.size()-3));
        aFGradY(0) += smCost*(optYF(0) - optCurFeat(optCurFeat.size()-2));
        aFGradZ(0) += smCost*(optZF(0) - optCurFeat(optCurFeat.size()-1));

        //ROS_INFO("Iter %d time %.6f\n",optIter,robot_state_->getTime().toSec());
        //aFGrad = aFGrad + optForces*enCost;
        aFGradX += optXF*enCostX;
        aFGradY += optYF*enCostY;
        VectorXd zFPos = (optZF.array() > 0).cast<double>();
        aFGradZ += ((zFPos.array()*enCostZPos + (1-zFPos.array())*enCostZNeg)*optZF.array()).matrix();
        //aFGradZ += (optZF.array() > 0).cast<double>().array()*enCostZPos;// + (optZF < 0).cast<double>().array()*enCostZNeg)*optZF.array();

        //optForces = optForces - aFGrad*optRate;
        gradScale = gradScale.array()*gradCool + aFGrad.array().square();

        /*for(int j = 0; j < MPC_HORIZON_BLOCKS; j++)
        {
            gsMat.block<MPC_BLOCK_SZ,3>(j*MPC_BLOCK_SZ,0)= (gsMat.middleRows(MPC_BLOCK_SZ*j,MPC_BLOCK_SZ).colwise().sum()/MPC_BLOCK_SZ).replicate(MPC_BLOCK_SZ,1);
        }*/

        optForces = optForces.array() - optRate*aFGrad.array()/(gradScale.array().sqrt());

        #ifdef TEST_GRAD
        if(optIter == checkIter)
        {
            //std::cout << optIter << ": " << aFMat << "\n";
            
            //std::cout <<"Scale:\n" << gsMat << "\n";
            //std::cout <<"Grad: \n" <<  (optRate*aFGrad.array()/(gradScale.array().sqrt())).transpose();
            //std::cout << "YSm\n" << aSmGradY.transpose();
            //std::cout << optXF.tail(optXF.size()-1).transpose() << "\n";
            //std::cout << optXF.head(optXF.size()-1).transpose() << "\n";
            //std::cout << "\n" << aFGrad.segment(0,30).transpose();
            std::cout.flush();
            return;
        }
        #endif

        #ifndef TEST_GRAD
        if(!(optIter%checkIter))
        {
            //std::cout << "Cut dist: " << optCutDist << "\n";
            //std::cout << optIter << " F:\n" << aFMat << "\n";
            //std::cout << "Pos:\n" << optFutPosMat << "\n";
            //std::cout << "Grad: \n" << aFGMat << "\n";
            //std::cout << "Scale: \n" << gsMat.array().sqrt() << "\n";
            //std::cout << xBarrCost*(sigmoidAll(X_BARR_SCALE*(optXPos.array() - optXBarrPos)) - sigmoidAll(X_BARR_SCALE*(optXBarrNeg - optXPos.array()))) << "\n\n";
        }
        #endif

        // And now we have that. Gradient should be set here, just apply it.
        #endif

            
        // Synchronize with global state. Consider turning this into a notify
        // rather than a poll.
        { // TAKE LOCK
        scoped_lock<interprocess_mutex> sLock(mpcShMem->stateMtx);
        if(!(mpcShMem->curStepInBlock == optStepInBlock) || mpcShMem->blockOffset > 0 || mpcShMem->curMode == MODE_NORMAL)
        {
            if(mpcShMem->curMode < MODE_MPC_WARM)
            {
                prevMode = mpcShMem->curMode;
                optRunning = 0;
                //talk("I WANT TO RETURN BUT THE MEAN MAN WON'T LET ME");
                return;
            }

            ros::Time lockStart = ros::Time::now();

            // Don't do this if we're in grad-test mode
            #ifndef TEST_GRAD
            // Take lock. Rest of this block is with lock.
            

            // Check and see if ctrl moved to the next block. If this happened,
            // we need to:
            // Shift forces back by a block
            // Propagate latents based on new data
            // Load quantities that only change here (pos to offset from)
            // Opt takes care of shifting cur/prev feat, so we don't have to
            // Right now, we're going to assume we never miss more than 1
            // offset step.
            if(mpcShMem->blockOffset > 0)
            {
                // Shift forces
                optForces.head(optForces.size() - FEAT_BLOCK_SZ) = optForces.tail(optForces.size() - FEAT_BLOCK_SZ);
                
                // Fill in new horizon forces with copies of the very last.
                // Hopefully this works OK, maybe something else would be
                // better?
                optForces.tail(FEAT_BLOCK_SZ) = optForces.tail(3).replicate(MPC_BLOCK_SZ,1);

                //gradScale.head(gradScale.size() - FEAT_BLOCK_SZ) = gradScale.tail(gradScale.size() - FEAT_BLOCK_SZ);
                //gradScale.tail(FEAT_BLOCK_SZ).fill(0);
                gradScale.fill(MIN_GRAD_SCALE);

                // We're not going to propagate latents inside the critical
                // section to keep it short - we'll do that right after.
                // Set indicator for that.
                optStepped = 1;

                optPrevPos = lastEndP;
                mpcShMem->blockOffset = 0;
                std::cout << "Tick: " << optIter-prevTick << "\n";
                prevTick = optIter;

                optPrevFeat.head(FEAT_BLOCK_SZ*2) = prevFeat;
                optCurFeat.head(FEAT_BLOCK_SZ*2) = curFeat;
                optPrevPos = lastEndP;
            }

            optStepInBlock = mpcShMem->curStepInBlock;


            optFutFFeat.head(optStepInBlock*3) = futFFeat.head(optStepInBlock*3);
            optFutFFeat.tail((MPC_BLOCK_SZ-optStepInBlock)*3) = optForces.segment(optStepInBlock*3,(MPC_BLOCK_SZ-optStepInBlock)*3);
            
            // Similarly populate the pose features for known steps. Just take
            // all, we'll block the ones we want in the loop.
            optFutPFeat = futPFeat;

            /*
            optInFeat.head(FEAT_BLOCK_SZ*2) = curFeat;
            optInFeat.segment(FEAT_BLOCK_SZ*2,FEAT_BLOCK_SZ) = optFutFFeat;
            */
            /*
            optXBarrPos = xBarrPos;
            optXBarrNeg = xBarrNeg;
            optYHoldPos = yHoldPos;
            optCutDist = cutDist;
            */
            mpcForces = optForces;

            // OK, got all the state info we need. Unlock.
            #endif
            ros::Time lockDone = ros::Time::now();

            double tDiff = lockDone.toSec() - lockStart.toSec();
            if(tDiff > 0.001)
            {
                std::stringstream ssL;
                ssL << "Opt synch locked for " << tDiff;
            }
        }
        sLock.unlock();
        } // RELEASE LOCK

        #ifdef PROP_LATS
        // Propagate latents if we stepped.
        if(optStepped)
        {
            prevL = curL;
            // Load in the fixed latents
            optFutL2.block<NUM_FIX_LATENT,MPC_HORIZON_BLOCKS>(0,0) = prevL.head(NUM_FIX_LATENT).replicate(1,MPC_HORIZON_BLOCKS);

            // Compute initial non-fixed latents
            aHP_L = sigmoidAll(WP_L*optPrevFeat);
            aHC_L = sigmoidAll(WC_L*optCurFeat);

            optFutL1.block<NUM_LATENT,1>(0,0) = sigmoidAll(WL_1*((aHP_L.array()*aHC_L.array()).matrix()));
            
            // CurL stores all the latents, including fixed ones even though we'll
            // replace some with fixed for this block.
            curL = sigmoidAll(WL_2*optFutL1.block<NUM_LATENT+1,1>(0,0) + WL_L*aCurL);

            // Some of the recurrent (L2) latents will stay fixed for all future
            // TS, fortunately this is pretty easy to do.
            optFutL2.block<NUM_LATENT-NUM_FIX_LATENT,1>(NUM_FIX_LATENT,0) = curL.bottomRows(NUM_LATENT-NUM_FIX_LATENT);
            optStepped = 0;
        }
        #endif

        optIter++;
    }
}

    // Constructor. Set up shared memory and map Eigen maps into it.
    MPCCartOptClass::MPCCartOptClass(const char* shMemLoc) : mpcForces(NULL,FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS), prevFeat(NULL,FEAT_BLOCK_SZ*2), curFeat(NULL,FEAT_BLOCK_SZ*2), futPFeat(NULL,FEAT_BLOCK_SZ), futFFeat(NULL,FEAT_BLOCK_SZ), lastEndP(NULL), lastEndF(NULL), shMemObj(open_or_create,shMemLoc,read_write)
    {
        // Default mode to normal so we never miss going to MPC mode
        prevMode = MODE_NORMAL;


        // Setup the shared memory to point to the name defined by the
        // rosparam
        

        // Size the memory to contain the shared data and point our struct
        // at it.
        shMemObj.truncate(sizeof(mpc_shmem_t));
        new(&region) mapped_region(shMemObj,read_write);

        mpcShMem = (mpc_shmem_t*)region.get_address();

        ROS_INFO("CHIGGITY CHECK IT %d",mpcShMem);

        //Set up Eigen maps into shared memory
        //This is actually awesome since once we do these, we can just use
        //'em like normal Eigen mats <3 Eigen
        new(&mpcForces) Map<VectorXd>((double*)&(mpcShMem->mpcForces),FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);
        new(&prevFeat) Map<VectorXd>((double*)&(mpcShMem->prevFeat),FEAT_BLOCK_SZ*2);
        new(&curFeat) Map<VectorXd>((double*)&(mpcShMem->curFeat),FEAT_BLOCK_SZ*2);
        new(&futPFeat) Map<VectorXd>((double*)&(mpcShMem->futPFeat),FEAT_BLOCK_SZ);
        new(&futFFeat) Map<VectorXd>((double*)&(mpcShMem->futFFeat),FEAT_BLOCK_SZ);
        new(&lastEndP) Map<Vector3d>((double*)&(mpcShMem->lastEndP));
        new(&lastEndF) Map<Vector3d>((double*)&(mpcShMem->lastEndF));

        optRunning = 0;

        curL.resize(NUM_LATENT);
        loadDeepWeights();

        mpcShMem->optReady = 1;
    }
    
    // running just means keep checking the shared memory and launch an
    // opt thread if we need it. Opt thread will take itself down when
    // we don't.
    void MPCCartOptClass::run()
    {
        ROS_INFO("CHIGGITY CHECK IT %d",mpcShMem->ctrlReady);
        ros::Rate rate(RUN_POLL_RATE);
        // Wait for the control process to be ready
        // This is important b/c control will set some stuff up in the 
        // shared memory
        while(node_.ok() && mpcShMem->ctrlReady != 1)
        {
            //ROS_INFO("READY: %d",mpcShMem->ctrlReady);
            //ROS_INFO("FUCK");
            //std::cout << curFeat(0) << "\n";
            rate.sleep();
            //boost::this_thread::sleep(boost::posix_time::milliseconds(RUN_SLEEP_TIME));
        }

        //DEBUG!!!
        //curMode = MODE_NORMAL;
        //mpcShMem->curMode = MODE_MPC_WARM;
        //int firstRun = 1;
        ROS_INFO("PAST");
        
        while(node_.ok())
        {
            //ROS_INFO("B4");
            //boost::this_thread::sleep(boost::posix_time::milliseconds(RUN_SLEEP_TIME));
            rate.sleep();
            //ROS_INFO("AFTER");
            // Don't do anything if we already have an opt thread (which
            // will
            if(optRunning)
                continue;

            scoped_lock<interprocess_mutex> sLock(mpcShMem->stateMtx);
            prevMode = curMode;
            curMode = mpcShMem->curMode;
            /*if(firstRun)
            {
                ROS_INFO("Cur: %d Prev: %d",curMode,prevMode);
                firstRun = 0;
            }*/
            if(prevMode < MODE_MPC_WARM && curMode >= MODE_MPC_WARM)
            {
                ROS_INFO("Starting MPC opt thread");
                optRunning = 1;
                optThread = new boost::thread(boost::bind(&MPCCartOptClass::optThreadFunc,this));
            }
            sLock.unlock();
        }

        // If we got here, it means ROS wants to take us down. Have to 
        // make sure the opt thread also comes down here.
        if(optRunning)
        {
            mpcShMem->curMode = MODE_NORMAL;
            optThread->join();
        }      
    }

int main(int argc, char **argv)
{
    ros::init(argc,argv,"mpc_cart_opt");
    std::string shMemName;
    std::string defaultShName(DEFAULT_SH_MEM);

    ros::NodeHandle node("~");

    node.param("sh_mem_name",shMemName,defaultShName);
    ROS_INFO("Mem name: %s",shMemName.c_str());

    ROS_INFO("HERE");

    MPCCartOptClass optObj(shMemName.c_str());
    ROS_INFO("HERE");

    optObj.run();
}
