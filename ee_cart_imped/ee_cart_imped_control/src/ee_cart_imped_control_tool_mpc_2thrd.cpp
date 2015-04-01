// OLD THREADED VERSION
// Working version before spinning off the opt thread into its own node using
// shared mem.

//For class and function comments, see 
//include/ee_cart_imped_control/ee_cart_imped_control.hpp
//or the API docs
//
//The structure of this code borrows from the Realtime controller
//KDL tutorial and the joint trajectory spline controller
//
// Hacked version which includes (or attempts to) feedback from the gripper.
// Uses some of the code from pr2_gripper_sensor_controller to do this.
// This gives a real-time interface to the gripper sensors which bypasses
// having to get a message into a realtime controller (which is a pain)

// MPC version. Hoo boy. 
#include "ee_cart_imped_control/ee_cart_imped_control_tool_mpc.hpp"
#include <pluginlib/class_list_macros.h>

using namespace ee_cart_imped_control_ns;

void EECartImpedControlClassToolMPC::talk(std::string str)
{
  if(talker->trylock())
  {
    talker->msg_.data = str;
    talker->unlockAndPublish();
  }
}

// Sets the current control mode. Has to set both the local var and the
// rosparam. 
void EECartImpedControlClassToolMPC::setMode(int newMode)
{
    curMode = newMode;
    //node_.setParam("control_mode",curMode);
}
    

/*
 * Read a weight matrix from a CSV file. Assumes the file matches the given
 * numbers of rows/cols so set them right OK?
 */
MatrixXd EECartImpedControlClassToolMPC::readWeightFile(std::string filename, int nR, int nC)
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

MatrixXd EECartImpedControlClassToolMPC::readWeightsFromDir(const std::string dirName, const std::string filename, int nR, int nC)
{
    char fnBuf[200];
    sprintf(fnBuf,"%s/%s",dirName.c_str(),filename.c_str());

    MatrixXd mat = readWeightFile(fnBuf,nR,nC);
    return mat;
}

/*
 * Helper function which loads all the weights needed for the predicitve deep
 * model. Makes it easier to change the structure.
 * Currently, this is a simple 1-layer recurrent net. We'll upgrade later.
 */
void EECartImpedControlClassToolMPC::loadDeepWeights()
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
void EECartImpedControlClassToolMPC::optThreadFunc()
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
    
    ROS_INFO("Started opt");
    talk("Started opt");    
    
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
  boost::mutex::scoped_lock sLock(stateMtx);
    optStepInBlock = curStepInBlock;

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

    optXBarrPos = xBarrPos;
    optXBarrNeg = xBarrNeg;
    optYHoldPos = yHoldPos;
    optCutDist = cutDist;

    optPrevPos = lastEndP;

    xBarrPos = optXBarrPos + optPrevPos(0);
    xBarrNeg = optXBarrNeg + optPrevPos(0);
    cutDist =  optPrevPos(2) - optCutDist;
    yHoldPos = optPrevPos(1);

    optXBarrPos = xBarrPos;
    optXBarrNeg = xBarrNeg;
    optYHoldPos = yHoldPos;
    optCutDist = cutDist;

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
            boost::this_thread::yield();
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
            boost::this_thread::yield();

            
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
        boost::this_thread::yield();

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
  boost::mutex::scoped_lock sLock(stateMtx);
        if(!(curStepInBlock == optStepInBlock) || blockOffset > 0 || curMode == MODE_NORMAL)
        {
            if(curMode < MODE_MPC_WARM)
            {
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
            if(blockOffset > 0)
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
                blockOffset = 0;
                //std::cout << "Tick: " << optIter << "\n";

                optPrevFeat.head(FEAT_BLOCK_SZ*2) = prevFeat;
                optCurFeat.head(FEAT_BLOCK_SZ*2) = curFeat;
                optPrevPos = lastEndP;
            }

            optStepInBlock = curStepInBlock;


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
        boost::this_thread::yield();

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

        if(!(optIter%SLEEP_PD))
        {
            ros::Time sleepStart = ros::Time::now(); 
            boost::this_thread::sleep(boost::posix_time::microseconds(1));
            ros::Time sleepEnd = ros::Time::now();

            std::stringstream ssTS;
            double tDiff = sleepEnd.toSec() - sleepStart.toSec();
            if(tDiff > 0.001)
            {
                ssTS << "Sleep diff: " << tDiff;
            }
        }

        optIter++;
    }


}

double EECartImpedControlClassToolMPC::linearlyInterpolate(double time, 
						    double startTime, 
						    double endTime, 
						    double startValue, 
						    double endValue) {
  //std::stringstream ss;

  /*double retVal = startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);*/

  //ss << boost::format("%.6f %.6f %.6f %.6f %.6f %.6f") % time % startTime % endTime % startValue % endValue % retVal;

  //talk("linearlyInterpolate() called");

  return startValue + 
    (time - startTime)*
    (endValue - startValue)/(endTime - startTime);
}

ee_cart_imped_msgs::StiffPoint 
EECartImpedControlClassToolMPC::sampleInterpolation() {

  boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
  desired_poses_box_.get(desired_poses_ptr);
  if (!desired_poses_ptr) {
    ROS_FATAL("ee_cart_imped_control: current trajectory was NULL!");
  }
  const EECartImpedTraj &desiredPoses = desired_poses_ptr->traj;
  const ee_cart_imped_msgs::StiffPoint &initial_point = 
    desired_poses_ptr->initial_point;
  const ros::Time &current_goal_start_time = desired_poses_ptr->starting_time;
  if (desiredPoses.size() == 0) {
    ROS_ERROR("ee_cart_imped_control: Empty trajectory");
    return last_point_;
  }
  if (last_goal_starting_time_ != current_goal_start_time.toSec()) {
    //we have never seen this goal before
    last_point_ = initial_point;
    last_point_.time_from_start = ros::Duration(0);
  }
  last_goal_starting_time_ = current_goal_start_time.toSec();

  // time from the start of the series of points
  double time = robot_state_->getTime().toSec();
  double timeFromStart = time - current_goal_start_time.toSec();  
  ee_cart_imped_msgs::StiffPoint next_point;
    

  //Find the correct trajectory segment to use
  //We don't want a current_goal_index_ because it has
  //the potential to get caught in a bad race condition
  int current_goal_index;
  for (current_goal_index = 0; 
       current_goal_index < (signed int)desiredPoses.size();
       current_goal_index++) {
    if (desiredPoses[current_goal_index].time_from_start.toSec() >= timeFromStart) {
      break;
    }
  }

      // If the tool frame is set to be relative to the hand, then we need
    // to update it every time we change the hand frame. 
    // If not, we already set it up when we got the trajectory, and don't need
    // to update it here
    if(!use_fixed_frame_)
    {
        if(current_goal_index > 0)
        {
            tool_frame_ = KDL::Frame(KDL::Rotation::Quaternion(desiredPoses[current_goal_index-1].pose.orientation.x,desiredPoses[current_goal_index-1].pose.orientation.y,desiredPoses[current_goal_index-1].pose.orientation.z,desiredPoses[current_goal_index-1].pose.orientation.w)*hand_to_tool_);
        }
        else
        {
            tool_frame_ = KDL::Frame(KDL::Rotation::Quaternion(initial_point.pose.orientation.x,initial_point.pose.orientation.y,initial_point.pose.orientation.z,initial_point.pose.orientation.w)*hand_to_tool_);
        }
          /*double hX, hY, hZ, hW;
  tool_frame_.M.GetQuaternion(hX,hY,hZ,hW);

   std::stringstream ss;
        ss << boost::format("Updated TF %.3f %.3f %.3f %.3f") % hX % hY % hZ % hW;
        talk(ss.str());*/
    }

  if (current_goal_index >= (signed int)desiredPoses.size()) {
    //we're done with the goal, hold the last position
    return desiredPoses[current_goal_index-1];
  }

  //did we move on to the next point?
  if (current_goal_index > 0 && last_point_.time_from_start.toSec() != 
      desiredPoses[current_goal_index-1].time_from_start.toSec()) {
    //this should be where we CURRENTLY ARE
    //...except if we're trying to run a fine-grained trajectory where
    // the trajectory itself matters. In that case, using current pos
    // will increase tracking error. So we switch to using the traj!
    /*last_point_.pose.position.x = x_.p(0);
    last_point_.pose.position.y = x_.p(1);
    last_point_.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(last_point_.pose.orientation.x,
		       last_point_.pose.orientation.y,
		       last_point_.pose.orientation.z,
		       last_point_.pose.orientation.w);*/

    // Sets start point based on traj rather than current. Can switch this
    // out if you'd rather use the current (for wide-spaced traj's maybe?)
    last_point_.pose.position.x = desiredPoses[current_goal_index-1].pose.position.x;
    last_point_.pose.position.y = desiredPoses[current_goal_index-1].pose.position.y;
    last_point_.pose.position.z = desiredPoses[current_goal_index-1].pose.position.z;
    last_point_.pose.orientation.x = desiredPoses[current_goal_index-1].pose.orientation.x;
    last_point_.pose.orientation.y = desiredPoses[current_goal_index-1].pose.orientation.y;
    last_point_.pose.orientation.z = desiredPoses[current_goal_index-1].pose.orientation.z;
    last_point_.pose.orientation.w = desiredPoses[current_goal_index-1].pose.orientation.w;

    last_point_.wrench_or_stiffness = 
      desiredPoses[current_goal_index-1].wrench_or_stiffness;
    last_point_.isForceX = desiredPoses[current_goal_index-1].isForceX;
    last_point_.isForceY = desiredPoses[current_goal_index-1].isForceY;
    last_point_.isForceZ = desiredPoses[current_goal_index-1].isForceZ;
    last_point_.isTorqueX = desiredPoses[current_goal_index-1].isTorqueX;
    last_point_.isTorqueY = desiredPoses[current_goal_index-1].isTorqueY;
    last_point_.isTorqueZ = desiredPoses[current_goal_index-1].isTorqueZ;
    last_point_.time_from_start = 
      desiredPoses[current_goal_index-1].time_from_start;

    /*std::stringstream ss;
    ss << boost::format("Updated frame, Z: %+.3f %+.3f %+.3f") % tool_frame_(0,2) % tool_frame_(1,2) % tool_frame_(2,2);
    talk(ss.str());*/
  }

  ee_cart_imped_msgs::StiffPoint start_point;
  const ee_cart_imped_msgs::StiffPoint &end_point = 
    desiredPoses[current_goal_index];
  //actually now last_point_ and initial point should be the
  //same if current_goal_index is zero
  if (current_goal_index == 0) {
    start_point = initial_point;
  } else { 
    start_point = last_point_;
  }

  double segStartTime = 0.0;
  if (current_goal_index > 0) {
    segStartTime = 
      desiredPoses[current_goal_index-1].time_from_start.toSec();
  }
  double segEndTime = 
    desiredPoses[current_goal_index].time_from_start.toSec();
  if (segEndTime <= segStartTime) {
    //just stay where we currently are
    next_point.pose.position.x = x_.p(0);
    next_point.pose.position.y = x_.p(1);
    next_point.pose.position.z = x_.p(2);
    x_.M.GetQuaternion(next_point.pose.orientation.x,
		       next_point.pose.orientation.y,
		       next_point.pose.orientation.z,
		       next_point.pose.orientation.w);

    //talk("In first clause");

  } else {
    
    next_point.pose.position.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.x, end_point.pose.position.x);

    /*if (updates_ % 11 == 0)
    {
        std::stringstream ss;
        ss << boost::format("%.6f %.6f %.6f %.6f %.6f %.6f") % timeFromStart % segStartTime % segEndTime % start_point.pose.position.x % end_point.pose.position.x % next_point.pose.position.x;
        talk(ss.str());
    }*/

    next_point.pose.position.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.y, end_point.pose.position.y); 

    next_point.pose.position.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.position.z, end_point.pose.position.z); 

    next_point.pose.orientation.x = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.x, end_point.pose.orientation.x); 

    next_point.pose.orientation.y = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.y, end_point.pose.orientation.y);
 
    next_point.pose.orientation.z = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.z, end_point.pose.orientation.z); 

    next_point.pose.orientation.w = linearlyInterpolate
      (timeFromStart, segStartTime, segEndTime, 
       start_point.pose.orientation.w, end_point.pose.orientation.w); 
  }
  //we don't currently interpolate between wrench
  //and stiffness as generally the user wants one
  //wrench through a full trajectory point and then
  //another at the next trajectory point
  //perhaps, however, we should put in some interpolation
  //to avoid fast transitions between very different wrenches
  //as these can lead to bad behavior
  next_point.wrench_or_stiffness = 
    desiredPoses[current_goal_index].wrench_or_stiffness;
  next_point.isForceX = desiredPoses[current_goal_index].isForceX;
  next_point.isForceY = desiredPoses[current_goal_index].isForceY;
  next_point.isForceZ = desiredPoses[current_goal_index].isForceZ;
  next_point.isTorqueX = desiredPoses[current_goal_index].isTorqueX;
  next_point.isTorqueY = desiredPoses[current_goal_index].isTorqueY;
  next_point.isTorqueZ = desiredPoses[current_goal_index].isTorqueZ;
  next_point.time_from_start = ros::Duration(timeFromStart);
  return next_point;
}

void EECartImpedControlClassToolMPC::toolPoseCB
(const geometry_msgs::Pose &msg)
{
  //talk("GOT NEW TOOL POSE");
  new_hand_to_tool_ = KDL::Rotation::Quaternion(msg.orientation.x, 
				      msg.orientation.y, 
				      msg.orientation.z, 
				      msg.orientation.w);

        std::stringstream ss;
       /* ss << boost::format("%.3f %.3f %.3f %.3f") % msg.orientation.x % 
				      msg.orientation.y %  
				      msg.orientation.z %  
				      msg.orientation.w;
        talk(ss.str());*/
}

// Helper fn. to hold current pos. I think this is just starting() though  
void EECartImpedControlClassToolMPC::holdCurPos()
{
    {
        boost::mutex::scoped_lock sLock(stateMtx);
        setMode(MODE_NORMAL);
    }

    starting();
}

void EECartImpedControlClassToolMPC::commandCB
(const ee_cart_imped_msgs::EECartImpedGoalConstPtr &msg) {
  if ((msg->trajectory).empty()) {
    //stop the controller
    starting();
    return;
  }
  //this is a new goal
  boost::shared_ptr<EECartImpedData> new_traj_ptr
    (new EECartImpedData());
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory.");
    starting();
    return;
  }
    
  EECartImpedData &new_traj = *new_traj_ptr;
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
  //Operation is in fact const (although not listed as such)
  read_only_chain_.getPositions(q0);    
  fksolver.JntToCart(q0, init_pos);

  new_traj.initial_point.pose.position.x = init_pos.p(0);
  new_traj.initial_point.pose.position.y = init_pos.p(1);
  new_traj.initial_point.pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion(new_traj.initial_point.pose.orientation.x,
  			   new_traj.initial_point.pose.orientation.y,
  			   new_traj.initial_point.pose.orientation.z,
  			   new_traj.initial_point.pose.orientation.w);
 		     
  for (size_t i = 0; i < msg->trajectory.size(); i++) {
    new_traj.traj.push_back(msg->trajectory[i]);
  }
  if (!new_traj_ptr) {
    ROS_ERROR("Null new trajectory after filling.");
    starting();
    return;
  }
  new_traj.starting_time = ros::Time::now();
  desired_poses_box_.set(new_traj_ptr);
  pressIntegral = 0;
  gripTipScale = 1;
  // Update the hand to tool transform to the last one sent to us
  hand_to_tool_ = new_hand_to_tool_;

  // Update fixed/relative tool frame based on param
  node_.param("use_fixed_frame",use_fixed_frame_,true);
  
  // Update KD's based on params
  node_.param("kd_p_x",Kd_.vel(0),0.0);
  node_.param("kd_p_y",Kd_.vel(1),0.0);
  node_.param("kd_p_z",Kd_.vel(2),0.0);

  //talk("JUST UPDATED KD");

  // If we're using a fixed frame, set it. If relative, it'll be set when
  // we sample from the trajectory so don't need to worry about it here
  if(use_fixed_frame_)
  {
    tool_frame_ = KDL::Frame(hand_to_tool_);
  }
  
  chain_.getEfforts(tau_prev_);

  { // TAKE LOCK
  boost::mutex::scoped_lock sLock(stateMtx);
  lastBump = 0;
  bumpMPC = 0;
    node_.param("downCutCost",downCutCost,DOWN_CUT_COST);
    node_.param("xBarrCost",xBarrCost,X_BARR_COST);
    node_.param("yStabCost",yStabCost,Y_STAB_COST);
    node_.param("zStopCost",zStopCost,Z_STOP_COST);
    node_.param("smCost",smCost,SM_COST);

    node_.param("enCostX",enCostX,EN_COST);
    node_.param("enCostY",enCostY,EN_COST);
    node_.param("enCostZPos",enCostZPos,EN_COST);
    node_.param("enCostZNeg",enCostZNeg,EN_COST);

    node_.param("optRate",optRate,OPT_RATE);
    node_.param("gradCool",gradCool,GRAD_COOL);

        node_.param("cutDist",cutDist,0.05);
    node_.param("xBarrPos",xBarrPos,0.02);
    node_.param("xBarrNeg",xBarrNeg,-0.02);
    node_.param("xBarrScale",xBarrScale,X_BARR_SCALE);
      curMode = MODE_NORMAL;
      curMPCF.fill(0.0);
    } // RELEASE LOCK
}

bool EECartImpedControlClassToolMPC::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {
    std::string root_name, tip_name;
    node_ = n;
    //ros::NodeHandle globalNH;
    gripTipDiff = 0;

    {// TAKE LOCK
    boost::mutex::scoped_lock sLock(stateMtx);
    curBlockNo = 0;
    curStepInBlock = 0;
    blockOffset = 0;
    prevMode = MODE_NORMAL;
      curMode = MODE_NORMAL;

    mpcForces.resize(FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS);
    prevFeat.resize(FEAT_BLOCK_SZ*2);
    curFeat.resize(FEAT_BLOCK_SZ*2);
    futPFeat.resize(FEAT_BLOCK_SZ);
    futFFeat.resize(FEAT_BLOCK_SZ);
    curL.resize(NUM_LATENT);
    loadDeepWeights();
    } // RELEASE LOCK

    if (!n.getParam("root_name", root_name))
    {
        ROS_ERROR("No root name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }
    if (!n.getParam("tip_name", tip_name))
    {
        ROS_ERROR("No tip name given in namespace: %s)",
                n.getNamespace().c_str());
        return false;
    }
    n.param("use_fixed_frame",use_fixed_frame_,true);

    // get a handle to the hardware interface 
    pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
    if(!hardwareInterface)
    {
        ROS_ERROR("Perhaps Something wrong with the hardware interface pointer!!!!");
    }

    #ifndef PRESS_DBG
    // get a handle to our accelerometer 
    std::string accelerometer_name;
    if(!n.getParam("accelerometer_name", accelerometer_name))
    {
        ROS_ERROR("No accelerometer given in namespace: '%s')", n.getNamespace().c_str());
        return false;
    }
    pr2_hardware_interface::Accelerometer* accelerometerHandle = hardwareInterface->getAccelerometer(accelerometer_name);
  
    if(!accelerometerHandle)
    {
        ROS_ERROR("PR2GripperSensorController could not find accel named '%s'", accelerometer_name.c_str());
        return false;
    }
    myAccelerationObserver = new accelerationObserver(accelerometerHandle);

    // get a handle to our left finger pressure sensors 
    std::string leftFinger_pressureSensor_name;
    if(!n.getParam("left_pressure_sensor_name", leftFinger_pressureSensor_name))
    {
        ROS_ERROR("No accelerometer given in namespace: '%s')",	n.getNamespace().c_str());
        return false;
    }
    pr2_hardware_interface::PressureSensor* leftFinger_pressureSensorHandle = hardwareInterface->getPressureSensor(leftFinger_pressureSensor_name);
    if(!leftFinger_pressureSensorHandle)
    {
        ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", leftFinger_pressureSensor_name.c_str());
        return false;
    }   

    // get a handle to our right finger pressure sensors 
    std::string rightFinger_pressureSensor_name;  
    if(!n.getParam("right_pressure_sensor_name", rightFinger_pressureSensor_name))
    {
        ROS_ERROR("No accelerometer given in namespace: '%s')",   n.getNamespace().c_str());
        return false;
    }
    //ROS_INFO("EE: L name: %s R name: %s",leftFinger_pressureSensor_name,rightFinger_pressureSensor_name);
    ROS_INFO("EE: Chkpt 4.5");
    pr2_hardware_interface::PressureSensor* rightFinger_pressureSensorHandle = hardwareInterface->getPressureSensor(rightFinger_pressureSensor_name);
    if(!rightFinger_pressureSensorHandle)
    {
        ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", rightFinger_pressureSensor_name.c_str());
        return false;
    }
    myPressureObserver = new pressureObserver(leftFinger_pressureSensorHandle, rightFinger_pressureSensorHandle);

    #endif

    // Construct a chain from the root to the tip and prepare the kinematics
    // Note the joints must be calibrated
    if (!chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR("EECartImpedControlClassTool could not use the chain from '%s' to '%s'",
                root_name.c_str(), tip_name.c_str());
        return false;
    }


    if (!read_only_chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR
	  ("EECartImpedControlClassTool could not use the chain from '%s' to '%s'",
	   root_name.c_str(), tip_name.c_str());
        return false;
    }
    
    // Store the robot handle for later use (to get time)
    robot_state_ = robot;

    // Construct the kdl solvers in non-realtime
    chain_.toKDL(kdl_chain_);
    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

    // Resize (pre-allocate) the variables in non-realtime
    q_.resize(kdl_chain_.getNrOfJoints());
    qdot_.resize(kdl_chain_.getNrOfJoints());
    tau_.resize(kdl_chain_.getNrOfJoints());
    tau_act_.resize(kdl_chain_.getNrOfJoints());
    tau_prev_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());

    subscriber_ = node_.subscribe("command", 1, &EECartImpedControlClassToolMPC::commandCB, this);
    tool_pose_sub_ = node_.subscribe("tool_pose", 1, &EECartImpedControlClassToolMPC::toolPoseCB, this);
    talker.reset
      (new realtime_tools::RealtimePublisher
       <std_msgs::String>
       (node_, "talker", 1));
    controller_state_publisher_.reset
      (new realtime_tools::RealtimePublisher
       <ee_cart_imped_msgs::EECartImpedFeedback>
       (node_, "state", 1));
    controller_state_publisher_->msg_.requested_joint_efforts.resize
      (kdl_chain_.getNrOfJoints());
    controller_state_publisher_->msg_.actual_joint_efforts.resize
      (kdl_chain_.getNrOfJoints());
    updates_ = 0;
  
    
    Kd_.vel(0) = 0.0;        // Translation x                                                   
    Kd_.vel(1) = 0.0;        // Translation y
    Kd_.vel(2) = 0.0;        // Translation z
    Kd_.rot(0) = KD_ROT; //0.0;        // Rotation x
    Kd_.rot(1) = KD_ROT; //0.0;        // Rotation y
    Kd_.rot(2) = KD_ROT; //0.0;        // Rotation z

    hand_to_tool_ = KDL::Rotation::RotZ(TOOL_ROT_ANG);
    new_hand_to_tool_ = hand_to_tool_;

    //Create a dummy trajectory
    boost::shared_ptr<EECartImpedData> dummy_ptr(new EECartImpedData());
    EECartImpedTraj &dummy = dummy_ptr->traj;
    dummy.resize(1);
    dummy[0].time_from_start = ros::Duration(0);
    desired_poses_box_.set(dummy_ptr);
    last_goal_starting_time_ = -1;
    pressIntegral = 0;
    pressDiffZero = 0;
    lastBump = 0;

      //Pass the trajectory through to the update loop
  { // TAKE LOCK
  boost::mutex::scoped_lock sLock(stateMtx);
  lastBump = 0;
  bumpMPC = 0;
    node_.param("downCutCost",downCutCost,DOWN_CUT_COST);
    node_.param("xBarrCost",xBarrCost,X_BARR_COST);
    node_.param("yStabCost",yStabCost,Y_STAB_COST);
    node_.param("zStopCost",zStopCost,Z_STOP_COST);
    node_.param("smCost",smCost,SM_COST);

    node_.param("enCostX",enCostX,EN_COST);
    node_.param("enCostY",enCostY,EN_COST);
    node_.param("enCostZPos",enCostZPos,EN_COST);
    node_.param("enCostZNeg",enCostZNeg,EN_COST);

    node_.param("optRate",optRate,OPT_RATE);
    node_.param("gradCool",gradCool,GRAD_COOL);

        node_.param("cutDist",cutDist,0.05);
    node_.param("xBarrPos",xBarrPos,0.02);
    node_.param("xBarrNeg",xBarrNeg,-0.02);
    node_.param("xBarrScale",xBarrScale,X_BARR_SCALE);
      curMode = MODE_NORMAL;
      curMPCF.fill(0.0);
      }
    return true;
}

void EECartImpedControlClassToolMPC::starting() {
  // Get the current joint values to compute the initial tip location.
  KDL::Frame init_pos;
  KDL::JntArray q0(kdl_chain_.getNrOfJoints());
  KDL::ChainFkSolverPos_recursive fksolver(kdl_chain_);
  //this operation is not listed as const but is in fact
  //in the current implementation
  read_only_chain_.getPositions(q0);
  fksolver.JntToCart(q0, init_pos);


  // Also reset the time-of-last-servo-cycle
  last_time_ = robot_state_->getTime();
  ///Hold current position trajectory
  boost::shared_ptr<EECartImpedData> hold_traj_ptr(new EECartImpedData());
  if (!hold_traj_ptr) {
    ROS_ERROR("While starting, trajectory pointer was null");
    return;
  }
  EECartImpedData &hold_traj = *hold_traj_ptr;
  hold_traj.traj.resize(1);
  
  hold_traj.traj[0].pose.position.x = init_pos.p(0);
  hold_traj.traj[0].pose.position.y = init_pos.p(1);
  hold_traj.traj[0].pose.position.z = init_pos.p(2);
  init_pos.M.GetQuaternion((hold_traj.traj[0].pose.orientation.x), 
			   (hold_traj.traj[0].pose.orientation.y), 
			   (hold_traj.traj[0].pose.orientation.z), 
			   (hold_traj.traj[0].pose.orientation.w));
  hold_traj.traj[0].wrench_or_stiffness.force.x = INIT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.force.y = INIT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.force.z = INIT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.x = INIT_ROT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.y = INIT_ROT_STIFFNESS;
  hold_traj.traj[0].wrench_or_stiffness.torque.z = INIT_ROT_STIFFNESS;
  hold_traj.traj[0].isForceX = false;
  hold_traj.traj[0].isForceY = false;
  hold_traj.traj[0].isForceZ = false;
  hold_traj.traj[0].isTorqueX = false;
  hold_traj.traj[0].isTorqueY = false;
  hold_traj.traj[0].isTorqueZ = false;
  hold_traj.traj[0].time_from_start = ros::Duration(0);
  hold_traj.initial_point = hold_traj.traj[0];
  hold_traj.starting_time = ros::Time::now();
  
  chain_.getEfforts(tau_prev_);
  if (!hold_traj_ptr) {
    ROS_ERROR("During starting hold trajectory was null after filling");
    return;
  }

  // Update fixed/relative tool frame based on param
  node_.param("use_fixed_frame",use_fixed_frame_,true);
  
  // If we're using a fixed frame, set it. If relative, it'll be set when
  // we sample from the trajectory so don't need to worry about it here
  if(use_fixed_frame_)
  {
    tool_frame_ = KDL::Frame(hand_to_tool_);
  }

  //Pass the trajectory through to the update loop
  desired_poses_box_.set(hold_traj_ptr);
  
  { // TAKE LOCK
  boost::mutex::scoped_lock sLock(stateMtx);
  lastBump = 0;
  bumpMPC = 0;
    node_.param("downCutCost",downCutCost,DOWN_CUT_COST);
    node_.param("xBarrCost",xBarrCost,X_BARR_COST);
    node_.param("yStabCost",yStabCost,Y_STAB_COST);
    node_.param("zStopCost",zStopCost,Z_STOP_COST);
    node_.param("smCost",smCost,SM_COST);

    node_.param("enCostX",enCostX,EN_COST);
    node_.param("enCostY",enCostY,EN_COST);
    node_.param("enCostZPos",enCostZPos,EN_COST);
    node_.param("enCostZNeg",enCostZNeg,EN_COST);

    node_.param("optRate",optRate,OPT_RATE);
    node_.param("gradCool",gradCool,GRAD_COOL);

        node_.param("cutDist",cutDist,0.05);
    node_.param("xBarrPos",xBarrPos,0.02);
    node_.param("xBarrNeg",xBarrNeg,-0.02);
    node_.param("xBarrScale",xBarrScale,X_BARR_SCALE);
      curMode = MODE_NORMAL;
      curMPCF.fill(0.0);
      }
 
}

void EECartImpedControlClassToolMPC::update()
{
    if(!(updates_%2000))
    {
        std::stringstream ssI;
        ssI << "MODE: " << curMode;
        talk(ssI.str());
    }

    prevTime = last_time_;
    last_time_ = robot_state_->getTime();

    {
        double tDiff = last_time_.toSec()-prevTime.toSec();
        if(tDiff > 0.002)
        {
            std::stringstream ssT;
            ssT << "Time :" << tDiff;
            talk(ssT.str());
        }
    }
    // Get the current joint positions and velocities
    chain_.getPositions(q_);
    chain_.getVelocities(qdot_);

    chain_.getEfforts(tau_act_);

    // Compute the forward kinematics and Jacobian (at this location)
    jnt_to_pose_solver_->JntToCart(q_, x_);
    jnt_to_jac_solver_->JntToJac(q_, J_);

    for (unsigned int i = 0 ; i < 6 ; i++)
    {
        xdot_(i) = 0;
        for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
            xdot_(i) += J_(i,j) * qdot_.qdot(j);
    }

    ee_cart_imped_msgs::StiffPoint desiredPose = sampleInterpolation();
    // When grabbing current forces, we're going to try to take the lock,
    // but we're not going to block on it. this is to make sure we don't get
    // stuck and can stay real-time. We'll just re-use old forces if we 
    // don't get it.
    if(curMode == MODE_MPC_CUT)
    { // TRY TO TAKE LOCK
        boost::mutex::scoped_lock sLock(stateMtx,boost::try_to_lock);
        if(sLock)
            curMPCF = -mpcForces.segment(blockOffset*MPC_BLOCK_SZ+curStepInBlock*3,3);
    } // RELEASE LOCK

    Fdes_(0) = desiredPose.wrench_or_stiffness.force.x;
    Fdes_(1) = desiredPose.wrench_or_stiffness.force.y;
    Fdes_(2) = desiredPose.wrench_or_stiffness.force.z;
    Fdes_(3) = desiredPose.wrench_or_stiffness.torque.x;
    Fdes_(4) = desiredPose.wrench_or_stiffness.torque.y;
    Fdes_(5) = desiredPose.wrench_or_stiffness.torque.z;

    Kp_.vel(0) = desiredPose.wrench_or_stiffness.force.x;
    Kp_.vel(1) = desiredPose.wrench_or_stiffness.force.y;
    Kp_.vel(2) = desiredPose.wrench_or_stiffness.force.z;
    Kp_.rot(0) = desiredPose.wrench_or_stiffness.torque.x;
    Kp_.rot(1) = desiredPose.wrench_or_stiffness.torque.y;
    Kp_.rot(2) = desiredPose.wrench_or_stiffness.torque.z;

    xd_.p(0) = desiredPose.pose.position.x;
    xd_.p(1) = desiredPose.pose.position.y;
    xd_.p(2) = desiredPose.pose.position.z;
    xd_.M = KDL::Rotation::Quaternion(desiredPose.pose.orientation.x, 
				      desiredPose.pose.orientation.y, 
				      desiredPose.pose.orientation.z, 
				      desiredPose.pose.orientation.w);

    #ifdef MPC_DBG
    if(curMode == MODE_MPC_CUT)
    {
        desiredPose.isForceX = 1;
        desiredPose.isForceZ = 1;
    }
    #endif

      // A large negative KP_Z is a sentinel for "step up MPC mode" - this will
      // bump the mode up by 1, either going normal->warmup or warmup->MPC
      if(Kp_(2) < -1 && !desiredPose.isForceZ)
      {
        Kp_(2) = -Kp_(2);
        // HACKED SO IT WON'T GO FULL MPC
        talk("NEGATIVE KP, BUMPING");
        if(curMode < MODE_MPC_CUT)
        {
            bumpMPC = 1;
        }
      }

    // Maybe move this computation to the pressure observer so we only
    // do it when there's new data
    #ifndef PRESS_DBG
    double fDiff = (myPressureObserver->padForce_left_cur_nonbiased) - (myPressureObserver->padForce_right_cur_nonbiased) - pressDiffZero;
    //double fDiff = (myPressureObserver->padForce_left_cur_nonbiased) - (myPressureObserver->padForce_right_cur_nonbiased) - pressDiffZero;
    #else
    // Just use 0 b/c we can't use the press sensor in Gazebo :(
    double fDiff = 0;
    #endif
    //Some modifications for the tool version - need to compute error and 
    //its derivative in terms of the tool frame, since stiffnesses/forces are
    //expressed in this frame. Frame itself will be initialized when
    //interpolating
    //xerr_prev_ = tool_frame_.Inverse(xerr_);
    xerr_prev_ = xerr_;

    // Calculate a Cartesian restoring force.
    xerr_.vel = x_.p - xd_.p;
    xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
            xd_.M.UnitY() * x_.M.UnitY() +
            xd_.M.UnitZ() * x_.M.UnitZ());  // I have no idea what this is
            // Turns out this craziness is the same as computing offset angles
            // around each axis. Yeah I have no idea how that works either.
    
    //xerr_ = tool_frame_.Inverse(xerr_);
    
    xerr_diff_ = xerr_ - xerr_prev_;

       // F_ is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz
    // MODIFIED! In the tool version, we assume that a desired X force
    // corresponds to a desired differential between the two fingertip pads.
    // We use a modified P-I controller, where the P-component is proportional
    // to the set-point, rather than the error - this makes sense because the
    // input and output variables are of the same order (two different forces)
    if (desiredPose.isForceX) {
      #ifndef PRESS_DBG
      #ifndef X_PRESS
      if(curMode == MODE_MPC_CUT)
      {
        if(curMPCF(0) > MPC_MAX_F)
            F_(0) = MPC_MAX_F;
        else if(curMPCF(0) < -MPC_MAX_F)
            F_(0) = -MPC_MAX_F;
        else
            F_(0) = curMPCF(0);
      }
      else
        F_(0) = Fdes_(0);
      #else
      double pressErr = fDiff*gripTipScale - Fdes_(0);

      Fdes_(0) = Fdes_(0) * gripTipScale;
      //F_(0) = KP_PR*Fdes_(0) + KI_PR*pressIntegral + INIT_VERT_F;
      F_(0) = KP_PR*pressErr + KI_PR*pressIntegral + INIT_VERT_F;
      
      /*if (!(updates_ % 10)) {
        std::stringstream ss;
        ss << boost::format("Requested vert force: %+.3f Desired diff: %+.3f") % F_(2) % Fdes_(0);
        talk(ss.str());
      }*/

      if(F_(0) > MAX_VERT_F){
        F_(0) = MAX_VERT_F;
        
        //Use the NEG_KI define to save time - could also do this by multiplying
        //w/ KI, but that'd waste some ops on the mul.
        #ifndef NEG_KI
        if(pressErr < 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
        #else
        if(pressErr > 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
        #endif

      } else if(F_(0) < -MAX_VERT_F) {
        F_(0) = -MAX_VERT_F;
        
        #ifndef NEG_KI
        if(pressErr > 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
        #else
        if(pressErr < 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
        #endif
      } else {
        pressIntegral = pressIntegral + pressErr;
      }

      // Sentinel for zeroing the fingertip sensors
      if (std::abs(Fdes_(0)) > 20) {
        pressDiffZero = pressDiffZero + fDiff;
        pressIntegral = 0;
        F_(0) = 0;

        // This should work out to just be the sign of the input, and let us
        // dynamically tell the controller if pressing harder will increase or
        // decrease the fingertip differential
        gripTipScale = Fdes_(0)/std::abs(Fdes_(0));

        //talk("GOT SENTINEL");
      }

      /*
      if (!(updates_ % 10)) {
        std::stringstream ss;
        double integScaled = pressIntegral * KI_PR;
        ss << boost::format("%.3f %.3f %.3f %.3f") % F_(0) % pressIntegral % integScaled % fDiff;
        talk(ss.str());
      }*/
      #endif
      #else
      if(curMode == MODE_MPC_CUT)
      {
        //std::cout << "HERE\n";
        if(curMPCF(0) > MPC_MAX_F)
            F_(0) = MPC_MAX_F;
        else if(curMPCF(0) < -MPC_MAX_F)
            F_(0) = -MPC_MAX_F;
        else
            F_(0) = curMPCF(0);
      }
      else
        F_(0) = Fdes_(0);
      #endif
    } else {
      if(Kp_(0) < F_EPS)
        F_(0) = 0;
      else
        F_(0) = -Kp_(0) * xerr_(0) - Kd_(0)*xerr_diff_(0);
    }

    if (desiredPose.isForceY) {
      if(curMode == MODE_MPC_CUT)
      {
        if(curMPCF(1) > MPC_MAX_F)
            F_(1) = MPC_MAX_F;
        else if(curMPCF(1) < -MPC_MAX_F)
            F_(1) = -MPC_MAX_F;
        else
            F_(1) = curMPCF(1);
      }
      else
        F_(1) = Fdes_(1);

      // For debugging
      //F_(2) = 0;      
      
      
    } else {
      if(Kp_(1) < F_EPS)
        F_(1) = 0;
      else
        F_(1) = -Kp_(1) * xerr_(1) - Kd_(1)*xerr_diff_(1);
      /*if(!(updates_ % 10))
      {
        std::stringstream ss;
        ss << boost::format("KD_P_Y: %.2f F from KP: %+2.2f from KD: %+2.2f total: %+2.2f") % Kd_(1) %  (-Kp_(1) * xerr_(1)) % (-Kd_(1)*xerr_diff_(1)) % F_(1);
        talk(ss.str());
      }*/
    }

    if (desiredPose.isForceZ) {
      if(curMode == MODE_MPC_CUT)
      {
        if(curMPCF(2) > MPC_MAX_F)
            F_(2) = MPC_MAX_F;
        else if(curMPCF(2) < -MPC_MAX_F)
            F_(2) = -MPC_MAX_F;
        else
            F_(2) = curMPCF(2);
        //std::cout << "In full MPC, ZF: " << F_(2) << "\n";
      }
      else
      F_(2) = Fdes_(2);
    } else {
      if(Kp_(2) < F_EPS)
        F_(2) = 0;
      else
        F_(2) = -Kp_(2) * xerr_(2) - Kd_(2)*xerr_diff_(2);
    }

    if (desiredPose.isTorqueX) {
      F_(3) = Fdes_(3);
    } else {
      F_(3) = -Kp_(3) * xerr_(3) - Kd_(3)*xerr_diff_(3);
    }

    if (desiredPose.isTorqueY) {
      F_(4) = Fdes_(4);
    } else {
      F_(4) = -Kp_(4) * xerr_(4) - Kd_(4)*xerr_diff_(4);
    }

    if (desiredPose.isTorqueZ) {
      F_(5) = Fdes_(5);
    } else {
      F_(5) = -Kp_(5) * xerr_(5) - Kd_(5)*xerr_diff_(5);
    }

    // Convert requested force back to the base frame, since that's what
    // we have the Jacobian in. Also convert the error back, since we use it
    // to compute error derivative next update. (maybe better to not?)
    //F_ = tool_frame_*F_;
    //xerr_ = tool_frame_*xerr_;

    if(curMode == MODE_MPC_CUT && std::abs(x_.p(2) - cutDist) < CUT_STOP_EPS)
    {
        talk("STOPPING CUT\n");
        holdCurPos();
    }


    // Only lock the state once every 10 ticks - run desynch for the others.
    // Maybe this should go further down, after F is computed?
    if(!(updates_ % 10))
    {
        /*if(curMode == MODE_MPC_CUT)
        {
            std::cout << "F: " << F_(0) << ", " << F_(1) << ", " << F_(2) << "\n";
            std::cout << "P: " << x_.p(0) << ", " << x_.p(1) << ", " << x_.p(2) << "\n";
            std::cout << "XB: " << xBarrPos << "  " << xBarrNeg << "CD :" << cutDist << "\n";
        }*/

        ros::Time beforeLock = ros::Time::now();
        // Get the global state lock before we do anything to it
        //boost::lock_guard<boost::mutex> ctrlLock(stateMtx);
        { // TAKE LOCK
  boost::mutex::scoped_lock sLock(stateMtx);

        prevMode = curMode;

        if(bumpMPC)
        {
            std::stringstream ssB;
            ssB << "Wants to bump, last: " << lastBump << ", now: " << updates_ << "mode: " << curMode;
            talk(ssB.str());
        }

        if(updates_ - lastBump > 20)
        {
            lastBump = 0;
        }

        if(bumpMPC && !lastBump)
        {
            talk("!!!! BUMPING MODE !!!!");
            lastBump = updates_;
            setMode(curMode + 1);
        }

        bumpMPC = 0;

        //node_.param("control_mode",curMode,MODE_NORMAL);

        //ROS_INFO("Cur mode: %d",curMode);

        // With lock:
        // Update timestep
        curStepInBlock = (curStepInBlock+1) % MPC_BLOCK_SZ;

        // Did the mode switch from non-MPC to MPC? If so, we need to
        // fire up the opt thread. It'll take care of its own setup.
        if(prevMode < MODE_MPC_WARM && curMode >= MODE_MPC_WARM)
        {
            ROS_INFO("Firing up opt");
            optThread = new boost::thread(boost::bind(&EECartImpedControlClassToolMPC::optThreadFunc,this));
            //optThread = optThrTmp;
        }

        // Just moved to next block. Need to do some bookkeeping to get
        // features in shape. In this thread, we shift the past/cur feats
        // back (making a new cur from what we stored for the future), and
        // update some timestamps.
        // Opt will compute new latents, shift forces back, and reset
        // blockOffset once it's done.
        if(!curStepInBlock)
        {
            curBlockNo = curBlockNo + 1;
            lastEndP = lastP;
            lastEndF = lastF;

            prevFeat = curFeat;
            curFeat.head(FEAT_BLOCK_SZ) = futPFeat;
            curFeat.tail(FEAT_BLOCK_SZ) = futFFeat;

            // Debug for now until opt actually does anything
            if(curMode >= MODE_MPC_WARM)
                blockOffset++;
        }

        // Standard every-time bookkeeping. Record last point & force
        lastP = Vector3d(x_.p(0),x_.p(1),x_.p(2));
        lastF = Vector3d(F_(0),F_(1),F_(2));

        int featStart = curStepInBlock*3;

        // Set future pose/force features.
        futPFeat.segment(featStart,3) = lastP - lastEndP;
        futFFeat.segment(featStart,3) = -lastF;

        // Move the feat start point up if we need to b/c we ticked over a
        // block edge
        featStart = featStart + blockOffset*MPC_BLOCK_SZ;

        // Read out forces from MPC. Might not actually be used (if we're
        // in warmup)
        curAppF = mpcForces.segment(featStart,3);
        nextAppF = mpcForces.segment(featStart+3,3);

        // Yay, we're done. Release lock and let the controller run. 
        // We now have forces to apply until the next 10-tick, and the opt
        // thread has info on the current state/forces/etc.
        ros::Time afterLock = ros::Time::now();

        double tDiff = afterLock.toSec()-beforeLock.toSec();
        if(tDiff > 0.002)
        {
            std::stringstream ssT;
            ssT << "Update Time :" << tDiff;
            talk(ssT.str());
        }
        sLock.unlock();
        }// RELEASE LOCK
    }


    /*double minRatio = 1.0;
    double tmpTau;*/
    // Convert the force into a set of joint torques
    // tau_ is a vector of joint torques q1...qn
    for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) {
        
        // iterate through the vector.  every joint torque is contributed to
        // by the Jacobian Transpose (note the index switching in J access) times
        // the desired force (from impedance OR explicit force)

        // So if a desired end effector wrench is specified, there is no position control on that dof
        // if a wrench is not specified, then there is impedance based (basically p-gain) 
      //position control on that dof
        tau_(i) = 0;
        for (unsigned int j = 0 ; j < 6 ; j++) {
            tau_(i) += J_(j,i) * F_(j);
        }

        // Check the change in each commanded effort vs. some max value. If any
        // of these changes is more than the max, we have to scale all changes
        // in tau's down so it reaches the max
        // Can't just cap individual tau's b/c of the coupled nature of the arm 
        // Add some small value to avoid divide-by-zero
        //minRatio = std::min(minRatio,MAX_TAU_DIFF/(0.1 + std::abs(tau_(i) - tau_prev_(i))));
    }
    
    //DEBUG! If you're actually using this for control, remember to set
    // the thresh back to 0.99
    /*if(minRatio < 1.99 && updates_ > 0)
    {
        std::stringstream ss1;
        //ss1 << boost::format("Ratio: %1.6f") % minRatio;
        for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) 
        {
            double tmpTau = minRatio*(tau_(i) - tau_prev_(i));
            ss1 << boost::format("%2.4f %2.4f %2.4f ") % tau_prev_(i) % tau_(i) % tmpTau;
        }
        talk(ss1.str());
    }*/

    /*std::stringstream ss1;
    
    for (unsigned int i = 0; i < 3; i++)
    {
        ss1 << boost::format("%2.6f %2.6f ") % x_.p(i) % xd_.p(i);
    }
    talk(ss1.str());*/

    /*for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++) 
    {
        tau_prev_(i) = tau_(i);
    }*/

    /*if(minRatio < 0.00001)
    {
        talk("WHOOPSIE");
    }*/

    // And finally send these torques out
    chain_.setEfforts(tau_);
    
    //Update pressure observer state
    #ifndef PRESS_DBG
    myPressureObserver->spin();
    #endif

    //publish the current state
    if (!(updates_ % 10)) {
      
      #ifdef LOG_DATA
      // Log data to the string output message for easy parsing
      std::stringstream ss;

      double rx, ry, rz, rw;

      boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
      desired_poses_box_.get(desired_poses_ptr);

      const ros::Time &current_goal_start_time = desired_poses_ptr->starting_time;
      double time = robot_state_->getTime().toSec();

      ss << boost::format("%.6f ") % (time - current_goal_start_time.toSec());

      // If this is too slow, can reorder so we do everything for DoF
      // i  first, then move on to next, but I like this format better
      for(int i = 0; i < 3; i++)
      {
        ss << boost::format("%.4f ") % x_.p(i);
      }

      x_.M.GetQuaternion(rx,ry,rz,rw);
      ss << boost::format("%.4f %.4f %.4f %.4f ") % rx % ry % rz % rw;

      for(int i = 0; i < 3; i++)
      {
        ss << boost::format("%.4f ") % xd_.p(i);
      }

      xd_.M.GetQuaternion(rx,ry,rz,rw);
      ss << boost::format("%.4f %.4f %.4f %.4f ") % rx % ry % rz % rw;

      for(int i = 0; i < 6; i++)
      {
        ss << boost::format("%.4f ") % F_(i);
      }

      for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
        ss << boost::format("%.4f ") % q_(i);
      }

      for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
        ss << boost::format("%.4f ") % tau_(i);
      }

      for(int i = 0; i < kdl_chain_.getNrOfJoints(); i++)
      {
        ss << boost::format("%.4f ") % tau_act_(i);
      }

      for(int i = 0; i < 22; i ++)
      {
        ss << boost::format("%.4f ") % myPressureObserver->pressure_current_zerod.pressure_left[i];
      }

      for(int i = 0; i < 22; i ++)
      {
        ss << boost::format("%.4f ") % myPressureObserver->pressure_current_zerod.pressure_right[i];
      }

      tool_frame_.M.GetQuaternion(rx,ry,rz,rw);
      ss << boost::format("%.4f %.4f %.4f %.4f ") % rx % ry % rz % rw;

      talk(ss.str());

      #endif


      //ss << boost::format("Pressure val: L: %+2.4f R: %+2.4f D: %+2.4f Z: %+2.4f") % myPressureObserver->padForce_left_cur_nonbiased % myPressureObserver->padForce_right_cur_nonbiased % fDiff % pressDiffZero;
      //talk(ss.str());

      //ss << boost::format("Force: %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f \n Frame Z: %+.3f %+.3f %+.3f") % F_(0) % F_(1) % F_(2) % F_(3) % F_(4) % F_(5) % tool_frame_(0,2) % tool_frame_(1,2) % tool_frame_(2,2);
      //talk(ss.str());

      KDL::Rotation offRot = xd_.M*(x_.M.Inverse());
      KDL::Vector offAx;
      double offAng = offRot.GetRotAngle(offAx);
      offAx = offAx*offAng;
      /*
      for(int i = 0; i < 3; i++)
      {
        ss << boost::format("%1.6f %1.6f ") % xerr_(i+3) % xerr_diff_(i+3);
      }

      talk(ss.str());*/

      if (controller_state_publisher_ && 
	  controller_state_publisher_->trylock()) {
	controller_state_publisher_->msg_.header.stamp = last_time_;
	controller_state_publisher_->msg_.desired = 
	  desiredPose;
	controller_state_publisher_->msg_.actual_pose.pose.position.x = 
	  x_.p.x();
	controller_state_publisher_->msg_.actual_pose.pose.position.y = 
	  x_.p.y();
	controller_state_publisher_->msg_.actual_pose.pose.position.z = 
	  x_.p.z();
	x_.M.GetQuaternion
	  (controller_state_publisher_->msg_.actual_pose.pose.orientation.x,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.y,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.z,
	   controller_state_publisher_->msg_.actual_pose.pose.orientation.w);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.x = F_(0);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.y = F_(1);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.force.z = F_(2);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.x = F_(3);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.y = F_(4);
	controller_state_publisher_->msg_.actual_pose.
	  wrench_or_stiffness.torque.z = F_(5);
	double eff_err = 0;
	for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); i++) {
	  eff_err += (tau_(i) - tau_act_(i))*(tau_(i) - tau_act_(i));
	  controller_state_publisher_->msg_.requested_joint_efforts[i] = 
	    tau_(i);
	  controller_state_publisher_->msg_.actual_joint_efforts[i] = 
	    tau_act_(i);
      //ss << boost::format("%2.6f %2.6f %2.6f ") % q_(i) % tau_(i) % tau_act_(i);
	}
    //talk(ss.str());
	controller_state_publisher_->msg_.effort_sq_error = eff_err;
	boost::shared_ptr<const EECartImpedData> desired_poses_ptr;
	desired_poses_box_.get(desired_poses_ptr);
	controller_state_publisher_->msg_.goal = desired_poses_ptr->traj;
	controller_state_publisher_->msg_.initial_point = last_point_;
	controller_state_publisher_->msg_.running_time =
	  robot_state_->getTime() - desired_poses_ptr->starting_time;
	controller_state_publisher_->unlockAndPublish();
      }
    }
    updates_++;

        {
            ros::Time endTime = ros::Time::now();
        double tDiff = endTime.toSec()-last_time_.toSec();
        if(tDiff > 0.002)
        {
            std::stringstream ssT;
            ssT << "ETime :" << tDiff;
            talk(ssT.str());
        }
    }
}

void EECartImpedControlClassToolMPC::stopping() {
  starting();
    { // TAKE LOCK
  boost::mutex::scoped_lock sLock(stateMtx);
    setMode(MODE_NORMAL);
    }
    talk("STOPPING CALLED, TRYING TO JOIN");
  optThread->join();

}


/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(ee_cart_imped_control,
            EECartImpedControlPluginToolMPC,
			ee_cart_imped_control_ns::EECartImpedControlClassToolMPC,
			pr2_controller_interface::Controller)

