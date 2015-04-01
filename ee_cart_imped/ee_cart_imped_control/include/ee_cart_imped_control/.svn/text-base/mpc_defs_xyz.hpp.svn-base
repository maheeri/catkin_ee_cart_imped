// Some common defines for the MPC controller. Used by both the control
// and opt processes, so they get their own file.

#ifndef MPC_DEF_H
#define MPC_DEF_H

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/shared_memory_object.hpp>

// Modes

// Totally normal ee_cart - stiffness/force control
#define MODE_NORMAL 1

// MPC "warmup" - runs MPC in the background, but doesn't actually use the
// forces - controller is still just normal ee_cart. This will warm up the
// latents and future optimal forces to make MPC work better later.
#define MODE_MPC_WARM 2

// Full cutting MPC. 
#define MODE_MPC_CUT 3

//Stop cutting when we get within 1mm of tgt pos
#define CUT_STOP_EPS 0.001

// MPC defines
//#define TEST_GRAD
#define PROP_LATS
#define MPC_DBG
//#define DONT_OPT

#define SLEEP_PD 10

#define MPC_MAX_F 15.0

// Total number of TS (0.01s increments) to predict
#define MPC_HORIZON 50

// Size of time-blocks in MPC
#define MPC_BLOCK_SZ 10

// Number of blocks til horizon
#define MPC_HORIZON_BLOCKS 10

// Size of a block (vector) of features
#define FEAT_BLOCK_SZ 30

// Various numbers of units
// Big version
#define NUM_LATENT 200
#define NUM_FIX_LATENT 100
#define NUM_FAC_L 100
#define NUM_FAC_O 500

// Small version
/*
#define NUM_LATENT 20
#define NUM_FIX_LATENT 10
#define NUM_FAC_L 10
#define NUM_FAC_O 50
*/

// Cost weights. Set these! 1's for now
#define DOWN_CUT_COST 8.6e3 //1200*0
#define X_BARR_COST 2.15e8
#define Y_STAB_COST 2.4e8
#define Z_STOP_COST 4.3e4
#define SM_COST 500.0
#define EN_COST 10.0

#define OPT_RATE 0.077
#define GRAD_COOL 0.9

// Scaling for the difference (before nonlin) for the x-barriers
#define X_BARR_SCALE 5000.0

// Other numerical stuff
// Epsilon for any smoothed abs functions, so that they're differentiable
#define ABS_EPS 1e-9

// Minimum gradient scale. If we start gradient scale at exactly 0, it might
// end up creating NaNs in some cases where gradients are close to 0
#define MIN_GRAD_SCALE 1e-5

#define WEIGHT_DIR "/home/pr2admin/mpcWeights"
//#define WEIGHT_DIR "/home/ianlenz/mpcWeights"

// Default name for shared mem. But this should really be set using the
// rosparam since otherwise both arms will use the same and that'll be a mess.
#define DEFAULT_SH_MEM "mpcCartShMem"

// Set high for debug, probably lower when doing actual opt (10-20 sounds good)
// (or replace w/ condition)
#define OPT_SYNC_PD 20


// Shared memory structure for MPC data shared between opt & control.
// Most of the double arrays here will be used with Eigen maps.
typedef struct mpc_shmem 
{
    double mpcForces[FEAT_BLOCK_SZ*MPC_HORIZON_BLOCKS];
    double prevFeat[FEAT_BLOCK_SZ*2];
    double curFeat[FEAT_BLOCK_SZ*2];
    double futPFeat[FEAT_BLOCK_SZ];
    double futFFeat[FEAT_BLOCK_SZ];
    double lastEndP[3];
    double lastEndF[3];

    // Keep params in the shared memory. It's a little inefficient, but
    // lets us use the namespace of the controller for them.
    double downCutCost, xBarrCost, yStabCost, zStopCost, smCost;
    double enCostX, enCostY, enCostZPos, enCostZNeg;
    double optRate, gradCool;
    double xBarrScale;
    double xBarrPos, xBarrNeg, yHoldPos, cutDist;

    int curStepInBlock, blockOffset;
    int curMode, prevMode;
    char optReady, ctrlReady;

    boost::interprocess::interprocess_mutex stateMtx;
} mpc_shmem_t;

#endif // MPC_DEF_H
