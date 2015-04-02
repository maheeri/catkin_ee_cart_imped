#ifndef MPC_CART_OPT_H
#define MPC_CART_OPT_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/mapped_region.hpp>

// Polling time to re-check state and see if we should launch the opt thread
// in ms
#define RUN_POLL_RATE 100.0

using namespace Eigen;
using namespace boost::interprocess;

class MPCCartOptClass {
    private:
    mpc_shmem_t *mpcShMem;
    mapped_region region;

    ros::NodeHandle node_;
    shared_memory_object shMemObj;
    char optRunning;

    // Control mode - 1 is normal, 2 is warmup, 3 is MPC
    int curMode;

    // Mode last seen by ctrl - this is used to recognize changes and do what
    // needs to be done when they happen (e.g. start MPC)
    int prevMode;

    // MPC variables

    // Synchronization mutex for shared state between opt and control.
    // Use this whenever accessing anything that both touch.
    boost::mutex stateMtx;

    boost::thread *optThread;

    // Future forces, as optimized by MPC. Opt writes, ctrl reads
    Map<VectorXd> mpcForces;

    // Current actual time block. This is imporant b/c when it ticks, opt
    // has to reset some stuff and shouldn't write to the global force state
    // until it's shifted.
    int curBlockNo;

    // Step # inside that block
    int curStepInBlock;
    // Number of blocks that ctrl is shifted from opt. Should usually be 0, but
    // might be 1 for a bit when we tick over a block, before opt can shift
    // things.
    int blockOffset;

    // Storage for previous and current state, and buffer for building the
    // next (which can be clamped by opt). Vectors since we'll use these as
    // weighted feats.
    Map<VectorXd> prevFeat;
    Map<VectorXd> curFeat;

    Map<VectorXd> futPFeat;
    Map<VectorXd> futFFeat;

    // Endpoint for the last sets of poses and forces
    Map<Vector3d> lastEndP;
    Map<Vector3d> lastEndF;

    // Current latent state
    VectorXd curL;
    VectorXd prevL;

    // Current and next force the controller should apply. These are not shared,
    // only used by ctrl, read out from global state. They're interpolated
    // between to get the actual control force to apply in the 1kHz controller.
    // It may make sense to read these from global state at 1kHz though.
    Vector3d curAppF;
    Vector3d nextAppF;

    // Deep weights go here. We'll store these as global Eigen matrices, read
    // from CSVs
    // To start, we're just going to use a regular recurrent deep net. We'll
    // add the fancy stuff later.
    /*
    MatrixXd WI;
    MatrixXd WO;
    */

    MatrixXd WP_L, WC_L, WL_1, WL_2, WL_O, WC_O, WF_O, WO;
    MatrixXd WL_2_Fu, WL_L_Fu;
    MatrixXd WL_L;
    MatrixXd LInit;
    
    // Keep output bias separate - this makes things a little easier
    MatrixXd BO;

    // Scaling factors and other params
    double downCutCost, xBarrCost, yStabCost, zStopCost, smCost;
    double enCostX, enCostY, enCostZPos, enCostZNeg;
    double optRate, gradCool;

    double xBarrScale;
    double xBarrPos, xBarrNeg, yHoldPos, cutDist;

    void talk(std::string str);
    void setMode(int newMode);

    MatrixXd readWeightFile(std::string filename, int nR, int nC);
    MatrixXd readWeightsFromDir(std::string dirName, std::string filename, int nR, int nC);
    void loadDeepWeights();

    void optThreadFunc();

    public:
    MPCCartOptClass(const char* shMemName);
    void run();
};

#endif // H-guard

