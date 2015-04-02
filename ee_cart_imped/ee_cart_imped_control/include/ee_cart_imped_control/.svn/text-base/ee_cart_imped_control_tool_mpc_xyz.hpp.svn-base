#ifndef __EE_CART_IMPED_CONTROL_TOOL_MPC_HPP__
#define __EE_CART_IMPED_CONTROL_TOOL_MPC_HPP__

#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>

#include <ros/ros.h>
#include <ee_cart_imped_msgs/EECartImpedAction.h>
#include <ee_cart_imped_msgs/EECartImpedGoal.h>
#include <ee_cart_imped_msgs/StiffPoint.h>

//#include <pr2_gripper_sensor_controller/pressure_observer.h>
//#include <pr2_gripper_sensor_controller/acceleration_observer.h>
#include <ee_cart_imped_control/pressure_observer.h>
#include <ee_cart_imped_control/acceleration_observer.h>
#include <pr2_hardware_interface/hardware_interface.h>

//#include <pr2_gripper_sensor_controller/pr2_gripper_sensor_controller.h>

#include <pr2_msgs/PressureState.h>
#include <std_msgs/String.h>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/mapped_region.hpp>

// If not commented out, will disable pressure sensor code
// Needs to be defined to run in Gazebo
// Will just do regular force control instead of pressure servoing
#define PRESS_DBG

// If defined, will make the X axis pressure-servoed
//#define X_PRESS

// If defined, we'll log a bunch of data to the talker topic as space-separated
// numbers (so it's easy to parse)
//#define LOG_DATA

// epsilon for force/stiff being near 0
#define F_EPS 1e-6

//Doxygen doesn't like these comments for some reason...
///Maximum stiffness for translation
//WARRANTY VOIDED! Raised the stiffness cap by a lot.
//DO NOT USE THIS FOR BIG CHANGES! Meant for smooth trajectories
//where we want to reduce error!
#define MAX_STIFFNESS 5000.0

///Maximum stiffness for rotation
//Also overclocked. Please be responsible etc.
#define ACCEPTABLE_ROT_STIFFNESS 500.0

//Want to have higher caps than we really want to start with, so set these
//separately
#define INIT_STIFFNESS 1000.0
#define INIT_ROT_STIFFNESS 75.0

//Fingertip pressure controller params.
//PI controller gains (P component is proportional to command, not error)
#define KP_PR -0.5
#define KI_PR -0.0003
#define NEG_KI

//Maximum absolute and initial forces to apply
//Initial force helps to get the tool in contact with the surface quicker,
//but should be low enough that it can easily be corrected for if it turns
//out to be too much.
#define MAX_VERT_F 20.0
#define INIT_VERT_F 2.0

#define MAX_TAU_DIFF 1.0

//Gain for derviative term for rotational components. Without this, controller
// will oscillate for some orientations
#define KD_ROT 500.0
//#define KD_ROT 250.0

//Rotation around gripper Z axis to get to tool frame
//This should give us a tool frame attached to the gripper where the X and Y
//axes define the gripper plane, with the Y axis pointing "up" (the direction
//we probably want to apply force in), and the X pointing "forwards". Z
//will be the gripper plane normal, pointing out of it ("sideways")
#define TOOL_ROT_ANG 0

using namespace Eigen;
using namespace boost::interprocess;

namespace ee_cart_imped_control_ns {

  /**
   * \brief Allows a user to control the force or stiffness applied by joints.
   *
   * This class contains the realtime controller for controlling joints
   * by stiffness or force.  To use this controller, you should use the
   * action wrapper, 
   *<A href=http://www.ros.org/wiki/ee_cart_imped_action>EECartImpedAction</A>.
   */
  class EECartImpedControlClassToolMPC: public pr2_controller_interface::Controller {
  private:
    mpc_shmem_t *mpcShMem;
    mapped_region region;
    shared_memory_object shMemObj;
    


    /// The current robot state 
    //(to get the time stamp)
    // Read-only after initialization
    pr2_mechanism_model::RobotState* robot_state_;
    
    /// The chain of links and joints in PR2 language for reference and commanding
    pr2_mechanism_model::Chain chain_;
    /// The chain of links and joints in PR2 language for reference only
    //Read-only after initialization
    pr2_mechanism_model::Chain read_only_chain_;
    /// The chain of links and joints in KDL language
    // Read-only after initialization
    KDL::Chain kdl_chain_;
    
    /// KDL Solver performing the joint angles to Cartesian pose calculation
    // Referenced only in update loop
    boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    /// KDL Solver performing the joint angles to Jacobian calculation
    // Referenced only in update loop
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    
    // The variables (which are declared as class variables
    // because they need to be pre-allocated, meaning we must
    // be very careful how we access them to avoid race conditions)
    
    /// Joint positions
    // Referenced only in update loop
    KDL::JntArray  q_; 
    /// Joint velocities
    // Referenced only in update loop
    KDL::JntArrayVel  qdot_;
    /// Joint torques   
    // Referenced only in update loop
    KDL::JntArray  tau_, tau_act_, tau_prev_;
    
    /// Tip pose
    // Referenced only in update loop
    KDL::Frame     x_, x_tool_;

    /// Tip desired pose          
    // Referenced only in update loop
    // Changes based on mode - for normal ee_cart, this is just the next
    // target pt. For cutting MPC, this is the goal pose for the cut. The 
    // controller will try to match the orientation, but X/Y/Z will be used
    // differently: 
    // X defines the midpoint of the sawing range
    // Y defines the Y value to try and hold
    // Z defines the ending Z value of the cut (which the controller will try
    // to stop at)
    KDL::Frame     xd_, xd_tool_;
    
    /// Cartesian error
    // Referenced only in update loop
    KDL::Twist     xerr_;
    /// Cartesian velocity
    // Referenced only in update loop
    KDL::Twist     xdot_;
    // Prev. & change in error
    KDL::Twist     xerr_prev_;
    KDL::Twist     xerr_diff_;


    /// Cartesian effort
    // Referenced only in update loop
    KDL::Wrench    F_;  
    /// Desired Cartesian force
    // Referenced only in update loop
    KDL::Wrench    Fdes_;
    /// Jacobian
    // Referenced only in update loop
    KDL::Jacobian  J_;

    //Reference rotation frame. Typically, this will be attached to the desired
    //gripper frame from the previous timestep (but not the actual frame since
    //this would introduce error). Force/stiffness control occurs in this frame,
    //rather than the global frame, since we'll probably want to make this
    //choice based on gripper axes rather than global axes when not working
    //with planar objects
    KDL::Frame tool_frame_;

    // Should the tool frame be used relative to the hand frame, or as a fixed
    // frame relative to the global frame?
    // This is useful because for some applications, the hand may be rotating,
    // but we want force/stiffness commands relative to some fixed frame
    // Specified by the use_fixed_frame parameter (default false)
    bool use_fixed_frame_;

    //Rotation from the hand frame (orientation specified in input traj's) to
    //the tool frame (which we choose force/stiffness control relative to)
    //Important so we can hold the tool at an angle and feel force on the
    //fingertips, and for other applications where the natural axes of F/S
    //control might be offset from the hand's
    KDL::Rotation hand_to_tool_;

    //Buffer for a new hand to tool rotation. The rotation stored here will
    //only be put into effect when the next new trajectory is received,
    //since switching this in the middle of a run might break things
    KDL::Rotation new_hand_to_tool_;

    // Store some info from the gripper tips
    int gripTipDiff;

    // Depending on the tool, downwards pressure might either increase or
    // decrease the tip differential. Store the sign of this, as communicated
    // by the zeroing command (set based on the sign of the sentinel value)
    // Could also use this value to dynamically scale
    int gripTipScale;

    int curForceStart;


    ///The time at which the goal we saw on the last iteration was started
    //Referenced only in update loop
    double last_goal_starting_time_;
    
    typedef std::vector<ee_cart_imped_msgs::StiffPoint> EECartImpedTraj;
    
    ///Class for storing a trajectory and associated information
    //This is a class so as to be able to lock and unlock this data
    //together easily
    class EECartImpedData {
    public:
      ///Desired trajectory
      EECartImpedTraj traj;
      ///The point the robot was at before beginning trajectory
      ee_cart_imped_msgs::StiffPoint initial_point;
      ///The starting time of the trajectory
      ros::Time starting_time;
      EECartImpedData() {}
    };

    ///Desired positions
    //This needs to be in realtime boxes because
    //we reference it in the update loop and modify 
    //it it in the command callback
    realtime_tools::RealtimeBox<
      boost::shared_ptr<const EECartImpedData> > desired_poses_box_;
    
    ///The actual position of the robot when we achieved the last point on the trajectory
    //Referenced only in updated loop
    ee_cart_imped_msgs::StiffPoint last_point_;

    // Note the gains are incorrectly typed as a twist,
    // as there is no appropriate type!
    /// Proportional gains
    //Referenced only in update loop
    KDL::Twist     Kp_;
    // Derivative gains
    //Referenced only in update loop
    KDL::Twist     Kd_;   
    
    /// Time of the last servo cycle
    //Modified in both command callback
    //and update loop, but it does not matter
    ros::Time last_time_, prevTime;
    
    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
	std_msgs::String> > 
    talker;

    accelerationObserver *myAccelerationObserver;   ///< accelerometer observer object
    pressureObserver *myPressureObserver;           ///< pressure observer object to do analysis on the incoming pressure sensor dat

    double pressIntegral;
    double pressDiffZero;


    // Mode last seen by ctrl - this is used to recognize changes and do what
    // needs to be done when they happen (e.g. start MPC)
    int prevMode;

    // MPC variables

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

    int bumpMPC;
    int lastBump;

    // 3DOF forces used for control. Normally updated every loop but might not
    // be if we can't get the lock.
    Vector3d curMPCF;

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

    Vector3d lastP;
    Vector3d lastF;

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

    void talk(std::string str);
    void setMode(int newMode);

    MatrixXd readWeightFile(std::string filename, int nR, int nC);
    MatrixXd readWeightsFromDir(std::string dirName, std::string filename, int nR, int nC);
    void loadDeepWeights();

    void optThreadFunc();

    /**
     * \brief Linearly interpolates between trajectory points
     *
     * Linearly interpolates between startValue and endValue over 
     * the time period startTime to endTime of the current goal.
     * @param time [seconds] that this goal has been running
     * @param startTime time that the goal began running
     * @param endTime time that the goal should finish running
     * @param startValue the values at the beginning of the interpolation (i.e. the values at startTime)
     * @param endValue the values at the end of the interpolation (i.e. the values at endTime)
     * @return ((time - startTime) * (endValue - startValue) / 
     (endTime - startTime)) + startValue; 
    */
    
    double linearlyInterpolate(double time, double startTime, 
			       double endTime, double startValue, 
			       double endValue);
    
    
    /**
     *\brief Callback when a new goal is received
     *
     *Cancels the currently active goal (if there is one), restarts
     *the controller and asks it to execute the new goal.
     *
     *@param msg the new goal
     *
     */
    void commandCB(const ee_cart_imped_msgs::EECartImpedGoalConstPtr &msg);
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    ros::Subscriber tool_pose_sub_;

    void toolPoseCB(const geometry_msgs::Pose &msg);

    void holdCurPos();
    
    /**
     *\brief Samples the interpolation and returns the point the
     * joints should currently be trying to achieve.
     *
     * Samples the interpolation, using 
     * EECartImpedControlClassTool::linearlyInterpolate, between the
     * current trajectory point and the next trajectory point.  Interpolation
     * is done for position and orientation, but not for wrench or stiffness
     * as generally it is desired that wrench or stiffness be constant
     * over a specific trajectory segment.
     *
     * @return A <A HREF=http://www.ros.org/doc/api/ee_cart_imped_msgs/html/msg/StiffPoint.html>StiffPoint</A> that is the point
     * the joints should attempt to achieve on this timestep.
     *
     */
    ee_cart_imped_msgs::StiffPoint sampleInterpolation();
    
    ///State publisher, published every 10 updates
    boost::scoped_ptr<
      realtime_tools::RealtimePublisher<
	ee_cart_imped_msgs::EECartImpedFeedback> > 
    controller_state_publisher_;
    
    ///The number of updates to the joint position
    //Referenced only in updates
    int updates_;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EECartImpedControlClassToolMPC();

    /**
     * \brief Controller initialization in non-realtime
     *
     * Initializes the controller on first startup.  Prepares the 
     * kinematics, pre-allocates the variables, subscribes to
     * command and advertises state.
     *
     * @param robot The current robot state
     * @param n A node handle for subscribing and advertising topics
     *
     * @return True on successful initialization, false otherwise
     *
     */
    bool init(pr2_mechanism_model::RobotState *robot,
	      ros::NodeHandle &n);
    /**
     * \brief Controller startup in realtime
     * 
     * Resets the controller to prepare for a new goal.  Sets the desired
     * position and orientation to be the current position and orientation
     * and sets the joints to have maximum stiffness so that they hold
     * their current position.
     */
    void starting();
    
    /**
     * \brief Controller update loop in realtime
     *
     * A PD controller for achieving the trajectory.
     * Uses EECartImpedControlClassTool::sampleInterpolation to
     * find the point that should be achieved on the current timestep.
     * Converts this point (which is in Cartesian coordinates) to joint
     * angles and uses a PD update (in the case of stiffness) to send
     * to the joints the correct force.
     *
     */
    void update();

    /**
     * \brief Controller stopping in realtime
     *
     * Calls EECartImpedControlClassTool::starting() to lock the joints into
     * their current position.
     *
     */
    void stopping();
    
  };
}

#endif //ee_cart_imped_control.hpp
