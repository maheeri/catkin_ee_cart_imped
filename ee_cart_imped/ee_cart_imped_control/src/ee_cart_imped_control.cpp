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
#include "ee_cart_imped_control/ee_cart_imped_control.hpp"
#include <pluginlib/class_list_macros.h>

using namespace ee_cart_imped_control_ns;

void EECartImpedControlClass::talk(std::string str)
{
  if(talker->trylock())
  {
    talker->msg_.data = str;
    talker->unlockAndPublish();
  }
}

double EECartImpedControlClass::linearlyInterpolate(double time, 
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
EECartImpedControlClass::sampleInterpolation() {

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

void EECartImpedControlClass::commandCB
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
  chain_.getEfforts(tau_prev_);
}

void EECartImpedControlClass::tipCallback(const pr2_msgs::PressureState &msg)
{
  ROS_INFO("Tip CB, prev: %d\n",gripTipDiff);
  gripTipDiff = msg.l_finger_tip[15];
}

bool EECartImpedControlClass::init(pr2_mechanism_model::RobotState *robot,
        ros::NodeHandle &n) {
    std::string root_name, tip_name;
    node_ = n;
    //ros::NodeHandle globalNH;
    gripTipDiff = 0;
    ROS_INFO("EE: Starting init");
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

    ROS_INFO("EE: Starting handles");
    // get a handle to the hardware interface 
    pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
    if(!hardwareInterface)
    {
        ROS_ERROR("Perhaps Something wrong with the hardware interface pointer!!!!");
    }

    ROS_INFO("EE: Chkpt 1");

    // get a handle to our accelerometer 
    std::string accelerometer_name;
    if(!n.getParam("accelerometer_name", accelerometer_name))
    {
        ROS_ERROR("No accelerometer given in namespace: '%s')", n.getNamespace().c_str());
        return false;
    }
    //pr2_hardware_interface::Accelerometer* accelerometerHandle = hardwareInterface->getAccelerometer(accelerometer_name);
  
    ROS_INFO("EE: Chkpt 2");
/*
    if(!accelerometerHandle)
    {
        ROS_ERROR("PR2GripperSensorController could not find sensor named '%s'", accelerometer_name.c_str());
        return false;
    }
    myAccelerationObserver = new accelerationObserver(accelerometerHandle);*/
    ROS_INFO("EE: Chkpt 3");


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

    ROS_INFO("EE: Chkpt 4");

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
    ROS_INFO("EE: Chkpt 5");
    myPressureObserver = new pressureObserver(leftFinger_pressureSensorHandle, rightFinger_pressureSensorHandle);

    ROS_INFO("EE: Chkpt 6");

    // Construct a chain from the root to the tip and prepare the kinematics
    // Note the joints must be calibrated
    if (!chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR("EECartImpedControlClass could not use the chain from '%s' to '%s'",
                root_name.c_str(), tip_name.c_str());
        return false;
    }


    if (!read_only_chain_.init(robot, root_name, tip_name))
    {
        ROS_ERROR
	  ("EECartImpedControlClass could not use the chain from '%s' to '%s'",
	   root_name.c_str(), tip_name.c_str());
        return false;
    }
    
    ROS_INFO("EE: DONE W/ ALL THAT NOISE");

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

    subscriber_ = node_.subscribe("command", 1, &EECartImpedControlClass::commandCB, this);
    //node_.subscribe("pressure",1, &EECartImpedControlClass::tipCallback, this);
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
    
    //Create a dummy trajectory
    boost::shared_ptr<EECartImpedData> dummy_ptr(new EECartImpedData());
    EECartImpedTraj &dummy = dummy_ptr->traj;
    dummy.resize(1);
    dummy[0].time_from_start = ros::Duration(0);
    desired_poses_box_.set(dummy_ptr);
    last_goal_starting_time_ = -1;
    pressIntegral = 0;
    pressDiffZero = 0; 
    return true;
}

void EECartImpedControlClass::starting() {
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
  //Pass the trajectory through to the update loop
  desired_poses_box_.set(hold_traj_ptr);
}

void EECartImpedControlClass::update()
{
    last_time_ = robot_state_->getTime();

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

    // Maybe move this computation to the pressure observer so we only
    // do it when there's new data
    double fDiff = (myPressureObserver->padForce_left_cur_nonbiased) - (myPressureObserver->padForce_right_cur_nonbiased) - pressDiffZero;

    xerr_prev_ = xerr_;
    // Calculate a Cartesian restoring force.
    xerr_.vel = x_.p - xd_.p;
    xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
            xd_.M.UnitY() * x_.M.UnitY() +
            xd_.M.UnitZ() * x_.M.UnitZ());  // I have no idea what this is
     xerr_diff_ = xerr_ - xerr_prev_;

    // F_ is a vector of forces/wrenches corresponding to x, y, z, tx,ty,tz,tw
    if (desiredPose.isForceX) {
      F_(0) = Fdes_(0);
    } else {
      F_(0) = -Kp_(0) * xerr_(0) - Kd_(0)*xdot_(0);
    }

    if (desiredPose.isForceY) {
      F_(1) = Fdes_(1);
    } else {
      F_(1) = -Kp_(1) * xerr_(1) - Kd_(1)*xdot_(1);
    }

    // MODIFIED! In the tool version, we assume that a desired Z force
    // corresponds to a desired differential between the two fingertip pads.
    // We use a modified P-I controller, where the P-component is proportional
    // to the set-point, rather than the error - this makes sense because the
    // input and output variables are of the same order (two different forces)
    if (desiredPose.isForceZ) {
      F_(2) = KP_PR*Fdes_(2) + KI_PR*pressIntegral + INIT_VERT_F;
      
      double pressErr = fDiff - Fdes_(2);      

      if(F_(2) > MAX_VERT_F){
        F_(2) = MAX_VERT_F;
        
        if(pressErr < 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
      } else if(F_(2) < -MAX_VERT_F) {
        F_(2) = -MAX_VERT_F;
        
        if(pressErr > 0)
        {
          pressIntegral = pressIntegral + pressErr;
        }
      } else {
        pressIntegral = pressIntegral + pressErr;
      }

      // Sentinel for zeroing the fingertip sensors
      if (Fdes_(2) > 20) {
        pressDiffZero = pressDiffZero + fDiff;
        pressIntegral = 0;
        F_(2) = 0;
      }  
      // For debugging
      //F_(2) = 0;      
      
      /*if (!(updates_ % 10)) {
        std::stringstream ss;
        ss << boost::format("Requested vert force: %+.3f I:%+.3f") % F_(2) % pressIntegral;
        talk(ss.str());
      }*/

    } else {
      F_(2) = -Kp_(2) * xerr_(2) - Kd_(2)*xdot_(2);
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
    myPressureObserver->spin();

    //publish the current state
    if (!(updates_ % 10)) {
      std::stringstream ss;
      /*ss << boost::format("Pressure val: L: %+2.4f R: %+2.4f D: %+2.4f Z: %+2.4f") % myPressureObserver->padForce_left_cur_nonbiased % myPressureObserver->padForce_right_cur_nonbiased % fDiff % pressDiffZero;
      talk(ss.str());*/

      KDL::Rotation offRot = xd_.M*(x_.M.Inverse());
      KDL::Vector offAx;
      double offAng = offRot.GetRotAngle(offAx);
      offAx = offAx*offAng;
      
      for(int i = 0; i < 3; i++)
      {
        ss << boost::format("%1.6f %1.6f ") % xerr_(i+3) % xerr_diff_(i+3);
      }

      talk(ss.str());

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
}

void EECartImpedControlClass::stopping() {
  starting();
}


/// Register controller to pluginlib
PLUGINLIB_DECLARE_CLASS(ee_cart_imped_control, EECartImpedControlPlugin,
			ee_cart_imped_control_ns::EECartImpedControlClass,
			pr2_controller_interface::Controller)

