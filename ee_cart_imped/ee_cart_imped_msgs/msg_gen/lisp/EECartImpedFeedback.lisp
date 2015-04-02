; Auto-generated. Do not edit!


(cl:in-package ee_cart_imped_msgs-msg)


;//! \htmlinclude EECartImpedFeedback.msg.html

(cl:defclass <EECartImpedFeedback> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal
    :reader goal
    :initarg :goal
    :type (cl:vector ee_cart_imped_msgs-msg:StiffPoint)
   :initform (cl:make-array 0 :element-type 'ee_cart_imped_msgs-msg:StiffPoint :initial-element (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint)))
   (initial_point
    :reader initial_point
    :initarg :initial_point
    :type ee_cart_imped_msgs-msg:StiffPoint
    :initform (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint))
   (desired
    :reader desired
    :initarg :desired
    :type ee_cart_imped_msgs-msg:StiffPoint
    :initform (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint))
   (actual_pose
    :reader actual_pose
    :initarg :actual_pose
    :type ee_cart_imped_msgs-msg:StiffPoint
    :initform (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint))
   (effort_sq_error
    :reader effort_sq_error
    :initarg :effort_sq_error
    :type cl:float
    :initform 0.0)
   (requested_joint_efforts
    :reader requested_joint_efforts
    :initarg :requested_joint_efforts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (actual_joint_efforts
    :reader actual_joint_efforts
    :initarg :actual_joint_efforts
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (running_time
    :reader running_time
    :initarg :running_time
    :type cl:real
    :initform 0))
)

(cl:defclass EECartImpedFeedback (<EECartImpedFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EECartImpedFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EECartImpedFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee_cart_imped_msgs-msg:<EECartImpedFeedback> is deprecated: use ee_cart_imped_msgs-msg:EECartImpedFeedback instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:header-val is deprecated.  Use ee_cart_imped_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:goal-val is deprecated.  Use ee_cart_imped_msgs-msg:goal instead.")
  (goal m))

(cl:ensure-generic-function 'initial_point-val :lambda-list '(m))
(cl:defmethod initial_point-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:initial_point-val is deprecated.  Use ee_cart_imped_msgs-msg:initial_point instead.")
  (initial_point m))

(cl:ensure-generic-function 'desired-val :lambda-list '(m))
(cl:defmethod desired-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:desired-val is deprecated.  Use ee_cart_imped_msgs-msg:desired instead.")
  (desired m))

(cl:ensure-generic-function 'actual_pose-val :lambda-list '(m))
(cl:defmethod actual_pose-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:actual_pose-val is deprecated.  Use ee_cart_imped_msgs-msg:actual_pose instead.")
  (actual_pose m))

(cl:ensure-generic-function 'effort_sq_error-val :lambda-list '(m))
(cl:defmethod effort_sq_error-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:effort_sq_error-val is deprecated.  Use ee_cart_imped_msgs-msg:effort_sq_error instead.")
  (effort_sq_error m))

(cl:ensure-generic-function 'requested_joint_efforts-val :lambda-list '(m))
(cl:defmethod requested_joint_efforts-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:requested_joint_efforts-val is deprecated.  Use ee_cart_imped_msgs-msg:requested_joint_efforts instead.")
  (requested_joint_efforts m))

(cl:ensure-generic-function 'actual_joint_efforts-val :lambda-list '(m))
(cl:defmethod actual_joint_efforts-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:actual_joint_efforts-val is deprecated.  Use ee_cart_imped_msgs-msg:actual_joint_efforts instead.")
  (actual_joint_efforts m))

(cl:ensure-generic-function 'running_time-val :lambda-list '(m))
(cl:defmethod running_time-val ((m <EECartImpedFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:running_time-val is deprecated.  Use ee_cart_imped_msgs-msg:running_time instead.")
  (running_time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EECartImpedFeedback>) ostream)
  "Serializes a message object of type '<EECartImpedFeedback>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'goal))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'goal))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'initial_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'desired) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'actual_pose) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'effort_sq_error))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'requested_joint_efforts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'requested_joint_efforts))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'actual_joint_efforts))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'actual_joint_efforts))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'running_time)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'running_time) (cl:floor (cl:slot-value msg 'running_time)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EECartImpedFeedback>) istream)
  "Deserializes a message object of type '<EECartImpedFeedback>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'goal) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'goal)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'initial_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'desired) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'actual_pose) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'effort_sq_error) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'requested_joint_efforts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'requested_joint_efforts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'actual_joint_efforts) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'actual_joint_efforts)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'running_time) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EECartImpedFeedback>)))
  "Returns string type for a message object of type '<EECartImpedFeedback>"
  "ee_cart_imped_msgs/EECartImpedFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECartImpedFeedback)))
  "Returns string type for a message object of type 'EECartImpedFeedback"
  "ee_cart_imped_msgs/EECartImpedFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EECartImpedFeedback>)))
  "Returns md5sum for a message object of type '<EECartImpedFeedback>"
  "4106b02683301dac2003809bdf610591")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EECartImpedFeedback)))
  "Returns md5sum for a message object of type 'EECartImpedFeedback"
  "4106b02683301dac2003809bdf610591")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EECartImpedFeedback>)))
  "Returns full string definition for message of type '<EECartImpedFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#current pose and squared error in force ~%#and the running time of this goal so far~%Header header~%ee_cart_imped_msgs/StiffPoint[] goal~%ee_cart_imped_msgs/StiffPoint initial_point~%ee_cart_imped_msgs/StiffPoint desired~%ee_cart_imped_msgs/StiffPoint actual_pose~%float64 effort_sq_error~%float64[] requested_joint_efforts~%float64[] actual_joint_efforts~%duration running_time~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ee_cart_imped_msgs/StiffPoint~%Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EECartImpedFeedback)))
  "Returns full string definition for message of type 'EECartImpedFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%#current pose and squared error in force ~%#and the running time of this goal so far~%Header header~%ee_cart_imped_msgs/StiffPoint[] goal~%ee_cart_imped_msgs/StiffPoint initial_point~%ee_cart_imped_msgs/StiffPoint desired~%ee_cart_imped_msgs/StiffPoint actual_pose~%float64 effort_sq_error~%float64[] requested_joint_efforts~%float64[] actual_joint_efforts~%duration running_time~%~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ee_cart_imped_msgs/StiffPoint~%Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EECartImpedFeedback>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'goal) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'initial_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actual_pose))
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'requested_joint_efforts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'actual_joint_efforts) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EECartImpedFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'EECartImpedFeedback
    (cl:cons ':header (header msg))
    (cl:cons ':goal (goal msg))
    (cl:cons ':initial_point (initial_point msg))
    (cl:cons ':desired (desired msg))
    (cl:cons ':actual_pose (actual_pose msg))
    (cl:cons ':effort_sq_error (effort_sq_error msg))
    (cl:cons ':requested_joint_efforts (requested_joint_efforts msg))
    (cl:cons ':actual_joint_efforts (actual_joint_efforts msg))
    (cl:cons ':running_time (running_time msg))
))
