; Auto-generated. Do not edit!


(cl:in-package ee_cart_imped_msgs-msg)


;//! \htmlinclude EECartImpedResult.msg.html

(cl:defclass <EECartImpedResult> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (desired
    :reader desired
    :initarg :desired
    :type ee_cart_imped_msgs-msg:StiffPoint
    :initform (cl:make-instance 'ee_cart_imped_msgs-msg:StiffPoint))
   (actual_pose
    :reader actual_pose
    :initarg :actual_pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (effort_sq_error
    :reader effort_sq_error
    :initarg :effort_sq_error
    :type cl:float
    :initform 0.0))
)

(cl:defclass EECartImpedResult (<EECartImpedResult>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EECartImpedResult>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EECartImpedResult)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee_cart_imped_msgs-msg:<EECartImpedResult> is deprecated: use ee_cart_imped_msgs-msg:EECartImpedResult instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EECartImpedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:header-val is deprecated.  Use ee_cart_imped_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <EECartImpedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:success-val is deprecated.  Use ee_cart_imped_msgs-msg:success instead.")
  (success m))

(cl:ensure-generic-function 'desired-val :lambda-list '(m))
(cl:defmethod desired-val ((m <EECartImpedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:desired-val is deprecated.  Use ee_cart_imped_msgs-msg:desired instead.")
  (desired m))

(cl:ensure-generic-function 'actual_pose-val :lambda-list '(m))
(cl:defmethod actual_pose-val ((m <EECartImpedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:actual_pose-val is deprecated.  Use ee_cart_imped_msgs-msg:actual_pose instead.")
  (actual_pose m))

(cl:ensure-generic-function 'effort_sq_error-val :lambda-list '(m))
(cl:defmethod effort_sq_error-val ((m <EECartImpedResult>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:effort_sq_error-val is deprecated.  Use ee_cart_imped_msgs-msg:effort_sq_error instead.")
  (effort_sq_error m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EECartImpedResult>) ostream)
  "Serializes a message object of type '<EECartImpedResult>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EECartImpedResult>) istream)
  "Deserializes a message object of type '<EECartImpedResult>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EECartImpedResult>)))
  "Returns string type for a message object of type '<EECartImpedResult>"
  "ee_cart_imped_msgs/EECartImpedResult")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EECartImpedResult)))
  "Returns string type for a message object of type 'EECartImpedResult"
  "ee_cart_imped_msgs/EECartImpedResult")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EECartImpedResult>)))
  "Returns md5sum for a message object of type '<EECartImpedResult>"
  "947387aff8267ef3d7c884c07783eb7d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EECartImpedResult)))
  "Returns md5sum for a message object of type 'EECartImpedResult"
  "947387aff8267ef3d7c884c07783eb7d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EECartImpedResult>)))
  "Returns full string definition for message of type '<EECartImpedResult>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%#whether it was successful~%#the pose and force we ended with~%Header header~%bool success~%ee_cart_imped_msgs/StiffPoint desired~%geometry_msgs/Pose actual_pose~%float64 effort_sq_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ee_cart_imped_msgs/StiffPoint~%Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EECartImpedResult)))
  "Returns full string definition for message of type 'EECartImpedResult"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result definition~%#whether it was successful~%#the pose and force we ended with~%Header header~%bool success~%ee_cart_imped_msgs/StiffPoint desired~%geometry_msgs/Pose actual_pose~%float64 effort_sq_error~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: ee_cart_imped_msgs/StiffPoint~%Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EECartImpedResult>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'desired))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'actual_pose))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EECartImpedResult>))
  "Converts a ROS message object to a list"
  (cl:list 'EECartImpedResult
    (cl:cons ':header (header msg))
    (cl:cons ':success (success msg))
    (cl:cons ':desired (desired msg))
    (cl:cons ':actual_pose (actual_pose msg))
    (cl:cons ':effort_sq_error (effort_sq_error msg))
))
