; Auto-generated. Do not edit!


(cl:in-package ee_cart_imped_msgs-msg)


;//! \htmlinclude StiffPoint.msg.html

(cl:defclass <StiffPoint> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (wrench_or_stiffness
    :reader wrench_or_stiffness
    :initarg :wrench_or_stiffness
    :type geometry_msgs-msg:Wrench
    :initform (cl:make-instance 'geometry_msgs-msg:Wrench))
   (isForceX
    :reader isForceX
    :initarg :isForceX
    :type cl:boolean
    :initform cl:nil)
   (isForceY
    :reader isForceY
    :initarg :isForceY
    :type cl:boolean
    :initform cl:nil)
   (isForceZ
    :reader isForceZ
    :initarg :isForceZ
    :type cl:boolean
    :initform cl:nil)
   (isTorqueX
    :reader isTorqueX
    :initarg :isTorqueX
    :type cl:boolean
    :initform cl:nil)
   (isTorqueY
    :reader isTorqueY
    :initarg :isTorqueY
    :type cl:boolean
    :initform cl:nil)
   (isTorqueZ
    :reader isTorqueZ
    :initarg :isTorqueZ
    :type cl:boolean
    :initform cl:nil)
   (time_from_start
    :reader time_from_start
    :initarg :time_from_start
    :type cl:real
    :initform 0))
)

(cl:defclass StiffPoint (<StiffPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StiffPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StiffPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ee_cart_imped_msgs-msg:<StiffPoint> is deprecated: use ee_cart_imped_msgs-msg:StiffPoint instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:header-val is deprecated.  Use ee_cart_imped_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:pose-val is deprecated.  Use ee_cart_imped_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'wrench_or_stiffness-val :lambda-list '(m))
(cl:defmethod wrench_or_stiffness-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:wrench_or_stiffness-val is deprecated.  Use ee_cart_imped_msgs-msg:wrench_or_stiffness instead.")
  (wrench_or_stiffness m))

(cl:ensure-generic-function 'isForceX-val :lambda-list '(m))
(cl:defmethod isForceX-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isForceX-val is deprecated.  Use ee_cart_imped_msgs-msg:isForceX instead.")
  (isForceX m))

(cl:ensure-generic-function 'isForceY-val :lambda-list '(m))
(cl:defmethod isForceY-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isForceY-val is deprecated.  Use ee_cart_imped_msgs-msg:isForceY instead.")
  (isForceY m))

(cl:ensure-generic-function 'isForceZ-val :lambda-list '(m))
(cl:defmethod isForceZ-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isForceZ-val is deprecated.  Use ee_cart_imped_msgs-msg:isForceZ instead.")
  (isForceZ m))

(cl:ensure-generic-function 'isTorqueX-val :lambda-list '(m))
(cl:defmethod isTorqueX-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isTorqueX-val is deprecated.  Use ee_cart_imped_msgs-msg:isTorqueX instead.")
  (isTorqueX m))

(cl:ensure-generic-function 'isTorqueY-val :lambda-list '(m))
(cl:defmethod isTorqueY-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isTorqueY-val is deprecated.  Use ee_cart_imped_msgs-msg:isTorqueY instead.")
  (isTorqueY m))

(cl:ensure-generic-function 'isTorqueZ-val :lambda-list '(m))
(cl:defmethod isTorqueZ-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:isTorqueZ-val is deprecated.  Use ee_cart_imped_msgs-msg:isTorqueZ instead.")
  (isTorqueZ m))

(cl:ensure-generic-function 'time_from_start-val :lambda-list '(m))
(cl:defmethod time_from_start-val ((m <StiffPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ee_cart_imped_msgs-msg:time_from_start-val is deprecated.  Use ee_cart_imped_msgs-msg:time_from_start instead.")
  (time_from_start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StiffPoint>) ostream)
  "Serializes a message object of type '<StiffPoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'wrench_or_stiffness) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isForceX) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isForceY) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isForceZ) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isTorqueX) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isTorqueY) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'isTorqueZ) 1 0)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'time_from_start)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'time_from_start) (cl:floor (cl:slot-value msg 'time_from_start)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StiffPoint>) istream)
  "Deserializes a message object of type '<StiffPoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'wrench_or_stiffness) istream)
    (cl:setf (cl:slot-value msg 'isForceX) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isForceY) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isForceZ) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isTorqueX) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isTorqueY) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'isTorqueZ) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time_from_start) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StiffPoint>)))
  "Returns string type for a message object of type '<StiffPoint>"
  "ee_cart_imped_msgs/StiffPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StiffPoint)))
  "Returns string type for a message object of type 'StiffPoint"
  "ee_cart_imped_msgs/StiffPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StiffPoint>)))
  "Returns md5sum for a message object of type '<StiffPoint>"
  "2b6b597656e805a0ca2e1de07dc31a66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StiffPoint)))
  "Returns md5sum for a message object of type 'StiffPoint"
  "2b6b597656e805a0ca2e1de07dc31a66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StiffPoint>)))
  "Returns full string definition for message of type '<StiffPoint>"
  (cl:format cl:nil "Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StiffPoint)))
  "Returns full string definition for message of type 'StiffPoint"
  (cl:format cl:nil "Header header~%#The pose to achieve in the stiffness directions~%geometry_msgs/Pose pose~%#Wrench or stiffness for each dimension~%geometry_msgs/Wrench wrench_or_stiffness~%#The following are True if a force/torque should~%#be exerted and False if a stiffness should be used.~%bool isForceX~%bool isForceY~%bool isForceZ~%bool isTorqueX~%bool isTorqueY~%bool isTorqueZ~%#The time from the start of the trajectory that this~%#point should be achieved.~%duration time_from_start~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Wrench~%# This represents force in free space, seperated into ~%# it's linear and angular parts.  ~%Vector3  force~%Vector3  torque~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StiffPoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'wrench_or_stiffness))
     1
     1
     1
     1
     1
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StiffPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'StiffPoint
    (cl:cons ':header (header msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':wrench_or_stiffness (wrench_or_stiffness msg))
    (cl:cons ':isForceX (isForceX msg))
    (cl:cons ':isForceY (isForceY msg))
    (cl:cons ':isForceZ (isForceZ msg))
    (cl:cons ':isTorqueX (isTorqueX msg))
    (cl:cons ':isTorqueY (isTorqueY msg))
    (cl:cons ':isTorqueZ (isTorqueZ msg))
    (cl:cons ':time_from_start (time_from_start msg))
))
