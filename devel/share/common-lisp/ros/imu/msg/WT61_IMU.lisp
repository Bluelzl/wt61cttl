; Auto-generated. Do not edit!


(cl:in-package imu-msg)


;//! \htmlinclude WT61_IMU.msg.html

(cl:defclass <WT61_IMU> (roslisp-msg-protocol:ros-message)
  ((Header
    :reader Header
    :initarg :Header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (angular_velocity
    :reader angular_velocity
    :initarg :angular_velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (linear_acceleration
    :reader linear_acceleration
    :initarg :linear_acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular
    :reader angular
    :initarg :angular
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass WT61_IMU (<WT61_IMU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WT61_IMU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WT61_IMU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name imu-msg:<WT61_IMU> is deprecated: use imu-msg:WT61_IMU instead.")))

(cl:ensure-generic-function 'Header-val :lambda-list '(m))
(cl:defmethod Header-val ((m <WT61_IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:Header-val is deprecated.  Use imu-msg:Header instead.")
  (Header m))

(cl:ensure-generic-function 'angular_velocity-val :lambda-list '(m))
(cl:defmethod angular_velocity-val ((m <WT61_IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:angular_velocity-val is deprecated.  Use imu-msg:angular_velocity instead.")
  (angular_velocity m))

(cl:ensure-generic-function 'linear_acceleration-val :lambda-list '(m))
(cl:defmethod linear_acceleration-val ((m <WT61_IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:linear_acceleration-val is deprecated.  Use imu-msg:linear_acceleration instead.")
  (linear_acceleration m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <WT61_IMU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader imu-msg:angular-val is deprecated.  Use imu-msg:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WT61_IMU>) ostream)
  "Serializes a message object of type '<WT61_IMU>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_acceleration) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WT61_IMU>) istream)
  "Deserializes a message object of type '<WT61_IMU>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_acceleration) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WT61_IMU>)))
  "Returns string type for a message object of type '<WT61_IMU>"
  "imu/WT61_IMU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WT61_IMU)))
  "Returns string type for a message object of type 'WT61_IMU"
  "imu/WT61_IMU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WT61_IMU>)))
  "Returns md5sum for a message object of type '<WT61_IMU>"
  "4f1af74405a72a71606218daaacc3332")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WT61_IMU)))
  "Returns md5sum for a message object of type 'WT61_IMU"
  "4f1af74405a72a71606218daaacc3332")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WT61_IMU>)))
  "Returns full string definition for message of type '<WT61_IMU>"
  (cl:format cl:nil "Header Header~%~%geometry_msgs/Vector3 angular_velocity~%~%geometry_msgs/Vector3 linear_acceleration~%~%geometry_msgs/Vector3 angular~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WT61_IMU)))
  "Returns full string definition for message of type 'WT61_IMU"
  (cl:format cl:nil "Header Header~%~%geometry_msgs/Vector3 angular_velocity~%~%geometry_msgs/Vector3 linear_acceleration~%~%geometry_msgs/Vector3 angular~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WT61_IMU>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_acceleration))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WT61_IMU>))
  "Converts a ROS message object to a list"
  (cl:list 'WT61_IMU
    (cl:cons ':Header (Header msg))
    (cl:cons ':angular_velocity (angular_velocity msg))
    (cl:cons ':linear_acceleration (linear_acceleration msg))
    (cl:cons ':angular (angular msg))
))
