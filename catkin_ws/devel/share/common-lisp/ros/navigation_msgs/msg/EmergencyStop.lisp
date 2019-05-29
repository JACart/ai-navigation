; Auto-generated. Do not edit!


(cl:in-package navigation_msgs-msg)


;//! \htmlinclude EmergencyStop.msg.html

(cl:defclass <EmergencyStop> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (emergency_stop
    :reader emergency_stop
    :initarg :emergency_stop
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass EmergencyStop (<EmergencyStop>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <EmergencyStop>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'EmergencyStop)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation_msgs-msg:<EmergencyStop> is deprecated: use navigation_msgs-msg:EmergencyStop instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_msgs-msg:header-val is deprecated.  Use navigation_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'emergency_stop-val :lambda-list '(m))
(cl:defmethod emergency_stop-val ((m <EmergencyStop>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_msgs-msg:emergency_stop-val is deprecated.  Use navigation_msgs-msg:emergency_stop instead.")
  (emergency_stop m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <EmergencyStop>) ostream)
  "Serializes a message object of type '<EmergencyStop>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'emergency_stop) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <EmergencyStop>) istream)
  "Deserializes a message object of type '<EmergencyStop>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'emergency_stop) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<EmergencyStop>)))
  "Returns string type for a message object of type '<EmergencyStop>"
  "navigation_msgs/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'EmergencyStop)))
  "Returns string type for a message object of type 'EmergencyStop"
  "navigation_msgs/EmergencyStop")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<EmergencyStop>)))
  "Returns md5sum for a message object of type '<EmergencyStop>"
  "f0ed97528fd0e784408fb431ee7d153c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'EmergencyStop)))
  "Returns md5sum for a message object of type 'EmergencyStop"
  "f0ed97528fd0e784408fb431ee7d153c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<EmergencyStop>)))
  "Returns full string definition for message of type '<EmergencyStop>"
  (cl:format cl:nil "std_msgs/Header header~%bool emergency_stop~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'EmergencyStop)))
  "Returns full string definition for message of type 'EmergencyStop"
  (cl:format cl:nil "std_msgs/Header header~%bool emergency_stop~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <EmergencyStop>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <EmergencyStop>))
  "Converts a ROS message object to a list"
  (cl:list 'EmergencyStop
    (cl:cons ':header (header msg))
    (cl:cons ':emergency_stop (emergency_stop msg))
))
