; Auto-generated. Do not edit!


(cl:in-package navigation_msgs-msg)


;//! \htmlinclude FErequest.msg.html

(cl:defclass <FErequest> (roslisp-msg-protocol:ros-message)
  ((starting_location
    :reader starting_location
    :initarg :starting_location
    :type cl:string
    :initform "")
   (final_location
    :reader final_location
    :initarg :final_location
    :type cl:string
    :initform "")
   (header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header)))
)

(cl:defclass FErequest (<FErequest>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FErequest>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FErequest)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation_msgs-msg:<FErequest> is deprecated: use navigation_msgs-msg:FErequest instead.")))

(cl:ensure-generic-function 'starting_location-val :lambda-list '(m))
(cl:defmethod starting_location-val ((m <FErequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_msgs-msg:starting_location-val is deprecated.  Use navigation_msgs-msg:starting_location instead.")
  (starting_location m))

(cl:ensure-generic-function 'final_location-val :lambda-list '(m))
(cl:defmethod final_location-val ((m <FErequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_msgs-msg:final_location-val is deprecated.  Use navigation_msgs-msg:final_location instead.")
  (final_location m))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FErequest>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_msgs-msg:header-val is deprecated.  Use navigation_msgs-msg:header instead.")
  (header m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FErequest>) ostream)
  "Serializes a message object of type '<FErequest>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'starting_location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'starting_location))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'final_location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'final_location))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FErequest>) istream)
  "Deserializes a message object of type '<FErequest>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'starting_location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'starting_location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'final_location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'final_location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FErequest>)))
  "Returns string type for a message object of type '<FErequest>"
  "navigation_msgs/FErequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FErequest)))
  "Returns string type for a message object of type 'FErequest"
  "navigation_msgs/FErequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FErequest>)))
  "Returns md5sum for a message object of type '<FErequest>"
  "90e91925255dbc49ba4bb3349ffc2807")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FErequest)))
  "Returns md5sum for a message object of type 'FErequest"
  "90e91925255dbc49ba4bb3349ffc2807")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FErequest>)))
  "Returns full string definition for message of type '<FErequest>"
  (cl:format cl:nil "string starting_location~%string final_location~%std_msgs/Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FErequest)))
  "Returns full string definition for message of type 'FErequest"
  (cl:format cl:nil "string starting_location~%string final_location~%std_msgs/Header header~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FErequest>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'starting_location))
     4 (cl:length (cl:slot-value msg 'final_location))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FErequest>))
  "Converts a ROS message object to a list"
  (cl:list 'FErequest
    (cl:cons ':starting_location (starting_location msg))
    (cl:cons ':final_location (final_location msg))
    (cl:cons ':header (header msg))
))
