;; Auto-generated. Do not edit!


(when (boundp 'navigation_msgs::EmergencyStop)
  (if (not (find-package "NAVIGATION_MSGS"))
    (make-package "NAVIGATION_MSGS"))
  (shadow 'EmergencyStop (find-package "NAVIGATION_MSGS")))
(unless (find-package "NAVIGATION_MSGS::EMERGENCYSTOP")
  (make-package "NAVIGATION_MSGS::EMERGENCYSTOP"))

(in-package "ROS")
;;//! \htmlinclude EmergencyStop.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass navigation_msgs::EmergencyStop
  :super ros::object
  :slots (_header _emergency_stop ))

(defmethod navigation_msgs::EmergencyStop
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:emergency_stop __emergency_stop) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _emergency_stop __emergency_stop)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:emergency_stop
   (&optional __emergency_stop)
   (if __emergency_stop (setq _emergency_stop __emergency_stop)) _emergency_stop)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _emergency_stop
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _emergency_stop
       (if _emergency_stop (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _emergency_stop
     (setq _emergency_stop (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get navigation_msgs::EmergencyStop :md5sum-) "f0ed97528fd0e784408fb431ee7d153c")
(setf (get navigation_msgs::EmergencyStop :datatype-) "navigation_msgs/EmergencyStop")
(setf (get navigation_msgs::EmergencyStop :definition-)
      "std_msgs/Header header
bool emergency_stop

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

")



(provide :navigation_msgs/EmergencyStop "f0ed97528fd0e784408fb431ee7d153c")


