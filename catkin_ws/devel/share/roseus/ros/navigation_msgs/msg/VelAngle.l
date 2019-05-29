;; Auto-generated. Do not edit!


(when (boundp 'navigation_msgs::VelAngle)
  (if (not (find-package "NAVIGATION_MSGS"))
    (make-package "NAVIGATION_MSGS"))
  (shadow 'VelAngle (find-package "NAVIGATION_MSGS")))
(unless (find-package "NAVIGATION_MSGS::VELANGLE")
  (make-package "NAVIGATION_MSGS::VELANGLE"))

(in-package "ROS")
;;//! \htmlinclude VelAngle.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass navigation_msgs::VelAngle
  :super ros::object
  :slots (_header _vel _angle _vel_curr ))

(defmethod navigation_msgs::VelAngle
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:vel __vel) 0.0)
    ((:angle __angle) 0.0)
    ((:vel_curr __vel_curr) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _vel (float __vel))
   (setq _angle (float __angle))
   (setq _vel_curr (float __vel_curr))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:vel_curr
   (&optional __vel_curr)
   (if __vel_curr (setq _vel_curr __vel_curr)) _vel_curr)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _vel
    8
    ;; float64 _angle
    8
    ;; float64 _vel_curr
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _vel_curr
       (sys::poke _vel_curr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _vel
     (setq _vel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _vel_curr
     (setq _vel_curr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get navigation_msgs::VelAngle :md5sum-) "b6d78569f7e4cb35a5565b9ea70cb7e5")
(setf (get navigation_msgs::VelAngle :datatype-) "navigation_msgs/VelAngle")
(setf (get navigation_msgs::VelAngle :definition-)
      "std_msgs/Header header
float64 vel
float64 angle
float64 vel_curr

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



(provide :navigation_msgs/VelAngle "b6d78569f7e4cb35a5565b9ea70cb7e5")


