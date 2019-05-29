;; Auto-generated. Do not edit!


(when (boundp 'navigation_msgs::FErequest)
  (if (not (find-package "NAVIGATION_MSGS"))
    (make-package "NAVIGATION_MSGS"))
  (shadow 'FErequest (find-package "NAVIGATION_MSGS")))
(unless (find-package "NAVIGATION_MSGS::FEREQUEST")
  (make-package "NAVIGATION_MSGS::FEREQUEST"))

(in-package "ROS")
;;//! \htmlinclude FErequest.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass navigation_msgs::FErequest
  :super ros::object
  :slots (_starting_location _final_location _header ))

(defmethod navigation_msgs::FErequest
  (:init
   (&key
    ((:starting_location __starting_location) "")
    ((:final_location __final_location) "")
    ((:header __header) (instance std_msgs::Header :init))
    )
   (send-super :init)
   (setq _starting_location (string __starting_location))
   (setq _final_location (string __final_location))
   (setq _header __header)
   self)
  (:starting_location
   (&optional __starting_location)
   (if __starting_location (setq _starting_location __starting_location)) _starting_location)
  (:final_location
   (&optional __final_location)
   (if __final_location (setq _final_location __final_location)) _final_location)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:serialization-length
   ()
   (+
    ;; string _starting_location
    4 (length _starting_location)
    ;; string _final_location
    4 (length _final_location)
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _starting_location
       (write-long (length _starting_location) s) (princ _starting_location s)
     ;; string _final_location
       (write-long (length _final_location) s) (princ _final_location s)
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _starting_location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _starting_location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _final_location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _final_location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;;
   self)
  )

(setf (get navigation_msgs::FErequest :md5sum-) "90e91925255dbc49ba4bb3349ffc2807")
(setf (get navigation_msgs::FErequest :datatype-) "navigation_msgs/FErequest")
(setf (get navigation_msgs::FErequest :definition-)
      "string starting_location
string final_location
std_msgs/Header header

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



(provide :navigation_msgs/FErequest "90e91925255dbc49ba4bb3349ffc2807")


