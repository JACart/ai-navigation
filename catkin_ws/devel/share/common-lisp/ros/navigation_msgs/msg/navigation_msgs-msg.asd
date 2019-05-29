
(cl:in-package :asdf)

(defsystem "navigation_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "EmergencyStop" :depends-on ("_package_EmergencyStop"))
    (:file "_package_EmergencyStop" :depends-on ("_package"))
    (:file "FErequest" :depends-on ("_package_FErequest"))
    (:file "_package_FErequest" :depends-on ("_package"))
    (:file "LatLongPoint" :depends-on ("_package_LatLongPoint"))
    (:file "_package_LatLongPoint" :depends-on ("_package"))
    (:file "VelAngle" :depends-on ("_package_VelAngle"))
    (:file "_package_VelAngle" :depends-on ("_package"))
    (:file "WaypointsArray" :depends-on ("_package_WaypointsArray"))
    (:file "_package_WaypointsArray" :depends-on ("_package"))
  ))