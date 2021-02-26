
(cl:in-package :asdf)

(defsystem "drone_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Diagnostics" :depends-on ("_package_Diagnostics"))
    (:file "_package_Diagnostics" :depends-on ("_package"))
    (:file "DroneInfo" :depends-on ("_package_DroneInfo"))
    (:file "_package_DroneInfo" :depends-on ("_package"))
    (:file "DroneInfoArray" :depends-on ("_package_DroneInfoArray"))
    (:file "_package_DroneInfoArray" :depends-on ("_package"))
    (:file "DronePose" :depends-on ("_package_DronePose"))
    (:file "_package_DronePose" :depends-on ("_package"))
    (:file "Goal" :depends-on ("_package_Goal"))
    (:file "_package_Goal" :depends-on ("_package"))
    (:file "LocalPlannerState" :depends-on ("_package_LocalPlannerState"))
    (:file "_package_LocalPlannerState" :depends-on ("_package"))
    (:file "RoomParams" :depends-on ("_package_RoomParams"))
    (:file "_package_RoomParams" :depends-on ("_package"))
    (:file "Strike" :depends-on ("_package_Strike"))
    (:file "_package_Strike" :depends-on ("_package"))
    (:file "WindowAngleDir" :depends-on ("_package_WindowAngleDir"))
    (:file "_package_WindowAngleDir" :depends-on ("_package"))
    (:file "WindowPointDir" :depends-on ("_package_WindowPointDir"))
    (:file "_package_WindowPointDir" :depends-on ("_package"))
  ))