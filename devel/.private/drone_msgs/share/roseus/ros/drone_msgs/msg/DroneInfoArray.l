;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::DroneInfoArray)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'DroneInfoArray (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::DRONEINFOARRAY")
  (make-package "DRONE_MSGS::DRONEINFOARRAY"))

(in-package "ROS")
;;//! \htmlinclude DroneInfoArray.msg.html


(defclass drone_msgs::DroneInfoArray
  :super ros::object
  :slots (_drones ))

(defmethod drone_msgs::DroneInfoArray
  (:init
   (&key
    ((:drones __drones) (let (r) (dotimes (i 0) (push (instance drone_msgs::DroneInfo :init) r)) r))
    )
   (send-super :init)
   (setq _drones __drones)
   self)
  (:drones
   (&rest __drones)
   (if (keywordp (car __drones))
       (send* _drones __drones)
     (progn
       (if __drones (setq _drones (car __drones)))
       _drones)))
  (:serialization-length
   ()
   (+
    ;; drone_msgs/DroneInfo[] _drones
    (apply #'+ (send-all _drones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; drone_msgs/DroneInfo[] _drones
     (write-long (length _drones) s)
     (dolist (elem _drones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; drone_msgs/DroneInfo[] _drones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _drones (let (r) (dotimes (i n) (push (instance drone_msgs::DroneInfo :init) r)) r))
     (dolist (elem- _drones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get drone_msgs::DroneInfoArray :md5sum-) "b567d9a3a60bda150b09c0bcd10bfc14")
(setf (get drone_msgs::DroneInfoArray :datatype-) "drone_msgs/DroneInfoArray")
(setf (get drone_msgs::DroneInfoArray :definition-)
      "drone_msgs/DroneInfo[] drones
================================================================================
MSG: drone_msgs/DroneInfo
int8 team_num   # number of team (0,1..n)
int8 id_drone   # id of drone (0,1..n)
int8 id_marker  # number of marker id
float32 health  # health of drone (100%..0%)
string ip
drone_msgs/DronePose pose   # ENU position of drone

================================================================================
MSG: drone_msgs/DronePose
geometry_msgs/Point point
float32 course

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :drone_msgs/DroneInfoArray "b567d9a3a60bda150b09c0bcd10bfc14")


