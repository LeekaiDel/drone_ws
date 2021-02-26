;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::DroneInfo)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'DroneInfo (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::DRONEINFO")
  (make-package "DRONE_MSGS::DRONEINFO"))

(in-package "ROS")
;;//! \htmlinclude DroneInfo.msg.html


(defclass drone_msgs::DroneInfo
  :super ros::object
  :slots (_team_num _id_drone _id_marker _health _ip _pose ))

(defmethod drone_msgs::DroneInfo
  (:init
   (&key
    ((:team_num __team_num) 0)
    ((:id_drone __id_drone) 0)
    ((:id_marker __id_marker) 0)
    ((:health __health) 0.0)
    ((:ip __ip) "")
    ((:pose __pose) (instance drone_msgs::DronePose :init))
    )
   (send-super :init)
   (setq _team_num (round __team_num))
   (setq _id_drone (round __id_drone))
   (setq _id_marker (round __id_marker))
   (setq _health (float __health))
   (setq _ip (string __ip))
   (setq _pose __pose)
   self)
  (:team_num
   (&optional __team_num)
   (if __team_num (setq _team_num __team_num)) _team_num)
  (:id_drone
   (&optional __id_drone)
   (if __id_drone (setq _id_drone __id_drone)) _id_drone)
  (:id_marker
   (&optional __id_marker)
   (if __id_marker (setq _id_marker __id_marker)) _id_marker)
  (:health
   (&optional __health)
   (if __health (setq _health __health)) _health)
  (:ip
   (&optional __ip)
   (if __ip (setq _ip __ip)) _ip)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:serialization-length
   ()
   (+
    ;; int8 _team_num
    1
    ;; int8 _id_drone
    1
    ;; int8 _id_marker
    1
    ;; float32 _health
    4
    ;; string _ip
    4 (length _ip)
    ;; drone_msgs/DronePose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _team_num
       (write-byte _team_num s)
     ;; int8 _id_drone
       (write-byte _id_drone s)
     ;; int8 _id_marker
       (write-byte _id_marker s)
     ;; float32 _health
       (sys::poke _health (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; string _ip
       (write-long (length _ip) s) (princ _ip s)
     ;; drone_msgs/DronePose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _team_num
     (setq _team_num (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _team_num 127) (setq _team_num (- _team_num 256)))
   ;; int8 _id_drone
     (setq _id_drone (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id_drone 127) (setq _id_drone (- _id_drone 256)))
   ;; int8 _id_marker
     (setq _id_marker (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id_marker 127) (setq _id_marker (- _id_marker 256)))
   ;; float32 _health
     (setq _health (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; string _ip
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _ip (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; drone_msgs/DronePose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get drone_msgs::DroneInfo :md5sum-) "142d303be68c1b2c6c6b79486ae3db7b")
(setf (get drone_msgs::DroneInfo :datatype-) "drone_msgs/DroneInfo")
(setf (get drone_msgs::DroneInfo :definition-)
      "int8 team_num   # number of team (0,1..n)
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



(provide :drone_msgs/DroneInfo "142d303be68c1b2c6c6b79486ae3db7b")


