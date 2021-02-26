;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::Goal)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'Goal (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::GOAL")
  (make-package "DRONE_MSGS::GOAL"))

(in-package "ROS")
;;//! \htmlinclude Goal.msg.html


(intern "*POSE*" (find-package "DRONE_MSGS::GOAL"))
(shadow '*POSE* (find-package "DRONE_MSGS::GOAL"))
(defconstant drone_msgs::Goal::*POSE* 0)
(intern "*VEL*" (find-package "DRONE_MSGS::GOAL"))
(shadow '*VEL* (find-package "DRONE_MSGS::GOAL"))
(defconstant drone_msgs::Goal::*VEL* 1)
(defclass drone_msgs::Goal
  :super ros::object
  :slots (_ctr_type _pose ))

(defmethod drone_msgs::Goal
  (:init
   (&key
    ((:ctr_type __ctr_type) 0)
    ((:pose __pose) (instance drone_msgs::DronePose :init))
    )
   (send-super :init)
   (setq _ctr_type (round __ctr_type))
   (setq _pose __pose)
   self)
  (:ctr_type
   (&optional __ctr_type)
   (if __ctr_type (setq _ctr_type __ctr_type)) _ctr_type)
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
    ;; byte _ctr_type
    1
    ;; drone_msgs/DronePose _pose
    (send _pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; byte _ctr_type
       (write-byte _ctr_type s)
     ;; drone_msgs/DronePose _pose
       (send _pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; byte _ctr_type
     (setq _ctr_type (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _ctr_type 127) (setq _ctr_type (- _ctr_type 256)))
   ;; drone_msgs/DronePose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;;
   self)
  )

(setf (get drone_msgs::Goal :md5sum-) "bf6e29cec64ab1c71dda19cb2e5b5f60")
(setf (get drone_msgs::Goal :datatype-) "drone_msgs/Goal")
(setf (get drone_msgs::Goal :definition-)
      "byte POSE=0
byte VEL=1

byte ctr_type
drone_msgs/DronePose pose

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



(provide :drone_msgs/Goal "bf6e29cec64ab1c71dda19cb2e5b5f60")


