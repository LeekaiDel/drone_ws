;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::DronePose)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'DronePose (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::DRONEPOSE")
  (make-package "DRONE_MSGS::DRONEPOSE"))

(in-package "ROS")
;;//! \htmlinclude DronePose.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass drone_msgs::DronePose
  :super ros::object
  :slots (_point _course ))

(defmethod drone_msgs::DronePose
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:course __course) 0.0)
    )
   (send-super :init)
   (setq _point __point)
   (setq _course (float __course))
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:course
   (&optional __course)
   (if __course (setq _course __course)) _course)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; float32 _course
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; float32 _course
       (sys::poke _course (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; float32 _course
     (setq _course (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone_msgs::DronePose :md5sum-) "c3922f772bbf0305ac9710fa392aba5a")
(setf (get drone_msgs::DronePose :datatype-) "drone_msgs/DronePose")
(setf (get drone_msgs::DronePose :definition-)
      "geometry_msgs/Point point
float32 course

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :drone_msgs/DronePose "c3922f772bbf0305ac9710fa392aba5a")


