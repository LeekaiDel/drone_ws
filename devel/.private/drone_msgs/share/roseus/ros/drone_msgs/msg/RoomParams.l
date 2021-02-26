;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::RoomParams)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'RoomParams (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::ROOMPARAMS")
  (make-package "DRONE_MSGS::ROOMPARAMS"))

(in-package "ROS")
;;//! \htmlinclude RoomParams.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass drone_msgs::RoomParams
  :super ros::object
  :slots (_lines _points ))

(defmethod drone_msgs::RoomParams
  (:init
   (&key
    ((:lines __lines) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    ((:points __points) (let (r) (dotimes (i 0) (push (instance geometry_msgs::Point :init) r)) r))
    )
   (send-super :init)
   (setq _lines __lines)
   (setq _points __points)
   self)
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ;; geometry_msgs/Point[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Point[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Point[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get drone_msgs::RoomParams :md5sum-) "722c360e0d28cee48e7fbb2df2294d61")
(setf (get drone_msgs::RoomParams :datatype-) "drone_msgs/RoomParams")
(setf (get drone_msgs::RoomParams :definition-)
      "geometry_msgs/Point[] lines
geometry_msgs/Point[] points

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :drone_msgs/RoomParams "722c360e0d28cee48e7fbb2df2294d61")


