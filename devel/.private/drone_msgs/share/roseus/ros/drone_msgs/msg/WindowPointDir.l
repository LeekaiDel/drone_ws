;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::WindowPointDir)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'WindowPointDir (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::WINDOWPOINTDIR")
  (make-package "DRONE_MSGS::WINDOWPOINTDIR"))

(in-package "ROS")
;;//! \htmlinclude WindowPointDir.msg.html


(defclass drone_msgs::WindowPointDir
  :super ros::object
  :slots (_found_window _point ))

(defmethod drone_msgs::WindowPointDir
  (:init
   (&key
    ((:found_window __found_window) nil)
    ((:point __point) (instance drone_msgs::DronePose :init))
    )
   (send-super :init)
   (setq _found_window __found_window)
   (setq _point __point)
   self)
  (:found_window
   (&optional __found_window)
   (if __found_window (setq _found_window __found_window)) _found_window)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:serialization-length
   ()
   (+
    ;; bool _found_window
    1
    ;; drone_msgs/DronePose _point
    (send _point :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _found_window
       (if _found_window (write-byte -1 s) (write-byte 0 s))
     ;; drone_msgs/DronePose _point
       (send _point :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _found_window
     (setq _found_window (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; drone_msgs/DronePose _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;;
   self)
  )

(setf (get drone_msgs::WindowPointDir :md5sum-) "6e775f1bde836c88e4039d51e180cd67")
(setf (get drone_msgs::WindowPointDir :datatype-) "drone_msgs/WindowPointDir")
(setf (get drone_msgs::WindowPointDir :definition-)
      " bool found_window
 drone_msgs/DronePose point

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



(provide :drone_msgs/WindowPointDir "6e775f1bde836c88e4039d51e180cd67")


