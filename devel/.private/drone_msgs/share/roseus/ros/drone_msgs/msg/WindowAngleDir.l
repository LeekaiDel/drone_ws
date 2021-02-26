;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::WindowAngleDir)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'WindowAngleDir (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::WINDOWANGLEDIR")
  (make-package "DRONE_MSGS::WINDOWANGLEDIR"))

(in-package "ROS")
;;//! \htmlinclude WindowAngleDir.msg.html


(defclass drone_msgs::WindowAngleDir
  :super ros::object
  :slots (_found_window _width_angle _height_angle ))

(defmethod drone_msgs::WindowAngleDir
  (:init
   (&key
    ((:found_window __found_window) nil)
    ((:width_angle __width_angle) 0.0)
    ((:height_angle __height_angle) 0.0)
    )
   (send-super :init)
   (setq _found_window __found_window)
   (setq _width_angle (float __width_angle))
   (setq _height_angle (float __height_angle))
   self)
  (:found_window
   (&optional __found_window)
   (if __found_window (setq _found_window __found_window)) _found_window)
  (:width_angle
   (&optional __width_angle)
   (if __width_angle (setq _width_angle __width_angle)) _width_angle)
  (:height_angle
   (&optional __height_angle)
   (if __height_angle (setq _height_angle __height_angle)) _height_angle)
  (:serialization-length
   ()
   (+
    ;; bool _found_window
    1
    ;; float32 _width_angle
    4
    ;; float32 _height_angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _found_window
       (if _found_window (write-byte -1 s) (write-byte 0 s))
     ;; float32 _width_angle
       (sys::poke _width_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _height_angle
       (sys::poke _height_angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _found_window
     (setq _found_window (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _width_angle
     (setq _width_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _height_angle
     (setq _height_angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone_msgs::WindowAngleDir :md5sum-) "e706742e1d1f28d3b56545716318772f")
(setf (get drone_msgs::WindowAngleDir :datatype-) "drone_msgs/WindowAngleDir")
(setf (get drone_msgs::WindowAngleDir :definition-)
      "bool found_window
float32 width_angle
float32 height_angle

")



(provide :drone_msgs/WindowAngleDir "e706742e1d1f28d3b56545716318772f")


