;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::Strike)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'Strike (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::STRIKE")
  (make-package "DRONE_MSGS::STRIKE"))

(in-package "ROS")
;;//! \htmlinclude Strike.msg.html


(defclass drone_msgs::Strike
  :super ros::object
  :slots (_id_drone _team_num _shot ))

(defmethod drone_msgs::Strike
  (:init
   (&key
    ((:id_drone __id_drone) 0)
    ((:team_num __team_num) 0)
    ((:shot __shot) 0.0)
    )
   (send-super :init)
   (setq _id_drone (round __id_drone))
   (setq _team_num (round __team_num))
   (setq _shot (float __shot))
   self)
  (:id_drone
   (&optional __id_drone)
   (if __id_drone (setq _id_drone __id_drone)) _id_drone)
  (:team_num
   (&optional __team_num)
   (if __team_num (setq _team_num __team_num)) _team_num)
  (:shot
   (&optional __shot)
   (if __shot (setq _shot __shot)) _shot)
  (:serialization-length
   ()
   (+
    ;; int8 _id_drone
    1
    ;; int8 _team_num
    1
    ;; float32 _shot
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _id_drone
       (write-byte _id_drone s)
     ;; int8 _team_num
       (write-byte _team_num s)
     ;; float32 _shot
       (sys::poke _shot (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _id_drone
     (setq _id_drone (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id_drone 127) (setq _id_drone (- _id_drone 256)))
   ;; int8 _team_num
     (setq _team_num (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _team_num 127) (setq _team_num (- _team_num 256)))
   ;; float32 _shot
     (setq _shot (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone_msgs::Strike :md5sum-) "bc5465527f2e02efe558071ee95658cf")
(setf (get drone_msgs::Strike :datatype-) "drone_msgs/Strike")
(setf (get drone_msgs::Strike :definition-)
      "int8 id_drone   # the id of the drone that shoots (0,1..n)
int8 team_num   # the number of team of the drone that shoots (0,1..n)
float32 shot  # the force of the shot
")



(provide :drone_msgs/Strike "bc5465527f2e02efe558071ee95658cf")


