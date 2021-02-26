;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::LocalPlannerState)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'LocalPlannerState (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::LOCALPLANNERSTATE")
  (make-package "DRONE_MSGS::LOCALPLANNERSTATE"))

(in-package "ROS")
;;//! \htmlinclude LocalPlannerState.msg.html


(defclass drone_msgs::LocalPlannerState
  :super ros::object
  :slots (_state _max_dist _k ))

(defmethod drone_msgs::LocalPlannerState
  (:init
   (&key
    ((:state __state) nil)
    ((:max_dist __max_dist) 0.0)
    ((:k __k) 0.0)
    )
   (send-super :init)
   (setq _state __state)
   (setq _max_dist (float __max_dist))
   (setq _k (float __k))
   self)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:max_dist
   (&optional __max_dist)
   (if __max_dist (setq _max_dist __max_dist)) _max_dist)
  (:k
   (&optional __k)
   (if __k (setq _k __k)) _k)
  (:serialization-length
   ()
   (+
    ;; bool _state
    1
    ;; float32 _max_dist
    4
    ;; float32 _k
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;; float32 _max_dist
       (sys::poke _max_dist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k
       (sys::poke _k (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _max_dist
     (setq _max_dist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k
     (setq _k (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone_msgs::LocalPlannerState :md5sum-) "152bbd8ec6848572b28cbf06443e0c97")
(setf (get drone_msgs::LocalPlannerState :datatype-) "drone_msgs/LocalPlannerState")
(setf (get drone_msgs::LocalPlannerState :definition-)
      "bool state
float32 max_dist
float32 k

")



(provide :drone_msgs/LocalPlannerState "152bbd8ec6848572b28cbf06443e0c97")


