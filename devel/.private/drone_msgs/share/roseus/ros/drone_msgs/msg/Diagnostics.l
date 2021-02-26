;; Auto-generated. Do not edit!


(when (boundp 'drone_msgs::Diagnostics)
  (if (not (find-package "DRONE_MSGS"))
    (make-package "DRONE_MSGS"))
  (shadow 'Diagnostics (find-package "DRONE_MSGS")))
(unless (find-package "DRONE_MSGS::DIAGNOSTICS")
  (make-package "DRONE_MSGS::DIAGNOSTICS"))

(in-package "ROS")
;;//! \htmlinclude Diagnostics.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass drone_msgs::Diagnostics
  :super ros::object
  :slots (_header _armed _init_home _init_origin _gps_send _status _mode _battery _health ))

(defmethod drone_msgs::Diagnostics
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:armed __armed) nil)
    ((:init_home __init_home) nil)
    ((:init_origin __init_origin) nil)
    ((:gps_send __gps_send) nil)
    ((:status __status) (instance sensor_msgs::NavSatStatus :init))
    ((:mode __mode) "")
    ((:battery __battery) 0.0)
    ((:health __health) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _armed __armed)
   (setq _init_home __init_home)
   (setq _init_origin __init_origin)
   (setq _gps_send __gps_send)
   (setq _status __status)
   (setq _mode (string __mode))
   (setq _battery (float __battery))
   (setq _health (float __health))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:armed
   (&optional __armed)
   (if __armed (setq _armed __armed)) _armed)
  (:init_home
   (&optional __init_home)
   (if __init_home (setq _init_home __init_home)) _init_home)
  (:init_origin
   (&optional __init_origin)
   (if __init_origin (setq _init_origin __init_origin)) _init_origin)
  (:gps_send
   (&optional __gps_send)
   (if __gps_send (setq _gps_send __gps_send)) _gps_send)
  (:status
   (&rest __status)
   (if (keywordp (car __status))
       (send* _status __status)
     (progn
       (if __status (setq _status (car __status)))
       _status)))
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:battery
   (&optional __battery)
   (if __battery (setq _battery __battery)) _battery)
  (:health
   (&optional __health)
   (if __health (setq _health __health)) _health)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _armed
    1
    ;; bool _init_home
    1
    ;; bool _init_origin
    1
    ;; bool _gps_send
    1
    ;; sensor_msgs/NavSatStatus _status
    (send _status :serialization-length)
    ;; string _mode
    4 (length _mode)
    ;; float32 _battery
    4
    ;; float32 _health
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _armed
       (if _armed (write-byte -1 s) (write-byte 0 s))
     ;; bool _init_home
       (if _init_home (write-byte -1 s) (write-byte 0 s))
     ;; bool _init_origin
       (if _init_origin (write-byte -1 s) (write-byte 0 s))
     ;; bool _gps_send
       (if _gps_send (write-byte -1 s) (write-byte 0 s))
     ;; sensor_msgs/NavSatStatus _status
       (send _status :serialize s)
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; float32 _battery
       (sys::poke _battery (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _health
       (sys::poke _health (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _armed
     (setq _armed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _init_home
     (setq _init_home (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _init_origin
     (setq _init_origin (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _gps_send
     (setq _gps_send (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; sensor_msgs/NavSatStatus _status
     (send _status :deserialize buf ptr-) (incf ptr- (send _status :serialization-length))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _battery
     (setq _battery (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _health
     (setq _health (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get drone_msgs::Diagnostics :md5sum-) "2c87ff1e63a374108ac3dbac9530310f")
(setf (get drone_msgs::Diagnostics :datatype-) "drone_msgs/Diagnostics")
(setf (get drone_msgs::Diagnostics :definition-)
      "std_msgs/Header header
bool armed
bool init_home
bool init_origin
bool gps_send
sensor_msgs/NavSatStatus status


string mode
float32 battery

float32 health  # health of drone (100%..0%)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/NavSatStatus
# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service

")



(provide :drone_msgs/Diagnostics "2c87ff1e63a374108ac3dbac9530310f")


