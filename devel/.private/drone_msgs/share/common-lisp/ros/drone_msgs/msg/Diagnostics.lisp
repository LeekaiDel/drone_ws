; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude Diagnostics.msg.html

(cl:defclass <Diagnostics> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil)
   (init_home
    :reader init_home
    :initarg :init_home
    :type cl:boolean
    :initform cl:nil)
   (init_origin
    :reader init_origin
    :initarg :init_origin
    :type cl:boolean
    :initform cl:nil)
   (gps_send
    :reader gps_send
    :initarg :gps_send
    :type cl:boolean
    :initform cl:nil)
   (status
    :reader status
    :initarg :status
    :type sensor_msgs-msg:NavSatStatus
    :initform (cl:make-instance 'sensor_msgs-msg:NavSatStatus))
   (mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform "")
   (battery
    :reader battery
    :initarg :battery
    :type cl:float
    :initform 0.0)
   (health
    :reader health
    :initarg :health
    :type cl:float
    :initform 0.0))
)

(cl:defclass Diagnostics (<Diagnostics>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Diagnostics>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Diagnostics)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<Diagnostics> is deprecated: use drone_msgs-msg:Diagnostics instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:header-val is deprecated.  Use drone_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:armed-val is deprecated.  Use drone_msgs-msg:armed instead.")
  (armed m))

(cl:ensure-generic-function 'init_home-val :lambda-list '(m))
(cl:defmethod init_home-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:init_home-val is deprecated.  Use drone_msgs-msg:init_home instead.")
  (init_home m))

(cl:ensure-generic-function 'init_origin-val :lambda-list '(m))
(cl:defmethod init_origin-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:init_origin-val is deprecated.  Use drone_msgs-msg:init_origin instead.")
  (init_origin m))

(cl:ensure-generic-function 'gps_send-val :lambda-list '(m))
(cl:defmethod gps_send-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:gps_send-val is deprecated.  Use drone_msgs-msg:gps_send instead.")
  (gps_send m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:status-val is deprecated.  Use drone_msgs-msg:status instead.")
  (status m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:mode-val is deprecated.  Use drone_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'battery-val :lambda-list '(m))
(cl:defmethod battery-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:battery-val is deprecated.  Use drone_msgs-msg:battery instead.")
  (battery m))

(cl:ensure-generic-function 'health-val :lambda-list '(m))
(cl:defmethod health-val ((m <Diagnostics>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:health-val is deprecated.  Use drone_msgs-msg:health instead.")
  (health m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Diagnostics>) ostream)
  "Serializes a message object of type '<Diagnostics>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'init_home) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'init_origin) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'gps_send) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'status) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'battery))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'health))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Diagnostics>) istream)
  "Deserializes a message object of type '<Diagnostics>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'init_home) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'init_origin) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'gps_send) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'status) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'battery) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'health) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Diagnostics>)))
  "Returns string type for a message object of type '<Diagnostics>"
  "drone_msgs/Diagnostics")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Diagnostics)))
  "Returns string type for a message object of type 'Diagnostics"
  "drone_msgs/Diagnostics")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Diagnostics>)))
  "Returns md5sum for a message object of type '<Diagnostics>"
  "2c87ff1e63a374108ac3dbac9530310f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Diagnostics)))
  "Returns md5sum for a message object of type 'Diagnostics"
  "2c87ff1e63a374108ac3dbac9530310f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Diagnostics>)))
  "Returns full string definition for message of type '<Diagnostics>"
  (cl:format cl:nil "std_msgs/Header header~%bool armed~%bool init_home~%bool init_origin~%bool gps_send~%sensor_msgs/NavSatStatus status~%~%~%string mode~%float32 battery~%~%float32 health  # health of drone (100%..0%)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/NavSatStatus~%# Navigation Satellite fix status for any Global Navigation Satellite System~%~%# Whether to output an augmented fix is determined by both the fix~%# type and the last time differential corrections were received.  A~%# fix is valid when status >= STATUS_FIX.~%~%int8 STATUS_NO_FIX =  -1        # unable to fix position~%int8 STATUS_FIX =      0        # unaugmented fix~%int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation~%int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation~%~%int8 status~%~%# Bits defining which Global Navigation Satellite System signals were~%# used by the receiver.~%~%uint16 SERVICE_GPS =     1~%uint16 SERVICE_GLONASS = 2~%uint16 SERVICE_COMPASS = 4      # includes BeiDou.~%uint16 SERVICE_GALILEO = 8~%~%uint16 service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Diagnostics)))
  "Returns full string definition for message of type 'Diagnostics"
  (cl:format cl:nil "std_msgs/Header header~%bool armed~%bool init_home~%bool init_origin~%bool gps_send~%sensor_msgs/NavSatStatus status~%~%~%string mode~%float32 battery~%~%float32 health  # health of drone (100%..0%)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/NavSatStatus~%# Navigation Satellite fix status for any Global Navigation Satellite System~%~%# Whether to output an augmented fix is determined by both the fix~%# type and the last time differential corrections were received.  A~%# fix is valid when status >= STATUS_FIX.~%~%int8 STATUS_NO_FIX =  -1        # unable to fix position~%int8 STATUS_FIX =      0        # unaugmented fix~%int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation~%int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation~%~%int8 status~%~%# Bits defining which Global Navigation Satellite System signals were~%# used by the receiver.~%~%uint16 SERVICE_GPS =     1~%uint16 SERVICE_GLONASS = 2~%uint16 SERVICE_COMPASS = 4      # includes BeiDou.~%uint16 SERVICE_GALILEO = 8~%~%uint16 service~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Diagnostics>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'status))
     4 (cl:length (cl:slot-value msg 'mode))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Diagnostics>))
  "Converts a ROS message object to a list"
  (cl:list 'Diagnostics
    (cl:cons ':header (header msg))
    (cl:cons ':armed (armed msg))
    (cl:cons ':init_home (init_home msg))
    (cl:cons ':init_origin (init_origin msg))
    (cl:cons ':gps_send (gps_send msg))
    (cl:cons ':status (status msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':battery (battery msg))
    (cl:cons ':health (health msg))
))
