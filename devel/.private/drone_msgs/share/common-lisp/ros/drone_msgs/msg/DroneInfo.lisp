; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude DroneInfo.msg.html

(cl:defclass <DroneInfo> (roslisp-msg-protocol:ros-message)
  ((team_num
    :reader team_num
    :initarg :team_num
    :type cl:fixnum
    :initform 0)
   (id_drone
    :reader id_drone
    :initarg :id_drone
    :type cl:fixnum
    :initform 0)
   (id_marker
    :reader id_marker
    :initarg :id_marker
    :type cl:fixnum
    :initform 0)
   (health
    :reader health
    :initarg :health
    :type cl:float
    :initform 0.0)
   (ip
    :reader ip
    :initarg :ip
    :type cl:string
    :initform "")
   (pose
    :reader pose
    :initarg :pose
    :type drone_msgs-msg:DronePose
    :initform (cl:make-instance 'drone_msgs-msg:DronePose)))
)

(cl:defclass DroneInfo (<DroneInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<DroneInfo> is deprecated: use drone_msgs-msg:DroneInfo instead.")))

(cl:ensure-generic-function 'team_num-val :lambda-list '(m))
(cl:defmethod team_num-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:team_num-val is deprecated.  Use drone_msgs-msg:team_num instead.")
  (team_num m))

(cl:ensure-generic-function 'id_drone-val :lambda-list '(m))
(cl:defmethod id_drone-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:id_drone-val is deprecated.  Use drone_msgs-msg:id_drone instead.")
  (id_drone m))

(cl:ensure-generic-function 'id_marker-val :lambda-list '(m))
(cl:defmethod id_marker-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:id_marker-val is deprecated.  Use drone_msgs-msg:id_marker instead.")
  (id_marker m))

(cl:ensure-generic-function 'health-val :lambda-list '(m))
(cl:defmethod health-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:health-val is deprecated.  Use drone_msgs-msg:health instead.")
  (health m))

(cl:ensure-generic-function 'ip-val :lambda-list '(m))
(cl:defmethod ip-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:ip-val is deprecated.  Use drone_msgs-msg:ip instead.")
  (ip m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <DroneInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:pose-val is deprecated.  Use drone_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneInfo>) ostream)
  "Serializes a message object of type '<DroneInfo>"
  (cl:let* ((signed (cl:slot-value msg 'team_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id_drone)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id_marker)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'health))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'ip))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'ip))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneInfo>) istream)
  "Deserializes a message object of type '<DroneInfo>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'team_num) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id_drone) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id_marker) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'health) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ip) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'ip) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneInfo>)))
  "Returns string type for a message object of type '<DroneInfo>"
  "drone_msgs/DroneInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneInfo)))
  "Returns string type for a message object of type 'DroneInfo"
  "drone_msgs/DroneInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneInfo>)))
  "Returns md5sum for a message object of type '<DroneInfo>"
  "142d303be68c1b2c6c6b79486ae3db7b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneInfo)))
  "Returns md5sum for a message object of type 'DroneInfo"
  "142d303be68c1b2c6c6b79486ae3db7b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneInfo>)))
  "Returns full string definition for message of type '<DroneInfo>"
  (cl:format cl:nil "int8 team_num   # number of team (0,1..n)~%int8 id_drone   # id of drone (0,1..n)~%int8 id_marker  # number of marker id~%float32 health  # health of drone (100%..0%)~%string ip~%drone_msgs/DronePose pose   # ENU position of drone~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneInfo)))
  "Returns full string definition for message of type 'DroneInfo"
  (cl:format cl:nil "int8 team_num   # number of team (0,1..n)~%int8 id_drone   # id of drone (0,1..n)~%int8 id_marker  # number of marker id~%float32 health  # health of drone (100%..0%)~%string ip~%drone_msgs/DronePose pose   # ENU position of drone~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneInfo>))
  (cl:+ 0
     1
     1
     1
     4
     4 (cl:length (cl:slot-value msg 'ip))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneInfo
    (cl:cons ':team_num (team_num msg))
    (cl:cons ':id_drone (id_drone msg))
    (cl:cons ':id_marker (id_marker msg))
    (cl:cons ':health (health msg))
    (cl:cons ':ip (ip msg))
    (cl:cons ':pose (pose msg))
))
