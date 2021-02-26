; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude Strike.msg.html

(cl:defclass <Strike> (roslisp-msg-protocol:ros-message)
  ((id_drone
    :reader id_drone
    :initarg :id_drone
    :type cl:fixnum
    :initform 0)
   (team_num
    :reader team_num
    :initarg :team_num
    :type cl:fixnum
    :initform 0)
   (shot
    :reader shot
    :initarg :shot
    :type cl:float
    :initform 0.0))
)

(cl:defclass Strike (<Strike>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Strike>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Strike)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<Strike> is deprecated: use drone_msgs-msg:Strike instead.")))

(cl:ensure-generic-function 'id_drone-val :lambda-list '(m))
(cl:defmethod id_drone-val ((m <Strike>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:id_drone-val is deprecated.  Use drone_msgs-msg:id_drone instead.")
  (id_drone m))

(cl:ensure-generic-function 'team_num-val :lambda-list '(m))
(cl:defmethod team_num-val ((m <Strike>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:team_num-val is deprecated.  Use drone_msgs-msg:team_num instead.")
  (team_num m))

(cl:ensure-generic-function 'shot-val :lambda-list '(m))
(cl:defmethod shot-val ((m <Strike>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:shot-val is deprecated.  Use drone_msgs-msg:shot instead.")
  (shot m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Strike>) ostream)
  "Serializes a message object of type '<Strike>"
  (cl:let* ((signed (cl:slot-value msg 'id_drone)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'team_num)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'shot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Strike>) istream)
  "Deserializes a message object of type '<Strike>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id_drone) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'team_num) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shot) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Strike>)))
  "Returns string type for a message object of type '<Strike>"
  "drone_msgs/Strike")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Strike)))
  "Returns string type for a message object of type 'Strike"
  "drone_msgs/Strike")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Strike>)))
  "Returns md5sum for a message object of type '<Strike>"
  "bc5465527f2e02efe558071ee95658cf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Strike)))
  "Returns md5sum for a message object of type 'Strike"
  "bc5465527f2e02efe558071ee95658cf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Strike>)))
  "Returns full string definition for message of type '<Strike>"
  (cl:format cl:nil "int8 id_drone   # the id of the drone that shoots (0,1..n)~%int8 team_num   # the number of team of the drone that shoots (0,1..n)~%float32 shot  # the force of the shot~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Strike)))
  "Returns full string definition for message of type 'Strike"
  (cl:format cl:nil "int8 id_drone   # the id of the drone that shoots (0,1..n)~%int8 team_num   # the number of team of the drone that shoots (0,1..n)~%float32 shot  # the force of the shot~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Strike>))
  (cl:+ 0
     1
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Strike>))
  "Converts a ROS message object to a list"
  (cl:list 'Strike
    (cl:cons ':id_drone (id_drone msg))
    (cl:cons ':team_num (team_num msg))
    (cl:cons ':shot (shot msg))
))
