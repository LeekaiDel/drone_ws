; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude Goal.msg.html

(cl:defclass <Goal> (roslisp-msg-protocol:ros-message)
  ((ctr_type
    :reader ctr_type
    :initarg :ctr_type
    :type cl:integer
    :initform 0)
   (pose
    :reader pose
    :initarg :pose
    :type drone_msgs-msg:DronePose
    :initform (cl:make-instance 'drone_msgs-msg:DronePose)))
)

(cl:defclass Goal (<Goal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<Goal> is deprecated: use drone_msgs-msg:Goal instead.")))

(cl:ensure-generic-function 'ctr_type-val :lambda-list '(m))
(cl:defmethod ctr_type-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:ctr_type-val is deprecated.  Use drone_msgs-msg:ctr_type instead.")
  (ctr_type m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Goal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:pose-val is deprecated.  Use drone_msgs-msg:pose instead.")
  (pose m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Goal>)))
    "Constants for message type '<Goal>"
  '((:POSE . 0)
    (:VEL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Goal)))
    "Constants for message type 'Goal"
  '((:POSE . 0)
    (:VEL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goal>) ostream)
  "Serializes a message object of type '<Goal>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ctr_type)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goal>) istream)
  "Deserializes a message object of type '<Goal>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'ctr_type)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goal>)))
  "Returns string type for a message object of type '<Goal>"
  "drone_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goal)))
  "Returns string type for a message object of type 'Goal"
  "drone_msgs/Goal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goal>)))
  "Returns md5sum for a message object of type '<Goal>"
  "bf6e29cec64ab1c71dda19cb2e5b5f60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goal)))
  "Returns md5sum for a message object of type 'Goal"
  "bf6e29cec64ab1c71dda19cb2e5b5f60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goal>)))
  "Returns full string definition for message of type '<Goal>"
  (cl:format cl:nil "byte POSE=0~%byte VEL=1~%~%byte ctr_type~%drone_msgs/DronePose pose~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goal)))
  "Returns full string definition for message of type 'Goal"
  (cl:format cl:nil "byte POSE=0~%byte VEL=1~%~%byte ctr_type~%drone_msgs/DronePose pose~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goal>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goal>))
  "Converts a ROS message object to a list"
  (cl:list 'Goal
    (cl:cons ':ctr_type (ctr_type msg))
    (cl:cons ':pose (pose msg))
))
