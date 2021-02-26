; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude LocalPlannerState.msg.html

(cl:defclass <LocalPlannerState> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil)
   (max_dist
    :reader max_dist
    :initarg :max_dist
    :type cl:float
    :initform 0.0)
   (k
    :reader k
    :initarg :k
    :type cl:float
    :initform 0.0))
)

(cl:defclass LocalPlannerState (<LocalPlannerState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalPlannerState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalPlannerState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<LocalPlannerState> is deprecated: use drone_msgs-msg:LocalPlannerState instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <LocalPlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:state-val is deprecated.  Use drone_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'max_dist-val :lambda-list '(m))
(cl:defmethod max_dist-val ((m <LocalPlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:max_dist-val is deprecated.  Use drone_msgs-msg:max_dist instead.")
  (max_dist m))

(cl:ensure-generic-function 'k-val :lambda-list '(m))
(cl:defmethod k-val ((m <LocalPlannerState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:k-val is deprecated.  Use drone_msgs-msg:k instead.")
  (k m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalPlannerState>) ostream)
  "Serializes a message object of type '<LocalPlannerState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalPlannerState>) istream)
  "Deserializes a message object of type '<LocalPlannerState>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max_dist) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalPlannerState>)))
  "Returns string type for a message object of type '<LocalPlannerState>"
  "drone_msgs/LocalPlannerState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalPlannerState)))
  "Returns string type for a message object of type 'LocalPlannerState"
  "drone_msgs/LocalPlannerState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalPlannerState>)))
  "Returns md5sum for a message object of type '<LocalPlannerState>"
  "152bbd8ec6848572b28cbf06443e0c97")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalPlannerState)))
  "Returns md5sum for a message object of type 'LocalPlannerState"
  "152bbd8ec6848572b28cbf06443e0c97")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalPlannerState>)))
  "Returns full string definition for message of type '<LocalPlannerState>"
  (cl:format cl:nil "bool state~%float32 max_dist~%float32 k~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalPlannerState)))
  "Returns full string definition for message of type 'LocalPlannerState"
  (cl:format cl:nil "bool state~%float32 max_dist~%float32 k~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalPlannerState>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalPlannerState>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalPlannerState
    (cl:cons ':state (state msg))
    (cl:cons ':max_dist (max_dist msg))
    (cl:cons ':k (k msg))
))
