; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude WindowAngleDir.msg.html

(cl:defclass <WindowAngleDir> (roslisp-msg-protocol:ros-message)
  ((found_window
    :reader found_window
    :initarg :found_window
    :type cl:boolean
    :initform cl:nil)
   (width_angle
    :reader width_angle
    :initarg :width_angle
    :type cl:float
    :initform 0.0)
   (height_angle
    :reader height_angle
    :initarg :height_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass WindowAngleDir (<WindowAngleDir>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WindowAngleDir>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WindowAngleDir)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<WindowAngleDir> is deprecated: use drone_msgs-msg:WindowAngleDir instead.")))

(cl:ensure-generic-function 'found_window-val :lambda-list '(m))
(cl:defmethod found_window-val ((m <WindowAngleDir>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:found_window-val is deprecated.  Use drone_msgs-msg:found_window instead.")
  (found_window m))

(cl:ensure-generic-function 'width_angle-val :lambda-list '(m))
(cl:defmethod width_angle-val ((m <WindowAngleDir>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:width_angle-val is deprecated.  Use drone_msgs-msg:width_angle instead.")
  (width_angle m))

(cl:ensure-generic-function 'height_angle-val :lambda-list '(m))
(cl:defmethod height_angle-val ((m <WindowAngleDir>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:height_angle-val is deprecated.  Use drone_msgs-msg:height_angle instead.")
  (height_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WindowAngleDir>) ostream)
  "Serializes a message object of type '<WindowAngleDir>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found_window) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'width_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'height_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WindowAngleDir>) istream)
  "Deserializes a message object of type '<WindowAngleDir>"
    (cl:setf (cl:slot-value msg 'found_window) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width_angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'height_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WindowAngleDir>)))
  "Returns string type for a message object of type '<WindowAngleDir>"
  "drone_msgs/WindowAngleDir")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WindowAngleDir)))
  "Returns string type for a message object of type 'WindowAngleDir"
  "drone_msgs/WindowAngleDir")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WindowAngleDir>)))
  "Returns md5sum for a message object of type '<WindowAngleDir>"
  "e706742e1d1f28d3b56545716318772f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WindowAngleDir)))
  "Returns md5sum for a message object of type 'WindowAngleDir"
  "e706742e1d1f28d3b56545716318772f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WindowAngleDir>)))
  "Returns full string definition for message of type '<WindowAngleDir>"
  (cl:format cl:nil "bool found_window~%float32 width_angle~%float32 height_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WindowAngleDir)))
  "Returns full string definition for message of type 'WindowAngleDir"
  (cl:format cl:nil "bool found_window~%float32 width_angle~%float32 height_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WindowAngleDir>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WindowAngleDir>))
  "Converts a ROS message object to a list"
  (cl:list 'WindowAngleDir
    (cl:cons ':found_window (found_window msg))
    (cl:cons ':width_angle (width_angle msg))
    (cl:cons ':height_angle (height_angle msg))
))
