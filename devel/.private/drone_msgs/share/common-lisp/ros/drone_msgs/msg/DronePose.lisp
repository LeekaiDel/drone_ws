; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude DronePose.msg.html

(cl:defclass <DronePose> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (course
    :reader course
    :initarg :course
    :type cl:float
    :initform 0.0))
)

(cl:defclass DronePose (<DronePose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DronePose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DronePose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<DronePose> is deprecated: use drone_msgs-msg:DronePose instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <DronePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:point-val is deprecated.  Use drone_msgs-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'course-val :lambda-list '(m))
(cl:defmethod course-val ((m <DronePose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:course-val is deprecated.  Use drone_msgs-msg:course instead.")
  (course m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DronePose>) ostream)
  "Serializes a message object of type '<DronePose>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'course))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DronePose>) istream)
  "Deserializes a message object of type '<DronePose>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'course) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DronePose>)))
  "Returns string type for a message object of type '<DronePose>"
  "drone_msgs/DronePose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DronePose)))
  "Returns string type for a message object of type 'DronePose"
  "drone_msgs/DronePose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DronePose>)))
  "Returns md5sum for a message object of type '<DronePose>"
  "c3922f772bbf0305ac9710fa392aba5a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DronePose)))
  "Returns md5sum for a message object of type 'DronePose"
  "c3922f772bbf0305ac9710fa392aba5a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DronePose>)))
  "Returns full string definition for message of type '<DronePose>"
  (cl:format cl:nil "geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DronePose)))
  "Returns full string definition for message of type 'DronePose"
  (cl:format cl:nil "geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DronePose>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DronePose>))
  "Converts a ROS message object to a list"
  (cl:list 'DronePose
    (cl:cons ':point (point msg))
    (cl:cons ':course (course msg))
))
