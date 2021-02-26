; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude WindowPointDir.msg.html

(cl:defclass <WindowPointDir> (roslisp-msg-protocol:ros-message)
  ((found_window
    :reader found_window
    :initarg :found_window
    :type cl:boolean
    :initform cl:nil)
   (point
    :reader point
    :initarg :point
    :type drone_msgs-msg:DronePose
    :initform (cl:make-instance 'drone_msgs-msg:DronePose)))
)

(cl:defclass WindowPointDir (<WindowPointDir>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WindowPointDir>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WindowPointDir)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<WindowPointDir> is deprecated: use drone_msgs-msg:WindowPointDir instead.")))

(cl:ensure-generic-function 'found_window-val :lambda-list '(m))
(cl:defmethod found_window-val ((m <WindowPointDir>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:found_window-val is deprecated.  Use drone_msgs-msg:found_window instead.")
  (found_window m))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <WindowPointDir>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:point-val is deprecated.  Use drone_msgs-msg:point instead.")
  (point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WindowPointDir>) ostream)
  "Serializes a message object of type '<WindowPointDir>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'found_window) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WindowPointDir>) istream)
  "Deserializes a message object of type '<WindowPointDir>"
    (cl:setf (cl:slot-value msg 'found_window) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WindowPointDir>)))
  "Returns string type for a message object of type '<WindowPointDir>"
  "drone_msgs/WindowPointDir")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WindowPointDir)))
  "Returns string type for a message object of type 'WindowPointDir"
  "drone_msgs/WindowPointDir")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WindowPointDir>)))
  "Returns md5sum for a message object of type '<WindowPointDir>"
  "6e775f1bde836c88e4039d51e180cd67")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WindowPointDir)))
  "Returns md5sum for a message object of type 'WindowPointDir"
  "6e775f1bde836c88e4039d51e180cd67")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WindowPointDir>)))
  "Returns full string definition for message of type '<WindowPointDir>"
  (cl:format cl:nil " bool found_window~% drone_msgs/DronePose point~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WindowPointDir)))
  "Returns full string definition for message of type 'WindowPointDir"
  (cl:format cl:nil " bool found_window~% drone_msgs/DronePose point~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WindowPointDir>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WindowPointDir>))
  "Converts a ROS message object to a list"
  (cl:list 'WindowPointDir
    (cl:cons ':found_window (found_window msg))
    (cl:cons ':point (point msg))
))
