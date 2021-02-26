; Auto-generated. Do not edit!


(cl:in-package drone_msgs-msg)


;//! \htmlinclude DroneInfoArray.msg.html

(cl:defclass <DroneInfoArray> (roslisp-msg-protocol:ros-message)
  ((drones
    :reader drones
    :initarg :drones
    :type (cl:vector drone_msgs-msg:DroneInfo)
   :initform (cl:make-array 0 :element-type 'drone_msgs-msg:DroneInfo :initial-element (cl:make-instance 'drone_msgs-msg:DroneInfo))))
)

(cl:defclass DroneInfoArray (<DroneInfoArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DroneInfoArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DroneInfoArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name drone_msgs-msg:<DroneInfoArray> is deprecated: use drone_msgs-msg:DroneInfoArray instead.")))

(cl:ensure-generic-function 'drones-val :lambda-list '(m))
(cl:defmethod drones-val ((m <DroneInfoArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader drone_msgs-msg:drones-val is deprecated.  Use drone_msgs-msg:drones instead.")
  (drones m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DroneInfoArray>) ostream)
  "Serializes a message object of type '<DroneInfoArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'drones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'drones))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DroneInfoArray>) istream)
  "Deserializes a message object of type '<DroneInfoArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'drones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'drones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'drone_msgs-msg:DroneInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DroneInfoArray>)))
  "Returns string type for a message object of type '<DroneInfoArray>"
  "drone_msgs/DroneInfoArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DroneInfoArray)))
  "Returns string type for a message object of type 'DroneInfoArray"
  "drone_msgs/DroneInfoArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DroneInfoArray>)))
  "Returns md5sum for a message object of type '<DroneInfoArray>"
  "b567d9a3a60bda150b09c0bcd10bfc14")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DroneInfoArray)))
  "Returns md5sum for a message object of type 'DroneInfoArray"
  "b567d9a3a60bda150b09c0bcd10bfc14")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DroneInfoArray>)))
  "Returns full string definition for message of type '<DroneInfoArray>"
  (cl:format cl:nil "drone_msgs/DroneInfo[] drones~%================================================================================~%MSG: drone_msgs/DroneInfo~%int8 team_num   # number of team (0,1..n)~%int8 id_drone   # id of drone (0,1..n)~%int8 id_marker  # number of marker id~%float32 health  # health of drone (100%..0%)~%string ip~%drone_msgs/DronePose pose   # ENU position of drone~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DroneInfoArray)))
  "Returns full string definition for message of type 'DroneInfoArray"
  (cl:format cl:nil "drone_msgs/DroneInfo[] drones~%================================================================================~%MSG: drone_msgs/DroneInfo~%int8 team_num   # number of team (0,1..n)~%int8 id_drone   # id of drone (0,1..n)~%int8 id_marker  # number of marker id~%float32 health  # health of drone (100%..0%)~%string ip~%drone_msgs/DronePose pose   # ENU position of drone~%~%================================================================================~%MSG: drone_msgs/DronePose~%geometry_msgs/Point point~%float32 course~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DroneInfoArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'drones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DroneInfoArray>))
  "Converts a ROS message object to a list"
  (cl:list 'DroneInfoArray
    (cl:cons ':drones (drones msg))
))
