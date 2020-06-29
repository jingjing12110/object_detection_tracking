; Auto-generated. Do not edit!


(cl:in-package lidar_msgs-msg)


;//! \htmlinclude ObjectData.msg.html

(cl:defclass <ObjectData> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (bbox
    :reader bbox
    :initarg :bbox
    :type (cl:vector lidar_msgs-msg:Object)
   :initform (cl:make-array 0 :element-type 'lidar_msgs-msg:Object :initial-element (cl:make-instance 'lidar_msgs-msg:Object)))
   (token
    :reader token
    :initarg :token
    :type cl:string
    :initform ""))
)

(cl:defclass ObjectData (<ObjectData>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObjectData>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObjectData)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lidar_msgs-msg:<ObjectData> is deprecated: use lidar_msgs-msg:ObjectData instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ObjectData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:header-val is deprecated.  Use lidar_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'bbox-val :lambda-list '(m))
(cl:defmethod bbox-val ((m <ObjectData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:bbox-val is deprecated.  Use lidar_msgs-msg:bbox instead.")
  (bbox m))

(cl:ensure-generic-function 'token-val :lambda-list '(m))
(cl:defmethod token-val ((m <ObjectData>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lidar_msgs-msg:token-val is deprecated.  Use lidar_msgs-msg:token instead.")
  (token m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObjectData>) ostream)
  "Serializes a message object of type '<ObjectData>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bbox))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bbox))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'token))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'token))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObjectData>) istream)
  "Deserializes a message object of type '<ObjectData>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bbox) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bbox)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'lidar_msgs-msg:Object))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'token) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'token) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObjectData>)))
  "Returns string type for a message object of type '<ObjectData>"
  "lidar_msgs/ObjectData")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObjectData)))
  "Returns string type for a message object of type 'ObjectData"
  "lidar_msgs/ObjectData")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObjectData>)))
  "Returns md5sum for a message object of type '<ObjectData>"
  "cce2fe89002b3c1acf48988217c33108")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObjectData)))
  "Returns md5sum for a message object of type 'ObjectData"
  "cce2fe89002b3c1acf48988217c33108")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObjectData>)))
  "Returns full string definition for message of type '<ObjectData>"
  (cl:format cl:nil "Header header~%~%Object[] bbox~%string token~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: lidar_msgs/Object~%~%Point3 center~%Point3 size~%Point2 velocity~%Point2[] corners~%Point2[] contours~%int16 id~%~%float64[36] predict_covariance~%~%~%~%================================================================================~%MSG: lidar_msgs/Point3~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: lidar_msgs/Point2~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObjectData)))
  "Returns full string definition for message of type 'ObjectData"
  (cl:format cl:nil "Header header~%~%Object[] bbox~%string token~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: lidar_msgs/Object~%~%Point3 center~%Point3 size~%Point2 velocity~%Point2[] corners~%Point2[] contours~%int16 id~%~%float64[36] predict_covariance~%~%~%~%================================================================================~%MSG: lidar_msgs/Point3~%float32 x~%float32 y~%float32 z~%================================================================================~%MSG: lidar_msgs/Point2~%float32 x~%float32 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObjectData>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bbox) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:length (cl:slot-value msg 'token))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObjectData>))
  "Converts a ROS message object to a list"
  (cl:list 'ObjectData
    (cl:cons ':header (header msg))
    (cl:cons ':bbox (bbox msg))
    (cl:cons ':token (token msg))
))
