; Auto-generated. Do not edit!


(cl:in-package costmap_2d-msg)


;//! \htmlinclude DstarPoint.msg.html

(cl:defclass <DstarPoint> (roslisp-msg-protocol:ros-message)
  ((index
    :reader index
    :initarg :index
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass DstarPoint (<DstarPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DstarPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DstarPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name costmap_2d-msg:<DstarPoint> is deprecated: use costmap_2d-msg:DstarPoint instead.")))

(cl:ensure-generic-function 'index-val :lambda-list '(m))
(cl:defmethod index-val ((m <DstarPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader costmap_2d-msg:index-val is deprecated.  Use costmap_2d-msg:index instead.")
  (index m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DstarPoint>) ostream)
  "Serializes a message object of type '<DstarPoint>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'index))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'index))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DstarPoint>) istream)
  "Deserializes a message object of type '<DstarPoint>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'index) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'index)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DstarPoint>)))
  "Returns string type for a message object of type '<DstarPoint>"
  "costmap_2d/DstarPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DstarPoint)))
  "Returns string type for a message object of type 'DstarPoint"
  "costmap_2d/DstarPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DstarPoint>)))
  "Returns md5sum for a message object of type '<DstarPoint>"
  "df62f201f3c7cdf0f85b4ec3adb4c299")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DstarPoint)))
  "Returns md5sum for a message object of type 'DstarPoint"
  "df62f201f3c7cdf0f85b4ec3adb4c299")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DstarPoint>)))
  "Returns full string definition for message of type '<DstarPoint>"
  (cl:format cl:nil "int32[] index~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DstarPoint)))
  "Returns full string definition for message of type 'DstarPoint"
  (cl:format cl:nil "int32[] index~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DstarPoint>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'index) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DstarPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'DstarPoint
    (cl:cons ':index (index msg))
))
