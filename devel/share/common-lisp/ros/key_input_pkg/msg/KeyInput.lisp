; Auto-generated. Do not edit!


(cl:in-package key_input_pkg-msg)


;//! \htmlinclude KeyInput.msg.html

(cl:defclass <KeyInput> (roslisp-msg-protocol:ros-message)
  ((keys
    :reader keys
    :initarg :keys
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass KeyInput (<KeyInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name key_input_pkg-msg:<KeyInput> is deprecated: use key_input_pkg-msg:KeyInput instead.")))

(cl:ensure-generic-function 'keys-val :lambda-list '(m))
(cl:defmethod keys-val ((m <KeyInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader key_input_pkg-msg:keys-val is deprecated.  Use key_input_pkg-msg:keys instead.")
  (keys m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyInput>) ostream)
  "Serializes a message object of type '<KeyInput>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'keys))))
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
   (cl:slot-value msg 'keys))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyInput>) istream)
  "Deserializes a message object of type '<KeyInput>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'keys) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'keys)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyInput>)))
  "Returns string type for a message object of type '<KeyInput>"
  "key_input_pkg/KeyInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyInput)))
  "Returns string type for a message object of type 'KeyInput"
  "key_input_pkg/KeyInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyInput>)))
  "Returns md5sum for a message object of type '<KeyInput>"
  "7baccca94a87d2d70852f2dc60f54b65")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyInput)))
  "Returns md5sum for a message object of type 'KeyInput"
  "7baccca94a87d2d70852f2dc60f54b65")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyInput>)))
  "Returns full string definition for message of type '<KeyInput>"
  (cl:format cl:nil "int32[] keys~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyInput)))
  "Returns full string definition for message of type 'KeyInput"
  (cl:format cl:nil "int32[] keys~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyInput>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'keys) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyInput>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyInput
    (cl:cons ':keys (keys msg))
))
