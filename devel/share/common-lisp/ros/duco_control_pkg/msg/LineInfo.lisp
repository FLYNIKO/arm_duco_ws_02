; Auto-generated. Do not edit!


(cl:in-package duco_control_pkg-msg)


;//! \htmlinclude LineInfo.msg.html

(cl:defclass <LineInfo> (roslisp-msg-protocol:ros-message)
  ((start_point
    :reader start_point
    :initarg :start_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (end_point
    :reader end_point
    :initarg :end_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (length
    :reader length
    :initarg :length
    :type cl:float
    :initform 0.0)
   (angle_deg
    :reader angle_deg
    :initarg :angle_deg
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0)
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass LineInfo (<LineInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name duco_control_pkg-msg:<LineInfo> is deprecated: use duco_control_pkg-msg:LineInfo instead.")))

(cl:ensure-generic-function 'start_point-val :lambda-list '(m))
(cl:defmethod start_point-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:start_point-val is deprecated.  Use duco_control_pkg-msg:start_point instead.")
  (start_point m))

(cl:ensure-generic-function 'end_point-val :lambda-list '(m))
(cl:defmethod end_point-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:end_point-val is deprecated.  Use duco_control_pkg-msg:end_point instead.")
  (end_point m))

(cl:ensure-generic-function 'length-val :lambda-list '(m))
(cl:defmethod length-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:length-val is deprecated.  Use duco_control_pkg-msg:length instead.")
  (length m))

(cl:ensure-generic-function 'angle_deg-val :lambda-list '(m))
(cl:defmethod angle_deg-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:angle_deg-val is deprecated.  Use duco_control_pkg-msg:angle_deg instead.")
  (angle_deg m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:distance-val is deprecated.  Use duco_control_pkg-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:type-val is deprecated.  Use duco_control_pkg-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <LineInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:id-val is deprecated.  Use duco_control_pkg-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineInfo>) ostream)
  "Serializes a message object of type '<LineInfo>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_point) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_point) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle_deg))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineInfo>) istream)
  "Deserializes a message object of type '<LineInfo>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_point) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_point) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'length) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle_deg) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineInfo>)))
  "Returns string type for a message object of type '<LineInfo>"
  "duco_control_pkg/LineInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineInfo)))
  "Returns string type for a message object of type 'LineInfo"
  "duco_control_pkg/LineInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineInfo>)))
  "Returns md5sum for a message object of type '<LineInfo>"
  "c268946f4721487ff5d43b6e4957d2b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineInfo)))
  "Returns md5sum for a message object of type 'LineInfo"
  "c268946f4721487ff5d43b6e4957d2b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineInfo>)))
  "Returns full string definition for message of type '<LineInfo>"
  (cl:format cl:nil "# LineInfo.msg~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%float32 length~%float32 angle_deg~%float32 distance~%int32 type~%int32 id~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineInfo)))
  "Returns full string definition for message of type 'LineInfo"
  (cl:format cl:nil "# LineInfo.msg~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%float32 length~%float32 angle_deg~%float32 distance~%int32 type~%int32 id~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineInfo>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_point))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_point))
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'LineInfo
    (cl:cons ':start_point (start_point msg))
    (cl:cons ':end_point (end_point msg))
    (cl:cons ':length (length msg))
    (cl:cons ':angle_deg (angle_deg msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':type (type msg))
    (cl:cons ':id (id msg))
))
