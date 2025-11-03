; Auto-generated. Do not edit!


(cl:in-package duco_control_pkg-msg)


;//! \htmlinclude LineDetectionArray.msg.html

(cl:defclass <LineDetectionArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (lines
    :reader lines
    :initarg :lines
    :type (cl:vector duco_control_pkg-msg:LineInfo)
   :initform (cl:make-array 0 :element-type 'duco_control_pkg-msg:LineInfo :initial-element (cl:make-instance 'duco_control_pkg-msg:LineInfo))))
)

(cl:defclass LineDetectionArray (<LineDetectionArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LineDetectionArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LineDetectionArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name duco_control_pkg-msg:<LineDetectionArray> is deprecated: use duco_control_pkg-msg:LineDetectionArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LineDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:header-val is deprecated.  Use duco_control_pkg-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'lines-val :lambda-list '(m))
(cl:defmethod lines-val ((m <LineDetectionArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:lines-val is deprecated.  Use duco_control_pkg-msg:lines instead.")
  (lines m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LineDetectionArray>) ostream)
  "Serializes a message object of type '<LineDetectionArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'lines))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'lines))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LineDetectionArray>) istream)
  "Deserializes a message object of type '<LineDetectionArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'lines) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'lines)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'duco_control_pkg-msg:LineInfo))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LineDetectionArray>)))
  "Returns string type for a message object of type '<LineDetectionArray>"
  "duco_control_pkg/LineDetectionArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LineDetectionArray)))
  "Returns string type for a message object of type 'LineDetectionArray"
  "duco_control_pkg/LineDetectionArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LineDetectionArray>)))
  "Returns md5sum for a message object of type '<LineDetectionArray>"
  "7f0d2b557aa2d2c956b6c6a0f2991a99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LineDetectionArray)))
  "Returns md5sum for a message object of type 'LineDetectionArray"
  "7f0d2b557aa2d2c956b6c6a0f2991a99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LineDetectionArray>)))
  "Returns full string definition for message of type '<LineDetectionArray>"
  (cl:format cl:nil "# LineDetectionArray.msg~%std_msgs/Header header~%duco_control_pkg/LineInfo[] lines # 包含多条 LineInfo 消息~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: duco_control_pkg/LineInfo~%# LineInfo.msg~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%float32 length~%float32 angle_deg~%float32 distance~%int32 type~%int32 id~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LineDetectionArray)))
  "Returns full string definition for message of type 'LineDetectionArray"
  (cl:format cl:nil "# LineDetectionArray.msg~%std_msgs/Header header~%duco_control_pkg/LineInfo[] lines # 包含多条 LineInfo 消息~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: duco_control_pkg/LineInfo~%# LineInfo.msg~%geometry_msgs/Point start_point~%geometry_msgs/Point end_point~%float32 length~%float32 angle_deg~%float32 distance~%int32 type~%int32 id~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LineDetectionArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'lines) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LineDetectionArray>))
  "Converts a ROS message object to a list"
  (cl:list 'LineDetectionArray
    (cl:cons ':header (header msg))
    (cl:cons ':lines (lines msg))
))
