; Auto-generated. Do not edit!


(cl:in-package duco_control_pkg-msg)


;//! \htmlinclude ObstacleFlags.msg.html

(cl:defclass <ObstacleFlags> (roslisp-msg-protocol:ros-message)
  ((left_front
    :reader left_front
    :initarg :left_front
    :type cl:boolean
    :initform cl:nil)
   (left_mid
    :reader left_mid
    :initarg :left_mid
    :type cl:boolean
    :initform cl:nil)
   (left_rear
    :reader left_rear
    :initarg :left_rear
    :type cl:boolean
    :initform cl:nil)
   (right_front
    :reader right_front
    :initarg :right_front
    :type cl:boolean
    :initform cl:nil)
   (right_mid
    :reader right_mid
    :initarg :right_mid
    :type cl:boolean
    :initform cl:nil)
   (right_rear
    :reader right_rear
    :initarg :right_rear
    :type cl:boolean
    :initform cl:nil)
   (center
    :reader center
    :initarg :center
    :type cl:boolean
    :initform cl:nil)
   (up
    :reader up
    :initarg :up
    :type cl:boolean
    :initform cl:nil)
   (down
    :reader down
    :initarg :down
    :type cl:boolean
    :initform cl:nil)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0)
   (safe_distance
    :reader safe_distance
    :initarg :safe_distance
    :type cl:float
    :initform 0.0))
)

(cl:defclass ObstacleFlags (<ObstacleFlags>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ObstacleFlags>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ObstacleFlags)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name duco_control_pkg-msg:<ObstacleFlags> is deprecated: use duco_control_pkg-msg:ObstacleFlags instead.")))

(cl:ensure-generic-function 'left_front-val :lambda-list '(m))
(cl:defmethod left_front-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:left_front-val is deprecated.  Use duco_control_pkg-msg:left_front instead.")
  (left_front m))

(cl:ensure-generic-function 'left_mid-val :lambda-list '(m))
(cl:defmethod left_mid-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:left_mid-val is deprecated.  Use duco_control_pkg-msg:left_mid instead.")
  (left_mid m))

(cl:ensure-generic-function 'left_rear-val :lambda-list '(m))
(cl:defmethod left_rear-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:left_rear-val is deprecated.  Use duco_control_pkg-msg:left_rear instead.")
  (left_rear m))

(cl:ensure-generic-function 'right_front-val :lambda-list '(m))
(cl:defmethod right_front-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:right_front-val is deprecated.  Use duco_control_pkg-msg:right_front instead.")
  (right_front m))

(cl:ensure-generic-function 'right_mid-val :lambda-list '(m))
(cl:defmethod right_mid-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:right_mid-val is deprecated.  Use duco_control_pkg-msg:right_mid instead.")
  (right_mid m))

(cl:ensure-generic-function 'right_rear-val :lambda-list '(m))
(cl:defmethod right_rear-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:right_rear-val is deprecated.  Use duco_control_pkg-msg:right_rear instead.")
  (right_rear m))

(cl:ensure-generic-function 'center-val :lambda-list '(m))
(cl:defmethod center-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:center-val is deprecated.  Use duco_control_pkg-msg:center instead.")
  (center m))

(cl:ensure-generic-function 'up-val :lambda-list '(m))
(cl:defmethod up-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:up-val is deprecated.  Use duco_control_pkg-msg:up instead.")
  (up m))

(cl:ensure-generic-function 'down-val :lambda-list '(m))
(cl:defmethod down-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:down-val is deprecated.  Use duco_control_pkg-msg:down instead.")
  (down m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:stamp-val is deprecated.  Use duco_control_pkg-msg:stamp instead.")
  (stamp m))

(cl:ensure-generic-function 'safe_distance-val :lambda-list '(m))
(cl:defmethod safe_distance-val ((m <ObstacleFlags>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader duco_control_pkg-msg:safe_distance-val is deprecated.  Use duco_control_pkg-msg:safe_distance instead.")
  (safe_distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ObstacleFlags>) ostream)
  "Serializes a message object of type '<ObstacleFlags>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_front) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_mid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'left_rear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_front) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_mid) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_rear) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'center) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'up) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'down) 1 0)) ostream)
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'safe_distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ObstacleFlags>) istream)
  "Deserializes a message object of type '<ObstacleFlags>"
    (cl:setf (cl:slot-value msg 'left_front) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_mid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'left_rear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_front) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_mid) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'right_rear) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'center) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'up) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'down) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'safe_distance) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ObstacleFlags>)))
  "Returns string type for a message object of type '<ObstacleFlags>"
  "duco_control_pkg/ObstacleFlags")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ObstacleFlags)))
  "Returns string type for a message object of type 'ObstacleFlags"
  "duco_control_pkg/ObstacleFlags")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ObstacleFlags>)))
  "Returns md5sum for a message object of type '<ObstacleFlags>"
  "d0c938d9c5630fb2581830805df06b63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ObstacleFlags)))
  "Returns md5sum for a message object of type 'ObstacleFlags"
  "d0c938d9c5630fb2581830805df06b63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ObstacleFlags>)))
  "Returns full string definition for message of type '<ObstacleFlags>"
  (cl:format cl:nil "# ObstacleFlags.msg~%# 自定义消息类型，用于发布各区域的避障标志~%~%# 各区域避障标志~%bool left_front    # 左前区有障碍~%bool left_mid      # 左中区有障碍~%bool left_rear     # 左后区有障碍~%bool right_front   # 右前区有障碍  ~%bool right_mid     # 右中区有障碍~%bool right_rear    # 右后区有障碍~%bool center        # 中区有障碍~%bool up            # 上区有障碍~%bool down          # 下区有障碍~%# 可选：添加时间戳和安全距离信息~%time stamp              # 检测时间戳~%float32 safe_distance   # 使用的安全距离~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ObstacleFlags)))
  "Returns full string definition for message of type 'ObstacleFlags"
  (cl:format cl:nil "# ObstacleFlags.msg~%# 自定义消息类型，用于发布各区域的避障标志~%~%# 各区域避障标志~%bool left_front    # 左前区有障碍~%bool left_mid      # 左中区有障碍~%bool left_rear     # 左后区有障碍~%bool right_front   # 右前区有障碍  ~%bool right_mid     # 右中区有障碍~%bool right_rear    # 右后区有障碍~%bool center        # 中区有障碍~%bool up            # 上区有障碍~%bool down          # 下区有障碍~%# 可选：添加时间戳和安全距离信息~%time stamp              # 检测时间戳~%float32 safe_distance   # 使用的安全距离~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ObstacleFlags>))
  (cl:+ 0
     1
     1
     1
     1
     1
     1
     1
     1
     1
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ObstacleFlags>))
  "Converts a ROS message object to a list"
  (cl:list 'ObstacleFlags
    (cl:cons ':left_front (left_front msg))
    (cl:cons ':left_mid (left_mid msg))
    (cl:cons ':left_rear (left_rear msg))
    (cl:cons ':right_front (right_front msg))
    (cl:cons ':right_mid (right_mid msg))
    (cl:cons ':right_rear (right_rear msg))
    (cl:cons ':center (center msg))
    (cl:cons ':up (up msg))
    (cl:cons ':down (down msg))
    (cl:cons ':stamp (stamp msg))
    (cl:cons ':safe_distance (safe_distance msg))
))
