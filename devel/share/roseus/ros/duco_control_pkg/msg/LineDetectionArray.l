;; Auto-generated. Do not edit!


(when (boundp 'duco_control_pkg::LineDetectionArray)
  (if (not (find-package "DUCO_CONTROL_PKG"))
    (make-package "DUCO_CONTROL_PKG"))
  (shadow 'LineDetectionArray (find-package "DUCO_CONTROL_PKG")))
(unless (find-package "DUCO_CONTROL_PKG::LINEDETECTIONARRAY")
  (make-package "DUCO_CONTROL_PKG::LINEDETECTIONARRAY"))

(in-package "ROS")
;;//! \htmlinclude LineDetectionArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass duco_control_pkg::LineDetectionArray
  :super ros::object
  :slots (_header _lines ))

(defmethod duco_control_pkg::LineDetectionArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:lines __lines) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _lines __lines)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:lines
   (&rest __lines)
   (if (keywordp (car __lines))
       (send* _lines __lines)
     (progn
       (if __lines (setq _lines (car __lines)))
       _lines)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; duco_control_pkg/LineInfo[] _lines
    (apply #'+ (send-all _lines :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; duco_control_pkg/LineInfo[] _lines
     (write-long (length _lines) s)
     (dolist (elem _lines)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; duco_control_pkg/LineInfo[] _lines
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _lines (let (r) (dotimes (i n) (push (instance duco_control_pkg::LineInfo :init) r)) r))
     (dolist (elem- _lines)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get duco_control_pkg::LineDetectionArray :md5sum-) "7f0d2b557aa2d2c956b6c6a0f2991a99")
(setf (get duco_control_pkg::LineDetectionArray :datatype-) "duco_control_pkg/LineDetectionArray")
(setf (get duco_control_pkg::LineDetectionArray :definition-)
      "# LineDetectionArray.msg
std_msgs/Header header
duco_control_pkg/LineInfo[] lines # 包含多条 LineInfo 消息
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: duco_control_pkg/LineInfo
# LineInfo.msg
geometry_msgs/Point start_point
geometry_msgs/Point end_point
float32 length
float32 angle_deg
float32 distance
int32 type
int32 id
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :duco_control_pkg/LineDetectionArray "7f0d2b557aa2d2c956b6c6a0f2991a99")


