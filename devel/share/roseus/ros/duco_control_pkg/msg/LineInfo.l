;; Auto-generated. Do not edit!


(when (boundp 'duco_control_pkg::LineInfo)
  (if (not (find-package "DUCO_CONTROL_PKG"))
    (make-package "DUCO_CONTROL_PKG"))
  (shadow 'LineInfo (find-package "DUCO_CONTROL_PKG")))
(unless (find-package "DUCO_CONTROL_PKG::LINEINFO")
  (make-package "DUCO_CONTROL_PKG::LINEINFO"))

(in-package "ROS")
;;//! \htmlinclude LineInfo.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass duco_control_pkg::LineInfo
  :super ros::object
  :slots (_start_point _end_point _length _angle_deg _distance _type _id ))

(defmethod duco_control_pkg::LineInfo
  (:init
   (&key
    ((:start_point __start_point) (instance geometry_msgs::Point :init))
    ((:end_point __end_point) (instance geometry_msgs::Point :init))
    ((:length __length) 0.0)
    ((:angle_deg __angle_deg) 0.0)
    ((:distance __distance) 0.0)
    ((:type __type) 0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _start_point __start_point)
   (setq _end_point __end_point)
   (setq _length (float __length))
   (setq _angle_deg (float __angle_deg))
   (setq _distance (float __distance))
   (setq _type (round __type))
   (setq _id (round __id))
   self)
  (:start_point
   (&rest __start_point)
   (if (keywordp (car __start_point))
       (send* _start_point __start_point)
     (progn
       (if __start_point (setq _start_point (car __start_point)))
       _start_point)))
  (:end_point
   (&rest __end_point)
   (if (keywordp (car __end_point))
       (send* _end_point __end_point)
     (progn
       (if __end_point (setq _end_point (car __end_point)))
       _end_point)))
  (:length
   (&optional __length)
   (if __length (setq _length __length)) _length)
  (:angle_deg
   (&optional __angle_deg)
   (if __angle_deg (setq _angle_deg __angle_deg)) _angle_deg)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _start_point
    (send _start_point :serialization-length)
    ;; geometry_msgs/Point _end_point
    (send _end_point :serialization-length)
    ;; float32 _length
    4
    ;; float32 _angle_deg
    4
    ;; float32 _distance
    4
    ;; int32 _type
    4
    ;; int32 _id
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _start_point
       (send _start_point :serialize s)
     ;; geometry_msgs/Point _end_point
       (send _end_point :serialize s)
     ;; float32 _length
       (sys::poke _length (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle_deg
       (sys::poke _angle_deg (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int32 _type
       (write-long _type s)
     ;; int32 _id
       (write-long _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _start_point
     (send _start_point :deserialize buf ptr-) (incf ptr- (send _start_point :serialization-length))
   ;; geometry_msgs/Point _end_point
     (send _end_point :deserialize buf ptr-) (incf ptr- (send _end_point :serialization-length))
   ;; float32 _length
     (setq _length (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle_deg
     (setq _angle_deg (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get duco_control_pkg::LineInfo :md5sum-) "c268946f4721487ff5d43b6e4957d2b0")
(setf (get duco_control_pkg::LineInfo :datatype-) "duco_control_pkg/LineInfo")
(setf (get duco_control_pkg::LineInfo :definition-)
      "# LineInfo.msg
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



(provide :duco_control_pkg/LineInfo "c268946f4721487ff5d43b6e4957d2b0")


