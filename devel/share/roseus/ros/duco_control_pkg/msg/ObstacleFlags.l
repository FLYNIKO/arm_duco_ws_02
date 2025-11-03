;; Auto-generated. Do not edit!


(when (boundp 'duco_control_pkg::ObstacleFlags)
  (if (not (find-package "DUCO_CONTROL_PKG"))
    (make-package "DUCO_CONTROL_PKG"))
  (shadow 'ObstacleFlags (find-package "DUCO_CONTROL_PKG")))
(unless (find-package "DUCO_CONTROL_PKG::OBSTACLEFLAGS")
  (make-package "DUCO_CONTROL_PKG::OBSTACLEFLAGS"))

(in-package "ROS")
;;//! \htmlinclude ObstacleFlags.msg.html


(defclass duco_control_pkg::ObstacleFlags
  :super ros::object
  :slots (_left_front _left_mid _left_rear _right_front _right_mid _right_rear _center _up _down _stamp _safe_distance ))

(defmethod duco_control_pkg::ObstacleFlags
  (:init
   (&key
    ((:left_front __left_front) nil)
    ((:left_mid __left_mid) nil)
    ((:left_rear __left_rear) nil)
    ((:right_front __right_front) nil)
    ((:right_mid __right_mid) nil)
    ((:right_rear __right_rear) nil)
    ((:center __center) nil)
    ((:up __up) nil)
    ((:down __down) nil)
    ((:stamp __stamp) (instance ros::time :init))
    ((:safe_distance __safe_distance) 0.0)
    )
   (send-super :init)
   (setq _left_front __left_front)
   (setq _left_mid __left_mid)
   (setq _left_rear __left_rear)
   (setq _right_front __right_front)
   (setq _right_mid __right_mid)
   (setq _right_rear __right_rear)
   (setq _center __center)
   (setq _up __up)
   (setq _down __down)
   (setq _stamp __stamp)
   (setq _safe_distance (float __safe_distance))
   self)
  (:left_front
   (&optional (__left_front :null))
   (if (not (eq __left_front :null)) (setq _left_front __left_front)) _left_front)
  (:left_mid
   (&optional (__left_mid :null))
   (if (not (eq __left_mid :null)) (setq _left_mid __left_mid)) _left_mid)
  (:left_rear
   (&optional (__left_rear :null))
   (if (not (eq __left_rear :null)) (setq _left_rear __left_rear)) _left_rear)
  (:right_front
   (&optional (__right_front :null))
   (if (not (eq __right_front :null)) (setq _right_front __right_front)) _right_front)
  (:right_mid
   (&optional (__right_mid :null))
   (if (not (eq __right_mid :null)) (setq _right_mid __right_mid)) _right_mid)
  (:right_rear
   (&optional (__right_rear :null))
   (if (not (eq __right_rear :null)) (setq _right_rear __right_rear)) _right_rear)
  (:center
   (&optional (__center :null))
   (if (not (eq __center :null)) (setq _center __center)) _center)
  (:up
   (&optional (__up :null))
   (if (not (eq __up :null)) (setq _up __up)) _up)
  (:down
   (&optional (__down :null))
   (if (not (eq __down :null)) (setq _down __down)) _down)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:safe_distance
   (&optional __safe_distance)
   (if __safe_distance (setq _safe_distance __safe_distance)) _safe_distance)
  (:serialization-length
   ()
   (+
    ;; bool _left_front
    1
    ;; bool _left_mid
    1
    ;; bool _left_rear
    1
    ;; bool _right_front
    1
    ;; bool _right_mid
    1
    ;; bool _right_rear
    1
    ;; bool _center
    1
    ;; bool _up
    1
    ;; bool _down
    1
    ;; time _stamp
    8
    ;; float32 _safe_distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _left_front
       (if _left_front (write-byte -1 s) (write-byte 0 s))
     ;; bool _left_mid
       (if _left_mid (write-byte -1 s) (write-byte 0 s))
     ;; bool _left_rear
       (if _left_rear (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_front
       (if _right_front (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_mid
       (if _right_mid (write-byte -1 s) (write-byte 0 s))
     ;; bool _right_rear
       (if _right_rear (write-byte -1 s) (write-byte 0 s))
     ;; bool _center
       (if _center (write-byte -1 s) (write-byte 0 s))
     ;; bool _up
       (if _up (write-byte -1 s) (write-byte 0 s))
     ;; bool _down
       (if _down (write-byte -1 s) (write-byte 0 s))
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;; float32 _safe_distance
       (sys::poke _safe_distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _left_front
     (setq _left_front (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left_mid
     (setq _left_mid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _left_rear
     (setq _left_rear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_front
     (setq _right_front (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_mid
     (setq _right_mid (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _right_rear
     (setq _right_rear (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _center
     (setq _center (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _up
     (setq _up (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _down
     (setq _down (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _safe_distance
     (setq _safe_distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get duco_control_pkg::ObstacleFlags :md5sum-) "d0c938d9c5630fb2581830805df06b63")
(setf (get duco_control_pkg::ObstacleFlags :datatype-) "duco_control_pkg/ObstacleFlags")
(setf (get duco_control_pkg::ObstacleFlags :definition-)
      "# ObstacleFlags.msg
# 自定义消息类型，用于发布各区域的避障标志

# 各区域避障标志
bool left_front    # 左前区有障碍
bool left_mid      # 左中区有障碍
bool left_rear     # 左后区有障碍
bool right_front   # 右前区有障碍  
bool right_mid     # 右中区有障碍
bool right_rear    # 右后区有障碍
bool center        # 中区有障碍
bool up            # 上区有障碍
bool down          # 下区有障碍
# 可选：添加时间戳和安全距离信息
time stamp              # 检测时间戳
float32 safe_distance   # 使用的安全距离
")



(provide :duco_control_pkg/ObstacleFlags "d0c938d9c5630fb2581830805df06b63")


