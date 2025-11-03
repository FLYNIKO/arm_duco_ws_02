;; Auto-generated. Do not edit!


(when (boundp 'key_input_pkg::KeyInput)
  (if (not (find-package "KEY_INPUT_PKG"))
    (make-package "KEY_INPUT_PKG"))
  (shadow 'KeyInput (find-package "KEY_INPUT_PKG")))
(unless (find-package "KEY_INPUT_PKG::KEYINPUT")
  (make-package "KEY_INPUT_PKG::KEYINPUT"))

(in-package "ROS")
;;//! \htmlinclude KeyInput.msg.html


(defclass key_input_pkg::KeyInput
  :super ros::object
  :slots (_keys ))

(defmethod key_input_pkg::KeyInput
  (:init
   (&key
    ((:keys __keys) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _keys __keys)
   self)
  (:keys
   (&optional __keys)
   (if __keys (setq _keys __keys)) _keys)
  (:serialization-length
   ()
   (+
    ;; int32[] _keys
    (* 4    (length _keys)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _keys
     (write-long (length _keys) s)
     (dotimes (i (length _keys))
       (write-long (elt _keys i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _keys
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _keys (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _keys i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get key_input_pkg::KeyInput :md5sum-) "7baccca94a87d2d70852f2dc60f54b65")
(setf (get key_input_pkg::KeyInput :datatype-) "key_input_pkg/KeyInput")
(setf (get key_input_pkg::KeyInput :definition-)
      "int32[] keys

")



(provide :key_input_pkg/KeyInput "7baccca94a87d2d70852f2dc60f54b65")


