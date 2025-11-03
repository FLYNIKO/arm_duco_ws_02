
(cl:in-package :asdf)

(defsystem "duco_control_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "LineDetectionArray" :depends-on ("_package_LineDetectionArray"))
    (:file "_package_LineDetectionArray" :depends-on ("_package"))
    (:file "LineInfo" :depends-on ("_package_LineInfo"))
    (:file "_package_LineInfo" :depends-on ("_package"))
    (:file "ObstacleFlags" :depends-on ("_package_ObstacleFlags"))
    (:file "_package_ObstacleFlags" :depends-on ("_package"))
  ))