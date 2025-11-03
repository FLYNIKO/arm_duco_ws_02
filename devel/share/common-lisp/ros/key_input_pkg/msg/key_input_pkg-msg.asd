
(cl:in-package :asdf)

(defsystem "key_input_pkg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "KeyInput" :depends-on ("_package_KeyInput"))
    (:file "_package_KeyInput" :depends-on ("_package"))
  ))