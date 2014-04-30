
(cl:in-package :asdf)

(defsystem "xsens-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Calibrate" :depends-on ("_package_Calibrate"))
    (:file "_package_Calibrate" :depends-on ("_package"))
  ))