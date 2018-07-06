
(cl:in-package :asdf)

(defsystem "bfsdriver_1_1-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "ImageStamp" :depends-on ("_package_ImageStamp"))
    (:file "_package_ImageStamp" :depends-on ("_package"))
  ))