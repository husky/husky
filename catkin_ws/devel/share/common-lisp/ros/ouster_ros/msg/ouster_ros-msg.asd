
(cl:in-package :asdf)

(defsystem "ouster_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PacketMsg" :depends-on ("_package_PacketMsg"))
    (:file "_package_PacketMsg" :depends-on ("_package"))
  ))