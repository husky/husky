
(cl:in-package :asdf)

(defsystem "ouster_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetConfig" :depends-on ("_package_GetConfig"))
    (:file "_package_GetConfig" :depends-on ("_package"))
    (:file "GetMetadata" :depends-on ("_package_GetMetadata"))
    (:file "_package_GetMetadata" :depends-on ("_package"))
    (:file "SetConfig" :depends-on ("_package_SetConfig"))
    (:file "_package_SetConfig" :depends-on ("_package"))
  ))