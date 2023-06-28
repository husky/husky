;; Auto-generated. Do not edit!


(when (boundp 'ouster_ros::SetConfig)
  (if (not (find-package "OUSTER_ROS"))
    (make-package "OUSTER_ROS"))
  (shadow 'SetConfig (find-package "OUSTER_ROS")))
(unless (find-package "OUSTER_ROS::SETCONFIG")
  (make-package "OUSTER_ROS::SETCONFIG"))
(unless (find-package "OUSTER_ROS::SETCONFIGREQUEST")
  (make-package "OUSTER_ROS::SETCONFIGREQUEST"))
(unless (find-package "OUSTER_ROS::SETCONFIGRESPONSE")
  (make-package "OUSTER_ROS::SETCONFIGRESPONSE"))

(in-package "ROS")





(defclass ouster_ros::SetConfigRequest
  :super ros::object
  :slots (_config_file ))

(defmethod ouster_ros::SetConfigRequest
  (:init
   (&key
    ((:config_file __config_file) "")
    )
   (send-super :init)
   (setq _config_file (string __config_file))
   self)
  (:config_file
   (&optional __config_file)
   (if __config_file (setq _config_file __config_file)) _config_file)
  (:serialization-length
   ()
   (+
    ;; string _config_file
    4 (length _config_file)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _config_file
       (write-long (length _config_file) s) (princ _config_file s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _config_file
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _config_file (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ouster_ros::SetConfigResponse
  :super ros::object
  :slots (_config ))

(defmethod ouster_ros::SetConfigResponse
  (:init
   (&key
    ((:config __config) "")
    )
   (send-super :init)
   (setq _config (string __config))
   self)
  (:config
   (&optional __config)
   (if __config (setq _config __config)) _config)
  (:serialization-length
   ()
   (+
    ;; string _config
    4 (length _config)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _config
       (write-long (length _config) s) (princ _config s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _config
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _config (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ouster_ros::SetConfig
  :super ros::object
  :slots ())

(setf (get ouster_ros::SetConfig :md5sum-) "7c21f4ef724c955b8242aed00884d81a")
(setf (get ouster_ros::SetConfig :datatype-) "ouster_ros/SetConfig")
(setf (get ouster_ros::SetConfig :request) ouster_ros::SetConfigRequest)
(setf (get ouster_ros::SetConfig :response) ouster_ros::SetConfigResponse)

(defmethod ouster_ros::SetConfigRequest
  (:response () (instance ouster_ros::SetConfigResponse :init)))

(setf (get ouster_ros::SetConfigRequest :md5sum-) "7c21f4ef724c955b8242aed00884d81a")
(setf (get ouster_ros::SetConfigRequest :datatype-) "ouster_ros/SetConfigRequest")
(setf (get ouster_ros::SetConfigRequest :definition-)
      "string config_file
---
string config
")

(setf (get ouster_ros::SetConfigResponse :md5sum-) "7c21f4ef724c955b8242aed00884d81a")
(setf (get ouster_ros::SetConfigResponse :datatype-) "ouster_ros/SetConfigResponse")
(setf (get ouster_ros::SetConfigResponse :definition-)
      "string config_file
---
string config
")



(provide :ouster_ros/SetConfig "7c21f4ef724c955b8242aed00884d81a")


