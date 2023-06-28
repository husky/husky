;; Auto-generated. Do not edit!


(when (boundp 'ouster_ros::GetConfig)
  (if (not (find-package "OUSTER_ROS"))
    (make-package "OUSTER_ROS"))
  (shadow 'GetConfig (find-package "OUSTER_ROS")))
(unless (find-package "OUSTER_ROS::GETCONFIG")
  (make-package "OUSTER_ROS::GETCONFIG"))
(unless (find-package "OUSTER_ROS::GETCONFIGREQUEST")
  (make-package "OUSTER_ROS::GETCONFIGREQUEST"))
(unless (find-package "OUSTER_ROS::GETCONFIGRESPONSE")
  (make-package "OUSTER_ROS::GETCONFIGRESPONSE"))

(in-package "ROS")





(defclass ouster_ros::GetConfigRequest
  :super ros::object
  :slots ())

(defmethod ouster_ros::GetConfigRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass ouster_ros::GetConfigResponse
  :super ros::object
  :slots (_config ))

(defmethod ouster_ros::GetConfigResponse
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

(defclass ouster_ros::GetConfig
  :super ros::object
  :slots ())

(setf (get ouster_ros::GetConfig :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get ouster_ros::GetConfig :datatype-) "ouster_ros/GetConfig")
(setf (get ouster_ros::GetConfig :request) ouster_ros::GetConfigRequest)
(setf (get ouster_ros::GetConfig :response) ouster_ros::GetConfigResponse)

(defmethod ouster_ros::GetConfigRequest
  (:response () (instance ouster_ros::GetConfigResponse :init)))

(setf (get ouster_ros::GetConfigRequest :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get ouster_ros::GetConfigRequest :datatype-) "ouster_ros/GetConfigRequest")
(setf (get ouster_ros::GetConfigRequest :definition-)
      "---
string config

")

(setf (get ouster_ros::GetConfigResponse :md5sum-) "b3532af339db184b4a6a974d00ee4fe6")
(setf (get ouster_ros::GetConfigResponse :datatype-) "ouster_ros/GetConfigResponse")
(setf (get ouster_ros::GetConfigResponse :definition-)
      "---
string config

")



(provide :ouster_ros/GetConfig "b3532af339db184b4a6a974d00ee4fe6")


