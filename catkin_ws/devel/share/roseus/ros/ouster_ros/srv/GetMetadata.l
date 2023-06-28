;; Auto-generated. Do not edit!


(when (boundp 'ouster_ros::GetMetadata)
  (if (not (find-package "OUSTER_ROS"))
    (make-package "OUSTER_ROS"))
  (shadow 'GetMetadata (find-package "OUSTER_ROS")))
(unless (find-package "OUSTER_ROS::GETMETADATA")
  (make-package "OUSTER_ROS::GETMETADATA"))
(unless (find-package "OUSTER_ROS::GETMETADATAREQUEST")
  (make-package "OUSTER_ROS::GETMETADATAREQUEST"))
(unless (find-package "OUSTER_ROS::GETMETADATARESPONSE")
  (make-package "OUSTER_ROS::GETMETADATARESPONSE"))

(in-package "ROS")





(defclass ouster_ros::GetMetadataRequest
  :super ros::object
  :slots ())

(defmethod ouster_ros::GetMetadataRequest
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

(defclass ouster_ros::GetMetadataResponse
  :super ros::object
  :slots (_metadata ))

(defmethod ouster_ros::GetMetadataResponse
  (:init
   (&key
    ((:metadata __metadata) "")
    )
   (send-super :init)
   (setq _metadata (string __metadata))
   self)
  (:metadata
   (&optional __metadata)
   (if __metadata (setq _metadata __metadata)) _metadata)
  (:serialization-length
   ()
   (+
    ;; string _metadata
    4 (length _metadata)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _metadata
       (write-long (length _metadata) s) (princ _metadata s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _metadata
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _metadata (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ouster_ros::GetMetadata
  :super ros::object
  :slots ())

(setf (get ouster_ros::GetMetadata :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::GetMetadata :datatype-) "ouster_ros/GetMetadata")
(setf (get ouster_ros::GetMetadata :request) ouster_ros::GetMetadataRequest)
(setf (get ouster_ros::GetMetadata :response) ouster_ros::GetMetadataResponse)

(defmethod ouster_ros::GetMetadataRequest
  (:response () (instance ouster_ros::GetMetadataResponse :init)))

(setf (get ouster_ros::GetMetadataRequest :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::GetMetadataRequest :datatype-) "ouster_ros/GetMetadataRequest")
(setf (get ouster_ros::GetMetadataRequest :definition-)
      "---
string metadata

")

(setf (get ouster_ros::GetMetadataResponse :md5sum-) "d37888e5a47bef783c189dec5351027e")
(setf (get ouster_ros::GetMetadataResponse :datatype-) "ouster_ros/GetMetadataResponse")
(setf (get ouster_ros::GetMetadataResponse :definition-)
      "---
string metadata

")



(provide :ouster_ros/GetMetadata "d37888e5a47bef783c189dec5351027e")


