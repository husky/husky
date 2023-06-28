; Auto-generated. Do not edit!


(cl:in-package ouster_ros-srv)


;//! \htmlinclude SetConfig-request.msg.html

(cl:defclass <SetConfig-request> (roslisp-msg-protocol:ros-message)
  ((config_file
    :reader config_file
    :initarg :config_file
    :type cl:string
    :initform ""))
)

(cl:defclass SetConfig-request (<SetConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<SetConfig-request> is deprecated: use ouster_ros-srv:SetConfig-request instead.")))

(cl:ensure-generic-function 'config_file-val :lambda-list '(m))
(cl:defmethod config_file-val ((m <SetConfig-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ouster_ros-srv:config_file-val is deprecated.  Use ouster_ros-srv:config_file instead.")
  (config_file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetConfig-request>) ostream)
  "Serializes a message object of type '<SetConfig-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'config_file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'config_file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetConfig-request>) istream)
  "Deserializes a message object of type '<SetConfig-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'config_file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'config_file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetConfig-request>)))
  "Returns string type for a service object of type '<SetConfig-request>"
  "ouster_ros/SetConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig-request)))
  "Returns string type for a service object of type 'SetConfig-request"
  "ouster_ros/SetConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetConfig-request>)))
  "Returns md5sum for a message object of type '<SetConfig-request>"
  "7c21f4ef724c955b8242aed00884d81a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetConfig-request)))
  "Returns md5sum for a message object of type 'SetConfig-request"
  "7c21f4ef724c955b8242aed00884d81a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetConfig-request>)))
  "Returns full string definition for message of type '<SetConfig-request>"
  (cl:format cl:nil "string config_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetConfig-request)))
  "Returns full string definition for message of type 'SetConfig-request"
  (cl:format cl:nil "string config_file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetConfig-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'config_file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetConfig-request
    (cl:cons ':config_file (config_file msg))
))
;//! \htmlinclude SetConfig-response.msg.html

(cl:defclass <SetConfig-response> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type cl:string
    :initform ""))
)

(cl:defclass SetConfig-response (<SetConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<SetConfig-response> is deprecated: use ouster_ros-srv:SetConfig-response instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <SetConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ouster_ros-srv:config-val is deprecated.  Use ouster_ros-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetConfig-response>) ostream)
  "Serializes a message object of type '<SetConfig-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'config))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'config))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetConfig-response>) istream)
  "Deserializes a message object of type '<SetConfig-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'config) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'config) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetConfig-response>)))
  "Returns string type for a service object of type '<SetConfig-response>"
  "ouster_ros/SetConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig-response)))
  "Returns string type for a service object of type 'SetConfig-response"
  "ouster_ros/SetConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetConfig-response>)))
  "Returns md5sum for a message object of type '<SetConfig-response>"
  "7c21f4ef724c955b8242aed00884d81a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetConfig-response)))
  "Returns md5sum for a message object of type 'SetConfig-response"
  "7c21f4ef724c955b8242aed00884d81a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetConfig-response>)))
  "Returns full string definition for message of type '<SetConfig-response>"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetConfig-response)))
  "Returns full string definition for message of type 'SetConfig-response"
  (cl:format cl:nil "string config~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetConfig-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetConfig-response
    (cl:cons ':config (config msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetConfig)))
  'SetConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetConfig)))
  'SetConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetConfig)))
  "Returns string type for a service object of type '<SetConfig>"
  "ouster_ros/SetConfig")