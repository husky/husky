; Auto-generated. Do not edit!


(cl:in-package ouster_ros-srv)


;//! \htmlinclude GetConfig-request.msg.html

(cl:defclass <GetConfig-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetConfig-request (<GetConfig-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfig-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfig-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<GetConfig-request> is deprecated: use ouster_ros-srv:GetConfig-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfig-request>) ostream)
  "Serializes a message object of type '<GetConfig-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfig-request>) istream)
  "Deserializes a message object of type '<GetConfig-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfig-request>)))
  "Returns string type for a service object of type '<GetConfig-request>"
  "ouster_ros/GetConfigRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig-request)))
  "Returns string type for a service object of type 'GetConfig-request"
  "ouster_ros/GetConfigRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfig-request>)))
  "Returns md5sum for a message object of type '<GetConfig-request>"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfig-request)))
  "Returns md5sum for a message object of type 'GetConfig-request"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfig-request>)))
  "Returns full string definition for message of type '<GetConfig-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfig-request)))
  "Returns full string definition for message of type 'GetConfig-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfig-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfig-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfig-request
))
;//! \htmlinclude GetConfig-response.msg.html

(cl:defclass <GetConfig-response> (roslisp-msg-protocol:ros-message)
  ((config
    :reader config
    :initarg :config
    :type cl:string
    :initform ""))
)

(cl:defclass GetConfig-response (<GetConfig-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetConfig-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetConfig-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<GetConfig-response> is deprecated: use ouster_ros-srv:GetConfig-response instead.")))

(cl:ensure-generic-function 'config-val :lambda-list '(m))
(cl:defmethod config-val ((m <GetConfig-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ouster_ros-srv:config-val is deprecated.  Use ouster_ros-srv:config instead.")
  (config m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetConfig-response>) ostream)
  "Serializes a message object of type '<GetConfig-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'config))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'config))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetConfig-response>) istream)
  "Deserializes a message object of type '<GetConfig-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetConfig-response>)))
  "Returns string type for a service object of type '<GetConfig-response>"
  "ouster_ros/GetConfigResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig-response)))
  "Returns string type for a service object of type 'GetConfig-response"
  "ouster_ros/GetConfigResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetConfig-response>)))
  "Returns md5sum for a message object of type '<GetConfig-response>"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetConfig-response)))
  "Returns md5sum for a message object of type 'GetConfig-response"
  "b3532af339db184b4a6a974d00ee4fe6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetConfig-response>)))
  "Returns full string definition for message of type '<GetConfig-response>"
  (cl:format cl:nil "string config~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetConfig-response)))
  "Returns full string definition for message of type 'GetConfig-response"
  (cl:format cl:nil "string config~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetConfig-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'config))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetConfig-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetConfig-response
    (cl:cons ':config (config msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetConfig)))
  'GetConfig-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetConfig)))
  'GetConfig-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetConfig)))
  "Returns string type for a service object of type '<GetConfig>"
  "ouster_ros/GetConfig")