; Auto-generated. Do not edit!


(cl:in-package ouster_ros-srv)


;//! \htmlinclude GetMetadata-request.msg.html

(cl:defclass <GetMetadata-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetMetadata-request (<GetMetadata-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMetadata-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMetadata-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<GetMetadata-request> is deprecated: use ouster_ros-srv:GetMetadata-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMetadata-request>) ostream)
  "Serializes a message object of type '<GetMetadata-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMetadata-request>) istream)
  "Deserializes a message object of type '<GetMetadata-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMetadata-request>)))
  "Returns string type for a service object of type '<GetMetadata-request>"
  "ouster_ros/GetMetadataRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMetadata-request)))
  "Returns string type for a service object of type 'GetMetadata-request"
  "ouster_ros/GetMetadataRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMetadata-request>)))
  "Returns md5sum for a message object of type '<GetMetadata-request>"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMetadata-request)))
  "Returns md5sum for a message object of type 'GetMetadata-request"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMetadata-request>)))
  "Returns full string definition for message of type '<GetMetadata-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMetadata-request)))
  "Returns full string definition for message of type 'GetMetadata-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMetadata-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMetadata-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMetadata-request
))
;//! \htmlinclude GetMetadata-response.msg.html

(cl:defclass <GetMetadata-response> (roslisp-msg-protocol:ros-message)
  ((metadata
    :reader metadata
    :initarg :metadata
    :type cl:string
    :initform ""))
)

(cl:defclass GetMetadata-response (<GetMetadata-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetMetadata-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetMetadata-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ouster_ros-srv:<GetMetadata-response> is deprecated: use ouster_ros-srv:GetMetadata-response instead.")))

(cl:ensure-generic-function 'metadata-val :lambda-list '(m))
(cl:defmethod metadata-val ((m <GetMetadata-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ouster_ros-srv:metadata-val is deprecated.  Use ouster_ros-srv:metadata instead.")
  (metadata m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetMetadata-response>) ostream)
  "Serializes a message object of type '<GetMetadata-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'metadata))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'metadata))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetMetadata-response>) istream)
  "Deserializes a message object of type '<GetMetadata-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'metadata) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'metadata) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetMetadata-response>)))
  "Returns string type for a service object of type '<GetMetadata-response>"
  "ouster_ros/GetMetadataResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMetadata-response)))
  "Returns string type for a service object of type 'GetMetadata-response"
  "ouster_ros/GetMetadataResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetMetadata-response>)))
  "Returns md5sum for a message object of type '<GetMetadata-response>"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetMetadata-response)))
  "Returns md5sum for a message object of type 'GetMetadata-response"
  "d37888e5a47bef783c189dec5351027e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetMetadata-response>)))
  "Returns full string definition for message of type '<GetMetadata-response>"
  (cl:format cl:nil "string metadata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetMetadata-response)))
  "Returns full string definition for message of type 'GetMetadata-response"
  (cl:format cl:nil "string metadata~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetMetadata-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'metadata))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetMetadata-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetMetadata-response
    (cl:cons ':metadata (metadata msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetMetadata)))
  'GetMetadata-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetMetadata)))
  'GetMetadata-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetMetadata)))
  "Returns string type for a service object of type '<GetMetadata>"
  "ouster_ros/GetMetadata")