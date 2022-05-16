; Auto-generated. Do not edit!


(cl:in-package rgbdslam-srv)


;//! \htmlinclude rgbdslam_ros_ui-request.msg.html

(cl:defclass <rgbdslam_ros_ui-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass rgbdslam_ros_ui-request (<rgbdslam_ros_ui-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui-request> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <rgbdslam_ros_ui-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rgbdslam-srv:command-val is deprecated.  Use rgbdslam-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui-request>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui-request>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui-request>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui-request>"
  "rgbdslam/rgbdslam_ros_uiRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui-request)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui-request"
  "rgbdslam/rgbdslam_ros_uiRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui-request>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui-request>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui-request)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui-request"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui-request>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui-request>"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui-request)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui-request"
  (cl:format cl:nil "string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude rgbdslam_ros_ui-response.msg.html

(cl:defclass <rgbdslam_ros_ui-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass rgbdslam_ros_ui-response (<rgbdslam_ros_ui-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui-response> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui-response>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui-response>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui-response>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui-response>"
  "rgbdslam/rgbdslam_ros_uiResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui-response)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui-response"
  "rgbdslam/rgbdslam_ros_uiResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui-response>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui-response>"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui-response)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui-response"
  "cba5e21e920a3a2b7b375cb65b64cdea")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui-response>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui-response)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rgbdslam_ros_ui)))
  'rgbdslam_ros_ui-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rgbdslam_ros_ui)))
  'rgbdslam_ros_ui-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui>"
  "rgbdslam/rgbdslam_ros_ui")