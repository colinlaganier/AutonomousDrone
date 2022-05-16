; Auto-generated. Do not edit!


(cl:in-package rgbdslam-srv)


;//! \htmlinclude rgbdslam_ros_ui_f-request.msg.html

(cl:defclass <rgbdslam_ros_ui_f-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass rgbdslam_ros_ui_f-request (<rgbdslam_ros_ui_f-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui_f-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui_f-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui_f-request> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui_f-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <rgbdslam_ros_ui_f-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rgbdslam-srv:command-val is deprecated.  Use rgbdslam-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <rgbdslam_ros_ui_f-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rgbdslam-srv:value-val is deprecated.  Use rgbdslam-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui_f-request>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui_f-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui_f-request>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui_f-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui_f-request>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_f-request>"
  "rgbdslam/rgbdslam_ros_ui_fRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_f-request)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui_f-request"
  "rgbdslam/rgbdslam_ros_ui_fRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui_f-request>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui_f-request>"
  "d8f674e014809463d0a122a49c328a89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui_f-request)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui_f-request"
  "d8f674e014809463d0a122a49c328a89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui_f-request>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui_f-request>"
  (cl:format cl:nil "string command~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui_f-request)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui_f-request"
  (cl:format cl:nil "string command~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui_f-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui_f-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui_f-request
    (cl:cons ':command (command msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude rgbdslam_ros_ui_f-response.msg.html

(cl:defclass <rgbdslam_ros_ui_f-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass rgbdslam_ros_ui_f-response (<rgbdslam_ros_ui_f-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui_f-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui_f-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui_f-response> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui_f-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui_f-response>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui_f-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui_f-response>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui_f-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui_f-response>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_f-response>"
  "rgbdslam/rgbdslam_ros_ui_fResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_f-response)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui_f-response"
  "rgbdslam/rgbdslam_ros_ui_fResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui_f-response>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui_f-response>"
  "d8f674e014809463d0a122a49c328a89")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui_f-response)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui_f-response"
  "d8f674e014809463d0a122a49c328a89")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui_f-response>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui_f-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui_f-response)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui_f-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui_f-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui_f-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui_f-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rgbdslam_ros_ui_f)))
  'rgbdslam_ros_ui_f-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rgbdslam_ros_ui_f)))
  'rgbdslam_ros_ui_f-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_f)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_f>"
  "rgbdslam/rgbdslam_ros_ui_f")