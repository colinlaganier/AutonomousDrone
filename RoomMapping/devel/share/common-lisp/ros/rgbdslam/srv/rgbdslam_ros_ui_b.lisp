; Auto-generated. Do not edit!


(cl:in-package rgbdslam-srv)


;//! \htmlinclude rgbdslam_ros_ui_b-request.msg.html

(cl:defclass <rgbdslam_ros_ui_b-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass rgbdslam_ros_ui_b-request (<rgbdslam_ros_ui_b-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui_b-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui_b-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui_b-request> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui_b-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <rgbdslam_ros_ui_b-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rgbdslam-srv:command-val is deprecated.  Use rgbdslam-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <rgbdslam_ros_ui_b-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rgbdslam-srv:value-val is deprecated.  Use rgbdslam-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui_b-request>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui_b-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui_b-request>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui_b-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui_b-request>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_b-request>"
  "rgbdslam/rgbdslam_ros_ui_bRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_b-request)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui_b-request"
  "rgbdslam/rgbdslam_ros_ui_bRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui_b-request>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui_b-request>"
  "95aa0151a35e3de365041ffa089ce8c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui_b-request)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui_b-request"
  "95aa0151a35e3de365041ffa089ce8c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui_b-request>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui_b-request>"
  (cl:format cl:nil "string command~%bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui_b-request)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui_b-request"
  (cl:format cl:nil "string command~%bool value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui_b-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'command))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui_b-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui_b-request
    (cl:cons ':command (command msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude rgbdslam_ros_ui_b-response.msg.html

(cl:defclass <rgbdslam_ros_ui_b-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass rgbdslam_ros_ui_b-response (<rgbdslam_ros_ui_b-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgbdslam_ros_ui_b-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgbdslam_ros_ui_b-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rgbdslam-srv:<rgbdslam_ros_ui_b-response> is deprecated: use rgbdslam-srv:rgbdslam_ros_ui_b-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgbdslam_ros_ui_b-response>) ostream)
  "Serializes a message object of type '<rgbdslam_ros_ui_b-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgbdslam_ros_ui_b-response>) istream)
  "Deserializes a message object of type '<rgbdslam_ros_ui_b-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgbdslam_ros_ui_b-response>)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_b-response>"
  "rgbdslam/rgbdslam_ros_ui_bResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_b-response)))
  "Returns string type for a service object of type 'rgbdslam_ros_ui_b-response"
  "rgbdslam/rgbdslam_ros_ui_bResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgbdslam_ros_ui_b-response>)))
  "Returns md5sum for a message object of type '<rgbdslam_ros_ui_b-response>"
  "95aa0151a35e3de365041ffa089ce8c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgbdslam_ros_ui_b-response)))
  "Returns md5sum for a message object of type 'rgbdslam_ros_ui_b-response"
  "95aa0151a35e3de365041ffa089ce8c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgbdslam_ros_ui_b-response>)))
  "Returns full string definition for message of type '<rgbdslam_ros_ui_b-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgbdslam_ros_ui_b-response)))
  "Returns full string definition for message of type 'rgbdslam_ros_ui_b-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgbdslam_ros_ui_b-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgbdslam_ros_ui_b-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rgbdslam_ros_ui_b-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rgbdslam_ros_ui_b)))
  'rgbdslam_ros_ui_b-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rgbdslam_ros_ui_b)))
  'rgbdslam_ros_ui_b-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgbdslam_ros_ui_b)))
  "Returns string type for a service object of type '<rgbdslam_ros_ui_b>"
  "rgbdslam/rgbdslam_ros_ui_b")