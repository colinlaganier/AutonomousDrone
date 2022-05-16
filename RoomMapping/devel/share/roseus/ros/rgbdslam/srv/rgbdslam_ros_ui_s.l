;; Auto-generated. Do not edit!


(when (boundp 'rgbdslam::rgbdslam_ros_ui_s)
  (if (not (find-package "RGBDSLAM"))
    (make-package "RGBDSLAM"))
  (shadow 'rgbdslam_ros_ui_s (find-package "RGBDSLAM")))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_S")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_S"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_SREQUEST")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_SREQUEST"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_SRESPONSE")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_SRESPONSE"))

(in-package "ROS")





(defclass rgbdslam::rgbdslam_ros_ui_sRequest
  :super ros::object
  :slots (_command _value ))

(defmethod rgbdslam::rgbdslam_ros_ui_sRequest
  (:init
   (&key
    ((:command __command) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _value (string __value))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rgbdslam::rgbdslam_ros_ui_sResponse
  :super ros::object
  :slots ())

(defmethod rgbdslam::rgbdslam_ros_ui_sResponse
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

(defclass rgbdslam::rgbdslam_ros_ui_s
  :super ros::object
  :slots ())

(setf (get rgbdslam::rgbdslam_ros_ui_s :md5sum-) "406bad1a44daaa500258274f332bb924")
(setf (get rgbdslam::rgbdslam_ros_ui_s :datatype-) "rgbdslam/rgbdslam_ros_ui_s")
(setf (get rgbdslam::rgbdslam_ros_ui_s :request) rgbdslam::rgbdslam_ros_ui_sRequest)
(setf (get rgbdslam::rgbdslam_ros_ui_s :response) rgbdslam::rgbdslam_ros_ui_sResponse)

(defmethod rgbdslam::rgbdslam_ros_ui_sRequest
  (:response () (instance rgbdslam::rgbdslam_ros_ui_sResponse :init)))

(setf (get rgbdslam::rgbdslam_ros_ui_sRequest :md5sum-) "406bad1a44daaa500258274f332bb924")
(setf (get rgbdslam::rgbdslam_ros_ui_sRequest :datatype-) "rgbdslam/rgbdslam_ros_ui_sRequest")
(setf (get rgbdslam::rgbdslam_ros_ui_sRequest :definition-)
      "string command
string value
---

")

(setf (get rgbdslam::rgbdslam_ros_ui_sResponse :md5sum-) "406bad1a44daaa500258274f332bb924")
(setf (get rgbdslam::rgbdslam_ros_ui_sResponse :datatype-) "rgbdslam/rgbdslam_ros_ui_sResponse")
(setf (get rgbdslam::rgbdslam_ros_ui_sResponse :definition-)
      "string command
string value
---

")



(provide :rgbdslam/rgbdslam_ros_ui_s "406bad1a44daaa500258274f332bb924")


