;; Auto-generated. Do not edit!


(when (boundp 'rgbdslam::rgbdslam_ros_ui_f)
  (if (not (find-package "RGBDSLAM"))
    (make-package "RGBDSLAM"))
  (shadow 'rgbdslam_ros_ui_f (find-package "RGBDSLAM")))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_F")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_F"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_FREQUEST")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_FREQUEST"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_FRESPONSE")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_FRESPONSE"))

(in-package "ROS")





(defclass rgbdslam::rgbdslam_ros_ui_fRequest
  :super ros::object
  :slots (_command _value ))

(defmethod rgbdslam::rgbdslam_ros_ui_fRequest
  (:init
   (&key
    ((:command __command) "")
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _value (float __value))
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
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass rgbdslam::rgbdslam_ros_ui_fResponse
  :super ros::object
  :slots ())

(defmethod rgbdslam::rgbdslam_ros_ui_fResponse
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

(defclass rgbdslam::rgbdslam_ros_ui_f
  :super ros::object
  :slots ())

(setf (get rgbdslam::rgbdslam_ros_ui_f :md5sum-) "d8f674e014809463d0a122a49c328a89")
(setf (get rgbdslam::rgbdslam_ros_ui_f :datatype-) "rgbdslam/rgbdslam_ros_ui_f")
(setf (get rgbdslam::rgbdslam_ros_ui_f :request) rgbdslam::rgbdslam_ros_ui_fRequest)
(setf (get rgbdslam::rgbdslam_ros_ui_f :response) rgbdslam::rgbdslam_ros_ui_fResponse)

(defmethod rgbdslam::rgbdslam_ros_ui_fRequest
  (:response () (instance rgbdslam::rgbdslam_ros_ui_fResponse :init)))

(setf (get rgbdslam::rgbdslam_ros_ui_fRequest :md5sum-) "d8f674e014809463d0a122a49c328a89")
(setf (get rgbdslam::rgbdslam_ros_ui_fRequest :datatype-) "rgbdslam/rgbdslam_ros_ui_fRequest")
(setf (get rgbdslam::rgbdslam_ros_ui_fRequest :definition-)
      "string command
float32 value
---
")

(setf (get rgbdslam::rgbdslam_ros_ui_fResponse :md5sum-) "d8f674e014809463d0a122a49c328a89")
(setf (get rgbdslam::rgbdslam_ros_ui_fResponse :datatype-) "rgbdslam/rgbdslam_ros_ui_fResponse")
(setf (get rgbdslam::rgbdslam_ros_ui_fResponse :definition-)
      "string command
float32 value
---
")



(provide :rgbdslam/rgbdslam_ros_ui_f "d8f674e014809463d0a122a49c328a89")


