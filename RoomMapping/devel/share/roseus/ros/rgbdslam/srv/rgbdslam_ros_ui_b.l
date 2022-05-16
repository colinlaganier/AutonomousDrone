;; Auto-generated. Do not edit!


(when (boundp 'rgbdslam::rgbdslam_ros_ui_b)
  (if (not (find-package "RGBDSLAM"))
    (make-package "RGBDSLAM"))
  (shadow 'rgbdslam_ros_ui_b (find-package "RGBDSLAM")))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_B")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_B"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_BREQUEST")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_BREQUEST"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI_BRESPONSE")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI_BRESPONSE"))

(in-package "ROS")





(defclass rgbdslam::rgbdslam_ros_ui_bRequest
  :super ros::object
  :slots (_command _value ))

(defmethod rgbdslam::rgbdslam_ros_ui_bRequest
  (:init
   (&key
    ((:command __command) "")
    ((:value __value) nil)
    )
   (send-super :init)
   (setq _command (string __command))
   (setq _value __value)
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
    ;; bool _value
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;; bool _value
       (if _value (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _value
     (setq _value (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass rgbdslam::rgbdslam_ros_ui_bResponse
  :super ros::object
  :slots ())

(defmethod rgbdslam::rgbdslam_ros_ui_bResponse
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

(defclass rgbdslam::rgbdslam_ros_ui_b
  :super ros::object
  :slots ())

(setf (get rgbdslam::rgbdslam_ros_ui_b :md5sum-) "95aa0151a35e3de365041ffa089ce8c7")
(setf (get rgbdslam::rgbdslam_ros_ui_b :datatype-) "rgbdslam/rgbdslam_ros_ui_b")
(setf (get rgbdslam::rgbdslam_ros_ui_b :request) rgbdslam::rgbdslam_ros_ui_bRequest)
(setf (get rgbdslam::rgbdslam_ros_ui_b :response) rgbdslam::rgbdslam_ros_ui_bResponse)

(defmethod rgbdslam::rgbdslam_ros_ui_bRequest
  (:response () (instance rgbdslam::rgbdslam_ros_ui_bResponse :init)))

(setf (get rgbdslam::rgbdslam_ros_ui_bRequest :md5sum-) "95aa0151a35e3de365041ffa089ce8c7")
(setf (get rgbdslam::rgbdslam_ros_ui_bRequest :datatype-) "rgbdslam/rgbdslam_ros_ui_bRequest")
(setf (get rgbdslam::rgbdslam_ros_ui_bRequest :definition-)
      "string command
bool value
---
")

(setf (get rgbdslam::rgbdslam_ros_ui_bResponse :md5sum-) "95aa0151a35e3de365041ffa089ce8c7")
(setf (get rgbdslam::rgbdslam_ros_ui_bResponse :datatype-) "rgbdslam/rgbdslam_ros_ui_bResponse")
(setf (get rgbdslam::rgbdslam_ros_ui_bResponse :definition-)
      "string command
bool value
---
")



(provide :rgbdslam/rgbdslam_ros_ui_b "95aa0151a35e3de365041ffa089ce8c7")


