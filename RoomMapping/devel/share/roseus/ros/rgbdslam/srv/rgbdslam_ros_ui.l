;; Auto-generated. Do not edit!


(when (boundp 'rgbdslam::rgbdslam_ros_ui)
  (if (not (find-package "RGBDSLAM"))
    (make-package "RGBDSLAM"))
  (shadow 'rgbdslam_ros_ui (find-package "RGBDSLAM")))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UI")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UI"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UIREQUEST")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UIREQUEST"))
(unless (find-package "RGBDSLAM::RGBDSLAM_ROS_UIRESPONSE")
  (make-package "RGBDSLAM::RGBDSLAM_ROS_UIRESPONSE"))

(in-package "ROS")





(defclass rgbdslam::rgbdslam_ros_uiRequest
  :super ros::object
  :slots (_command ))

(defmethod rgbdslam::rgbdslam_ros_uiRequest
  (:init
   (&key
    ((:command __command) "")
    )
   (send-super :init)
   (setq _command (string __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; string _command
    4 (length _command)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _command
       (write-long (length _command) s) (princ _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _command
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _command (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass rgbdslam::rgbdslam_ros_uiResponse
  :super ros::object
  :slots ())

(defmethod rgbdslam::rgbdslam_ros_uiResponse
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

(defclass rgbdslam::rgbdslam_ros_ui
  :super ros::object
  :slots ())

(setf (get rgbdslam::rgbdslam_ros_ui :md5sum-) "cba5e21e920a3a2b7b375cb65b64cdea")
(setf (get rgbdslam::rgbdslam_ros_ui :datatype-) "rgbdslam/rgbdslam_ros_ui")
(setf (get rgbdslam::rgbdslam_ros_ui :request) rgbdslam::rgbdslam_ros_uiRequest)
(setf (get rgbdslam::rgbdslam_ros_ui :response) rgbdslam::rgbdslam_ros_uiResponse)

(defmethod rgbdslam::rgbdslam_ros_uiRequest
  (:response () (instance rgbdslam::rgbdslam_ros_uiResponse :init)))

(setf (get rgbdslam::rgbdslam_ros_uiRequest :md5sum-) "cba5e21e920a3a2b7b375cb65b64cdea")
(setf (get rgbdslam::rgbdslam_ros_uiRequest :datatype-) "rgbdslam/rgbdslam_ros_uiRequest")
(setf (get rgbdslam::rgbdslam_ros_uiRequest :definition-)
      "string command
---
")

(setf (get rgbdslam::rgbdslam_ros_uiResponse :md5sum-) "cba5e21e920a3a2b7b375cb65b64cdea")
(setf (get rgbdslam::rgbdslam_ros_uiResponse :datatype-) "rgbdslam/rgbdslam_ros_uiResponse")
(setf (get rgbdslam::rgbdslam_ros_uiResponse :definition-)
      "string command
---
")



(provide :rgbdslam/rgbdslam_ros_ui "cba5e21e920a3a2b7b375cb65b64cdea")


