
(cl:in-package :asdf)

(defsystem "rgbdslam-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "rgbdslam_ros_ui" :depends-on ("_package_rgbdslam_ros_ui"))
    (:file "_package_rgbdslam_ros_ui" :depends-on ("_package"))
    (:file "rgbdslam_ros_ui_b" :depends-on ("_package_rgbdslam_ros_ui_b"))
    (:file "_package_rgbdslam_ros_ui_b" :depends-on ("_package"))
    (:file "rgbdslam_ros_ui_f" :depends-on ("_package_rgbdslam_ros_ui_f"))
    (:file "_package_rgbdslam_ros_ui_f" :depends-on ("_package"))
    (:file "rgbdslam_ros_ui_s" :depends-on ("_package_rgbdslam_ros_ui_s"))
    (:file "_package_rgbdslam_ros_ui_s" :depends-on ("_package"))
  ))