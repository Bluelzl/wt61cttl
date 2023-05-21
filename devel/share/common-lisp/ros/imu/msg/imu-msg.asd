
(cl:in-package :asdf)

(defsystem "imu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "WT61_IMU" :depends-on ("_package_WT61_IMU"))
    (:file "_package_WT61_IMU" :depends-on ("_package"))
  ))