
(cl:in-package :asdf)

(defsystem "lidar_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Object" :depends-on ("_package_Object"))
    (:file "_package_Object" :depends-on ("_package"))
    (:file "ObjectData" :depends-on ("_package_ObjectData"))
    (:file "_package_ObjectData" :depends-on ("_package"))
    (:file "Point2" :depends-on ("_package_Point2"))
    (:file "_package_Point2" :depends-on ("_package"))
    (:file "Point3" :depends-on ("_package_Point3"))
    (:file "_package_Point3" :depends-on ("_package"))
    (:file "PointF" :depends-on ("_package_PointF"))
    (:file "_package_PointF" :depends-on ("_package"))
  ))