;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::Point3)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'Point3 (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::POINT3")
  (make-package "LIDAR_MSGS::POINT3"))

(in-package "ROS")
;;//! \htmlinclude Point3.msg.html


(defclass lidar_msgs::Point3
  :super ros::object
  :slots (_x _y _z ))

(defmethod lidar_msgs::Point3
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:z __z) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _z (float __z))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:z
   (&optional __z)
   (if __z (setq _z __z)) _z)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; float32 _z
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _z
       (sys::poke _z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _z
     (setq _z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get lidar_msgs::Point3 :md5sum-) "cc153912f1453b708d221682bc23d9ac")
(setf (get lidar_msgs::Point3 :datatype-) "lidar_msgs/Point3")
(setf (get lidar_msgs::Point3 :definition-)
      "float32 x
float32 y
float32 z
")



(provide :lidar_msgs/Point3 "cc153912f1453b708d221682bc23d9ac")


