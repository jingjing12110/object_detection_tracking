;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::Object)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'Object (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::OBJECT")
  (make-package "LIDAR_MSGS::OBJECT"))

(in-package "ROS")
;;//! \htmlinclude Object.msg.html


(defclass lidar_msgs::Object
  :super ros::object
  :slots (_center _size _velocity _corners _contours _id _predict_covariance ))

(defmethod lidar_msgs::Object
  (:init
   (&key
    ((:center __center) (instance lidar_msgs::Point3 :init))
    ((:size __size) (instance lidar_msgs::Point3 :init))
    ((:velocity __velocity) (instance lidar_msgs::Point2 :init))
    ((:corners __corners) (let (r) (dotimes (i 0) (push (instance lidar_msgs::Point2 :init) r)) r))
    ((:contours __contours) (let (r) (dotimes (i 0) (push (instance lidar_msgs::Point2 :init) r)) r))
    ((:id __id) 0)
    ((:predict_covariance __predict_covariance) (make-array 36 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _center __center)
   (setq _size __size)
   (setq _velocity __velocity)
   (setq _corners __corners)
   (setq _contours __contours)
   (setq _id (round __id))
   (setq _predict_covariance __predict_covariance)
   self)
  (:center
   (&rest __center)
   (if (keywordp (car __center))
       (send* _center __center)
     (progn
       (if __center (setq _center (car __center)))
       _center)))
  (:size
   (&rest __size)
   (if (keywordp (car __size))
       (send* _size __size)
     (progn
       (if __size (setq _size (car __size)))
       _size)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:corners
   (&rest __corners)
   (if (keywordp (car __corners))
       (send* _corners __corners)
     (progn
       (if __corners (setq _corners (car __corners)))
       _corners)))
  (:contours
   (&rest __contours)
   (if (keywordp (car __contours))
       (send* _contours __contours)
     (progn
       (if __contours (setq _contours (car __contours)))
       _contours)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:predict_covariance
   (&optional __predict_covariance)
   (if __predict_covariance (setq _predict_covariance __predict_covariance)) _predict_covariance)
  (:serialization-length
   ()
   (+
    ;; lidar_msgs/Point3 _center
    (send _center :serialization-length)
    ;; lidar_msgs/Point3 _size
    (send _size :serialization-length)
    ;; lidar_msgs/Point2 _velocity
    (send _velocity :serialization-length)
    ;; lidar_msgs/Point2[] _corners
    (apply #'+ (send-all _corners :serialization-length)) 4
    ;; lidar_msgs/Point2[] _contours
    (apply #'+ (send-all _contours :serialization-length)) 4
    ;; int16 _id
    2
    ;; float64[36] _predict_covariance
    (* 8    36)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; lidar_msgs/Point3 _center
       (send _center :serialize s)
     ;; lidar_msgs/Point3 _size
       (send _size :serialize s)
     ;; lidar_msgs/Point2 _velocity
       (send _velocity :serialize s)
     ;; lidar_msgs/Point2[] _corners
     (write-long (length _corners) s)
     (dolist (elem _corners)
       (send elem :serialize s)
       )
     ;; lidar_msgs/Point2[] _contours
     (write-long (length _contours) s)
     (dolist (elem _contours)
       (send elem :serialize s)
       )
     ;; int16 _id
       (write-word _id s)
     ;; float64[36] _predict_covariance
     (dotimes (i 36)
       (sys::poke (elt _predict_covariance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; lidar_msgs/Point3 _center
     (send _center :deserialize buf ptr-) (incf ptr- (send _center :serialization-length))
   ;; lidar_msgs/Point3 _size
     (send _size :deserialize buf ptr-) (incf ptr- (send _size :serialization-length))
   ;; lidar_msgs/Point2 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; lidar_msgs/Point2[] _corners
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _corners (let (r) (dotimes (i n) (push (instance lidar_msgs::Point2 :init) r)) r))
     (dolist (elem- _corners)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; lidar_msgs/Point2[] _contours
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _contours (let (r) (dotimes (i n) (push (instance lidar_msgs::Point2 :init) r)) r))
     (dolist (elem- _contours)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; int16 _id
     (setq _id (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64[36] _predict_covariance
   (dotimes (i (length _predict_covariance))
     (setf (elt _predict_covariance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get lidar_msgs::Object :md5sum-) "be0179af1f0ec1bf6324c4d7897b9aac")
(setf (get lidar_msgs::Object :datatype-) "lidar_msgs/Object")
(setf (get lidar_msgs::Object :definition-)
      "
Point3 center
Point3 size
Point2 velocity
Point2[] corners
Point2[] contours
int16 id

float64[36] predict_covariance



================================================================================
MSG: lidar_msgs/Point3
float32 x
float32 y
float32 z
================================================================================
MSG: lidar_msgs/Point2
float32 x
float32 y

")



(provide :lidar_msgs/Object "be0179af1f0ec1bf6324c4d7897b9aac")


