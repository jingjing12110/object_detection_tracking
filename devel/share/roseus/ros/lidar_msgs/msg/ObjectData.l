;; Auto-generated. Do not edit!


(when (boundp 'lidar_msgs::ObjectData)
  (if (not (find-package "LIDAR_MSGS"))
    (make-package "LIDAR_MSGS"))
  (shadow 'ObjectData (find-package "LIDAR_MSGS")))
(unless (find-package "LIDAR_MSGS::OBJECTDATA")
  (make-package "LIDAR_MSGS::OBJECTDATA"))

(in-package "ROS")
;;//! \htmlinclude ObjectData.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lidar_msgs::ObjectData
  :super ros::object
  :slots (_header _bbox _token ))

(defmethod lidar_msgs::ObjectData
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:bbox __bbox) (let (r) (dotimes (i 0) (push (instance lidar_msgs::Object :init) r)) r))
    ((:token __token) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _bbox __bbox)
   (setq _token (string __token))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:token
   (&optional __token)
   (if __token (setq _token __token)) _token)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; lidar_msgs/Object[] _bbox
    (apply #'+ (send-all _bbox :serialization-length)) 4
    ;; string _token
    4 (length _token)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; lidar_msgs/Object[] _bbox
     (write-long (length _bbox) s)
     (dolist (elem _bbox)
       (send elem :serialize s)
       )
     ;; string _token
       (write-long (length _token) s) (princ _token s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; lidar_msgs/Object[] _bbox
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _bbox (let (r) (dotimes (i n) (push (instance lidar_msgs::Object :init) r)) r))
     (dolist (elem- _bbox)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string _token
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _token (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get lidar_msgs::ObjectData :md5sum-) "cce2fe89002b3c1acf48988217c33108")
(setf (get lidar_msgs::ObjectData :datatype-) "lidar_msgs/ObjectData")
(setf (get lidar_msgs::ObjectData :definition-)
      "Header header

Object[] bbox
string token

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: lidar_msgs/Object

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



(provide :lidar_msgs/ObjectData "cce2fe89002b3c1acf48988217c33108")


