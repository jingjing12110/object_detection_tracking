;; Auto-generated. Do not edit!


(when (boundp 'tracking_msgs::WaypointState)
  (if (not (find-package "TRACKING_MSGS"))
    (make-package "TRACKING_MSGS"))
  (shadow 'WaypointState (find-package "TRACKING_MSGS")))
(unless (find-package "TRACKING_MSGS::WAYPOINTSTATE")
  (make-package "TRACKING_MSGS::WAYPOINTSTATE"))

(in-package "ROS")
;;//! \htmlinclude WaypointState.msg.html


(intern "*NULLSTATE*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*NULLSTATE* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*NULLSTATE* 0)
(intern "*STR_LEFT*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*STR_LEFT* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*STR_LEFT* 1)
(intern "*STR_RIGHT*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*STR_RIGHT* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*STR_RIGHT* 2)
(intern "*STR_STRAIGHT*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*STR_STRAIGHT* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*STR_STRAIGHT* 3)
(intern "*STR_BACK*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*STR_BACK* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*STR_BACK* 4)
(intern "*TYPE_STOPLINE*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_STOPLINE* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_STOPLINE* 1)
(intern "*TYPE_STOP*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_STOP* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_STOP* 2)
(intern "*TYPE_EVENT_NULL*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_NULL* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_NULL* 0)
(intern "*TYPE_EVENT_GOAL*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_GOAL* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_GOAL* 1)
(intern "*TYPE_EVENT_MIDDLE_GOAL*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_MIDDLE_GOAL* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_MIDDLE_GOAL* 2)
(intern "*TYPE_EVENT_POSITION_STOP*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_POSITION_STOP* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_POSITION_STOP* 3)
(intern "*TYPE_EVENT_BUS_STOP*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_BUS_STOP* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_BUS_STOP* 4)
(intern "*TYPE_EVENT_PARKING*" (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(shadow '*TYPE_EVENT_PARKING* (find-package "TRACKING_MSGS::WAYPOINTSTATE"))
(defconstant tracking_msgs::WaypointState::*TYPE_EVENT_PARKING* 5)
(defclass tracking_msgs::WaypointState
  :super ros::object
  :slots (_aid _lanechange_state _steering_state _accel_state _stop_state _event_state ))

(defmethod tracking_msgs::WaypointState
  (:init
   (&key
    ((:aid __aid) 0)
    ((:lanechange_state __lanechange_state) 0)
    ((:steering_state __steering_state) 0)
    ((:accel_state __accel_state) 0)
    ((:stop_state __stop_state) 0)
    ((:event_state __event_state) 0)
    )
   (send-super :init)
   (setq _aid (round __aid))
   (setq _lanechange_state (round __lanechange_state))
   (setq _steering_state (round __steering_state))
   (setq _accel_state (round __accel_state))
   (setq _stop_state (round __stop_state))
   (setq _event_state (round __event_state))
   self)
  (:aid
   (&optional __aid)
   (if __aid (setq _aid __aid)) _aid)
  (:lanechange_state
   (&optional __lanechange_state)
   (if __lanechange_state (setq _lanechange_state __lanechange_state)) _lanechange_state)
  (:steering_state
   (&optional __steering_state)
   (if __steering_state (setq _steering_state __steering_state)) _steering_state)
  (:accel_state
   (&optional __accel_state)
   (if __accel_state (setq _accel_state __accel_state)) _accel_state)
  (:stop_state
   (&optional __stop_state)
   (if __stop_state (setq _stop_state __stop_state)) _stop_state)
  (:event_state
   (&optional __event_state)
   (if __event_state (setq _event_state __event_state)) _event_state)
  (:serialization-length
   ()
   (+
    ;; int32 _aid
    4
    ;; uint8 _lanechange_state
    1
    ;; uint8 _steering_state
    1
    ;; uint8 _accel_state
    1
    ;; uint8 _stop_state
    1
    ;; uint8 _event_state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _aid
       (write-long _aid s)
     ;; uint8 _lanechange_state
       (write-byte _lanechange_state s)
     ;; uint8 _steering_state
       (write-byte _steering_state s)
     ;; uint8 _accel_state
       (write-byte _accel_state s)
     ;; uint8 _stop_state
       (write-byte _stop_state s)
     ;; uint8 _event_state
       (write-byte _event_state s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _aid
     (setq _aid (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _lanechange_state
     (setq _lanechange_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _steering_state
     (setq _steering_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _accel_state
     (setq _accel_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _stop_state
     (setq _stop_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _event_state
     (setq _event_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get tracking_msgs::WaypointState :md5sum-) "623a0ffbb0a052490adc981c2a3e8edd")
(setf (get tracking_msgs::WaypointState :datatype-) "tracking_msgs/WaypointState")
(setf (get tracking_msgs::WaypointState :definition-)
      "int32 aid
uint8 NULLSTATE=0

# lanechange
uint8 lanechange_state

# bilinker
uint8 steering_state
uint8 STR_LEFT=1
uint8 STR_RIGHT=2
uint8 STR_STRAIGHT=3
uint8 STR_BACK=4

uint8 accel_state

uint8 stop_state
# 1 is stopline, 2 is stop which can only be released manually.
uint8 TYPE_STOPLINE=1
uint8 TYPE_STOP=2

uint8 event_state
uint8 TYPE_EVENT_NULL = 0
uint8 TYPE_EVENT_GOAL = 1
uint8 TYPE_EVENT_MIDDLE_GOAL = 2
uint8 TYPE_EVENT_POSITION_STOP = 3
uint8 TYPE_EVENT_BUS_STOP = 4
uint8 TYPE_EVENT_PARKING = 5

")



(provide :tracking_msgs/WaypointState "623a0ffbb0a052490adc981c2a3e8edd")


