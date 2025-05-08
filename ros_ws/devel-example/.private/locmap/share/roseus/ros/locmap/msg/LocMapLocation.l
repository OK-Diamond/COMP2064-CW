;; Auto-generated. Do not edit!


(when (boundp 'locmap::LocMapLocation)
  (if (not (find-package "LOCMAP"))
    (make-package "LOCMAP"))
  (shadow 'LocMapLocation (find-package "LOCMAP")))
(unless (find-package "LOCMAP::LOCMAPLOCATION")
  (make-package "LOCMAP::LOCMAPLOCATION"))

(in-package "ROS")
;;//! \htmlinclude LocMapLocation.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass locmap::LocMapLocation
  :super ros::object
  :slots (_header _pose _name _group ))

(defmethod locmap::LocMapLocation
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:pose __pose) (instance geometry_msgs::PoseStamped :init))
    ((:name __name) "")
    ((:group __group) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _pose __pose)
   (setq _name (string __name))
   (setq _group (string __group))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:group
   (&optional __group)
   (if __group (setq _group __group)) _group)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped _pose
    (send _pose :serialization-length)
    ;; string _name
    4 (length _name)
    ;; string _group
    4 (length _group)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped _pose
       (send _pose :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;; string _group
       (write-long (length _group) s) (princ _group s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _group
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _group (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get locmap::LocMapLocation :md5sum-) "9db4e1ca22b055ae17262848c33aa6fa")
(setf (get locmap::LocMapLocation :datatype-) "locmap/LocMapLocation")
(setf (get locmap::LocMapLocation :definition-)
      "Header header

geometry_msgs/PoseStamped pose
string name
string group
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
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :locmap/LocMapLocation "9db4e1ca22b055ae17262848c33aa6fa")


