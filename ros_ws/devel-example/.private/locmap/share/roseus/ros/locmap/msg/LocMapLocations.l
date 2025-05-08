;; Auto-generated. Do not edit!


(when (boundp 'locmap::LocMapLocations)
  (if (not (find-package "LOCMAP"))
    (make-package "LOCMAP"))
  (shadow 'LocMapLocations (find-package "LOCMAP")))
(unless (find-package "LOCMAP::LOCMAPLOCATIONS")
  (make-package "LOCMAP::LOCMAPLOCATIONS"))

(in-package "ROS")
;;//! \htmlinclude LocMapLocations.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass locmap::LocMapLocations
  :super ros::object
  :slots (_header _locations _groups ))

(defmethod locmap::LocMapLocations
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:locations __locations) ())
    ((:groups __groups) (let (r) (dotimes (i 0) (push "" r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _locations __locations)
   (setq _groups __groups)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:locations
   (&rest __locations)
   (if (keywordp (car __locations))
       (send* _locations __locations)
     (progn
       (if __locations (setq _locations (car __locations)))
       _locations)))
  (:groups
   (&optional __groups)
   (if __groups (setq _groups __groups)) _groups)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; locmap/LocMapLocation[] _locations
    (apply #'+ (send-all _locations :serialization-length)) 4
    ;; string[] _groups
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _groups)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; locmap/LocMapLocation[] _locations
     (write-long (length _locations) s)
     (dolist (elem _locations)
       (send elem :serialize s)
       )
     ;; string[] _groups
     (write-long (length _groups) s)
     (dolist (elem _groups)
       (write-long (length elem) s) (princ elem s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; locmap/LocMapLocation[] _locations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _locations (let (r) (dotimes (i n) (push (instance locmap::LocMapLocation :init) r)) r))
     (dolist (elem- _locations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; string[] _groups
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _groups (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _groups i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;;
   self)
  )

(setf (get locmap::LocMapLocations :md5sum-) "3bd3045c7f0b78ca25575427c64ec7d5")
(setf (get locmap::LocMapLocations :datatype-) "locmap/LocMapLocations")
(setf (get locmap::LocMapLocations :definition-)
      "Header header

LocMapLocation[] locations
string[] groups
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
MSG: locmap/LocMapLocation
Header header

geometry_msgs/PoseStamped pose
string name
string group
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



(provide :locmap/LocMapLocations "3bd3045c7f0b78ca25575427c64ec7d5")


