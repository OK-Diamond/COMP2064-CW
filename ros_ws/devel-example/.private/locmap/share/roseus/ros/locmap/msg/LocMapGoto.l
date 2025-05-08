;; Auto-generated. Do not edit!


(when (boundp 'locmap::LocMapGoto)
  (if (not (find-package "LOCMAP"))
    (make-package "LOCMAP"))
  (shadow 'LocMapGoto (find-package "LOCMAP")))
(unless (find-package "LOCMAP::LOCMAPGOTO")
  (make-package "LOCMAP::LOCMAPGOTO"))

(in-package "ROS")
;;//! \htmlinclude LocMapGoto.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass locmap::LocMapGoto
  :super ros::object
  :slots (_header _location_name ))

(defmethod locmap::LocMapGoto
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:location_name __location_name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _location_name (string __location_name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:location_name
   (&optional __location_name)
   (if __location_name (setq _location_name __location_name)) _location_name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _location_name
    4 (length _location_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _location_name
       (write-long (length _location_name) s) (princ _location_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _location_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _location_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get locmap::LocMapGoto :md5sum-) "a2a536bbdbeabd2be949e6fb00d6394b")
(setf (get locmap::LocMapGoto :datatype-) "locmap/LocMapGoto")
(setf (get locmap::LocMapGoto :definition-)
      "Header header

string location_name
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

")



(provide :locmap/LocMapGoto "a2a536bbdbeabd2be949e6fb00d6394b")


