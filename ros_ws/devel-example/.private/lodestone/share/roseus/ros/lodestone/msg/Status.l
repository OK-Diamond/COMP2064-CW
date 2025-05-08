;; Auto-generated. Do not edit!


(when (boundp 'lodestone::Status)
  (if (not (find-package "LODESTONE"))
    (make-package "LODESTONE"))
  (shadow 'Status (find-package "LODESTONE")))
(unless (find-package "LODESTONE::STATUS")
  (make-package "LODESTONE::STATUS"))

(in-package "ROS")
;;//! \htmlinclude Status.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lodestone::Status
  :super ros::object
  :slots (_header _code _name ))

(defmethod lodestone::Status
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:code __code) 0)
    ((:name __name) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _code (round __code))
   (setq _name (string __name))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:code
   (&optional __code)
   (if __code (setq _code __code)) _code)
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint8 _code
    1
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint8 _code
       (write-byte _code s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint8 _code
     (setq _code (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get lodestone::Status :md5sum-) "50dcb3c77e77cc53312ff8a83fd7929b")
(setf (get lodestone::Status :datatype-) "lodestone/Status")
(setf (get lodestone::Status :definition-)
      "Header header

uint8 code
string name
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



(provide :lodestone/Status "50dcb3c77e77cc53312ff8a83fd7929b")


