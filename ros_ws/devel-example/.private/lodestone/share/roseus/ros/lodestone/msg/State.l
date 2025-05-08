;; Auto-generated. Do not edit!


(when (boundp 'lodestone::State)
  (if (not (find-package "LODESTONE"))
    (make-package "LODESTONE"))
  (shadow 'State (find-package "LODESTONE")))
(unless (find-package "LODESTONE::STATE")
  (make-package "LODESTONE::STATE"))

(in-package "ROS")
;;//! \htmlinclude State.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lodestone::State
  :super ros::object
  :slots (_header _state_code _state_name _status_code _status_name _patients _free_gps ))

(defmethod lodestone::State
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:state_code __state_code) 0)
    ((:state_name __state_name) "")
    ((:status_code __status_code) 0)
    ((:status_name __status_name) "")
    ((:patients __patients) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:free_gps __free_gps) (let (r) (dotimes (i 0) (push nil r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _state_code (round __state_code))
   (setq _state_name (string __state_name))
   (setq _status_code (round __status_code))
   (setq _status_name (string __status_name))
   (setq _patients __patients)
   (setq _free_gps __free_gps)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:state_code
   (&optional __state_code)
   (if __state_code (setq _state_code __state_code)) _state_code)
  (:state_name
   (&optional __state_name)
   (if __state_name (setq _state_name __state_name)) _state_name)
  (:status_code
   (&optional __status_code)
   (if __status_code (setq _status_code __status_code)) _status_code)
  (:status_name
   (&optional __status_name)
   (if __status_name (setq _status_name __status_name)) _status_name)
  (:patients
   (&optional __patients)
   (if __patients (setq _patients __patients)) _patients)
  (:free_gps
   (&optional __free_gps)
   (if __free_gps (setq _free_gps __free_gps)) _free_gps)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int8 _state_code
    1
    ;; string _state_name
    4 (length _state_name)
    ;; int8 _status_code
    1
    ;; string _status_name
    4 (length _status_name)
    ;; string[] _patients
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _patients)) 4
    ;; bool[] _free_gps
    (* 1    (length _free_gps)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int8 _state_code
       (write-byte _state_code s)
     ;; string _state_name
       (write-long (length _state_name) s) (princ _state_name s)
     ;; int8 _status_code
       (write-byte _status_code s)
     ;; string _status_name
       (write-long (length _status_name) s) (princ _status_name s)
     ;; string[] _patients
     (write-long (length _patients) s)
     (dolist (elem _patients)
       (write-long (length elem) s) (princ elem s)
       )
     ;; bool[] _free_gps
     (write-long (length _free_gps) s)
     (dotimes (i (length _free_gps))
       (if (elt _free_gps i) (write-byte -1 s) (write-byte 0 s))
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int8 _state_code
     (setq _state_code (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _state_code 127) (setq _state_code (- _state_code 256)))
   ;; string _state_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _status_code
     (setq _status_code (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _status_code 127) (setq _status_code (- _status_code 256)))
   ;; string _status_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string[] _patients
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _patients (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _patients i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; bool[] _free_gps
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _free_gps (make-list n))
     (dotimes (i n)
     (setf (elt _free_gps i) (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
     ))
   ;;
   self)
  )

(setf (get lodestone::State :md5sum-) "4e3672c940da58e350b2feda44bddd06")
(setf (get lodestone::State :datatype-) "lodestone/State")
(setf (get lodestone::State :definition-)
      "Header header

int8 state_code
string state_name

int8 status_code
string status_name

string[] patients
bool[] free_gps
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



(provide :lodestone/State "4e3672c940da58e350b2feda44bddd06")


