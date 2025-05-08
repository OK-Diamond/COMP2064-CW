; Auto-generated. Do not edit!


(cl:in-package locmap-msg)


;//! \htmlinclude LocMapGoto.msg.html

(cl:defclass <LocMapGoto> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (location_name
    :reader location_name
    :initarg :location_name
    :type cl:string
    :initform ""))
)

(cl:defclass LocMapGoto (<LocMapGoto>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocMapGoto>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocMapGoto)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name locmap-msg:<LocMapGoto> is deprecated: use locmap-msg:LocMapGoto instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <LocMapGoto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locmap-msg:header-val is deprecated.  Use locmap-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'location_name-val :lambda-list '(m))
(cl:defmethod location_name-val ((m <LocMapGoto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader locmap-msg:location_name-val is deprecated.  Use locmap-msg:location_name instead.")
  (location_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocMapGoto>) ostream)
  "Serializes a message object of type '<LocMapGoto>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocMapGoto>) istream)
  "Deserializes a message object of type '<LocMapGoto>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocMapGoto>)))
  "Returns string type for a message object of type '<LocMapGoto>"
  "locmap/LocMapGoto")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocMapGoto)))
  "Returns string type for a message object of type 'LocMapGoto"
  "locmap/LocMapGoto")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocMapGoto>)))
  "Returns md5sum for a message object of type '<LocMapGoto>"
  "a2a536bbdbeabd2be949e6fb00d6394b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocMapGoto)))
  "Returns md5sum for a message object of type 'LocMapGoto"
  "a2a536bbdbeabd2be949e6fb00d6394b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocMapGoto>)))
  "Returns full string definition for message of type '<LocMapGoto>"
  (cl:format cl:nil "Header header~%~%string location_name~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocMapGoto)))
  "Returns full string definition for message of type 'LocMapGoto"
  (cl:format cl:nil "Header header~%~%string location_name~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocMapGoto>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'location_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocMapGoto>))
  "Converts a ROS message object to a list"
  (cl:list 'LocMapGoto
    (cl:cons ':header (header msg))
    (cl:cons ':location_name (location_name msg))
))
