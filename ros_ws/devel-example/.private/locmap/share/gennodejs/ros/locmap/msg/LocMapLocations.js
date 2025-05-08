// Auto-generated. Do not edit!

// (in-package locmap.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let LocMapLocation = require('./LocMapLocation.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class LocMapLocations {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.locations = null;
      this.groups = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('locations')) {
        this.locations = initObj.locations
      }
      else {
        this.locations = [];
      }
      if (initObj.hasOwnProperty('groups')) {
        this.groups = initObj.groups
      }
      else {
        this.groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocMapLocations
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [locations]
    // Serialize the length for message field [locations]
    bufferOffset = _serializer.uint32(obj.locations.length, buffer, bufferOffset);
    obj.locations.forEach((val) => {
      bufferOffset = LocMapLocation.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [groups]
    bufferOffset = _arraySerializer.string(obj.groups, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocMapLocations
    let len;
    let data = new LocMapLocations(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [locations]
    // Deserialize array length for message field [locations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.locations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.locations[i] = LocMapLocation.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [groups]
    data.groups = _arrayDeserializer.string(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.locations.forEach((val) => {
      length += LocMapLocation.getMessageSize(val);
    });
    object.groups.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'locmap/LocMapLocations';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bd3045c7f0b78ca25575427c64ec7d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocMapLocations(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.locations !== undefined) {
      resolved.locations = new Array(msg.locations.length);
      for (let i = 0; i < resolved.locations.length; ++i) {
        resolved.locations[i] = LocMapLocation.Resolve(msg.locations[i]);
      }
    }
    else {
      resolved.locations = []
    }

    if (msg.groups !== undefined) {
      resolved.groups = msg.groups;
    }
    else {
      resolved.groups = []
    }

    return resolved;
    }
};

module.exports = LocMapLocations;
