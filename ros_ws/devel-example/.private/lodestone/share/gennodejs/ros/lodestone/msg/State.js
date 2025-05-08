// Auto-generated. Do not edit!

// (in-package lodestone.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.state_code = null;
      this.state_name = null;
      this.status_code = null;
      this.status_name = null;
      this.patients = null;
      this.free_gps = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('state_code')) {
        this.state_code = initObj.state_code
      }
      else {
        this.state_code = 0;
      }
      if (initObj.hasOwnProperty('state_name')) {
        this.state_name = initObj.state_name
      }
      else {
        this.state_name = '';
      }
      if (initObj.hasOwnProperty('status_code')) {
        this.status_code = initObj.status_code
      }
      else {
        this.status_code = 0;
      }
      if (initObj.hasOwnProperty('status_name')) {
        this.status_name = initObj.status_name
      }
      else {
        this.status_name = '';
      }
      if (initObj.hasOwnProperty('patients')) {
        this.patients = initObj.patients
      }
      else {
        this.patients = [];
      }
      if (initObj.hasOwnProperty('free_gps')) {
        this.free_gps = initObj.free_gps
      }
      else {
        this.free_gps = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type State
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [state_code]
    bufferOffset = _serializer.int8(obj.state_code, buffer, bufferOffset);
    // Serialize message field [state_name]
    bufferOffset = _serializer.string(obj.state_name, buffer, bufferOffset);
    // Serialize message field [status_code]
    bufferOffset = _serializer.int8(obj.status_code, buffer, bufferOffset);
    // Serialize message field [status_name]
    bufferOffset = _serializer.string(obj.status_name, buffer, bufferOffset);
    // Serialize message field [patients]
    bufferOffset = _arraySerializer.string(obj.patients, buffer, bufferOffset, null);
    // Serialize message field [free_gps]
    bufferOffset = _arraySerializer.bool(obj.free_gps, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type State
    let len;
    let data = new State(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [state_code]
    data.state_code = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [state_name]
    data.state_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [status_code]
    data.status_code = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [status_name]
    data.status_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [patients]
    data.patients = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [free_gps]
    data.free_gps = _arrayDeserializer.bool(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.state_name);
    length += _getByteLength(object.status_name);
    object.patients.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += object.free_gps.length;
    return length + 18;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lodestone/State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4e3672c940da58e350b2feda44bddd06';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new State(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.state_code !== undefined) {
      resolved.state_code = msg.state_code;
    }
    else {
      resolved.state_code = 0
    }

    if (msg.state_name !== undefined) {
      resolved.state_name = msg.state_name;
    }
    else {
      resolved.state_name = ''
    }

    if (msg.status_code !== undefined) {
      resolved.status_code = msg.status_code;
    }
    else {
      resolved.status_code = 0
    }

    if (msg.status_name !== undefined) {
      resolved.status_name = msg.status_name;
    }
    else {
      resolved.status_name = ''
    }

    if (msg.patients !== undefined) {
      resolved.patients = msg.patients;
    }
    else {
      resolved.patients = []
    }

    if (msg.free_gps !== undefined) {
      resolved.free_gps = msg.free_gps;
    }
    else {
      resolved.free_gps = []
    }

    return resolved;
    }
};

module.exports = State;
