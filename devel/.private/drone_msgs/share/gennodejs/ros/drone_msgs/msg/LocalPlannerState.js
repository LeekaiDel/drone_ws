// Auto-generated. Do not edit!

// (in-package drone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class LocalPlannerState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.state = null;
      this.max_dist = null;
      this.k = null;
    }
    else {
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = false;
      }
      if (initObj.hasOwnProperty('max_dist')) {
        this.max_dist = initObj.max_dist
      }
      else {
        this.max_dist = 0.0;
      }
      if (initObj.hasOwnProperty('k')) {
        this.k = initObj.k
      }
      else {
        this.k = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LocalPlannerState
    // Serialize message field [state]
    bufferOffset = _serializer.bool(obj.state, buffer, bufferOffset);
    // Serialize message field [max_dist]
    bufferOffset = _serializer.float32(obj.max_dist, buffer, bufferOffset);
    // Serialize message field [k]
    bufferOffset = _serializer.float32(obj.k, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LocalPlannerState
    let len;
    let data = new LocalPlannerState(null);
    // Deserialize message field [state]
    data.state = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [max_dist]
    data.max_dist = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [k]
    data.k = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/LocalPlannerState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '152bbd8ec6848572b28cbf06443e0c97';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool state
    float32 max_dist
    float32 k
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new LocalPlannerState(null);
    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = false
    }

    if (msg.max_dist !== undefined) {
      resolved.max_dist = msg.max_dist;
    }
    else {
      resolved.max_dist = 0.0
    }

    if (msg.k !== undefined) {
      resolved.k = msg.k;
    }
    else {
      resolved.k = 0.0
    }

    return resolved;
    }
};

module.exports = LocalPlannerState;
