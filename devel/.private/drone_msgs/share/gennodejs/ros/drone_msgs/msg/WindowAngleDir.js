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

class WindowAngleDir {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.found_window = null;
      this.width_angle = null;
      this.height_angle = null;
    }
    else {
      if (initObj.hasOwnProperty('found_window')) {
        this.found_window = initObj.found_window
      }
      else {
        this.found_window = false;
      }
      if (initObj.hasOwnProperty('width_angle')) {
        this.width_angle = initObj.width_angle
      }
      else {
        this.width_angle = 0.0;
      }
      if (initObj.hasOwnProperty('height_angle')) {
        this.height_angle = initObj.height_angle
      }
      else {
        this.height_angle = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WindowAngleDir
    // Serialize message field [found_window]
    bufferOffset = _serializer.bool(obj.found_window, buffer, bufferOffset);
    // Serialize message field [width_angle]
    bufferOffset = _serializer.float32(obj.width_angle, buffer, bufferOffset);
    // Serialize message field [height_angle]
    bufferOffset = _serializer.float32(obj.height_angle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WindowAngleDir
    let len;
    let data = new WindowAngleDir(null);
    // Deserialize message field [found_window]
    data.found_window = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [width_angle]
    data.width_angle = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [height_angle]
    data.height_angle = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/WindowAngleDir';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e706742e1d1f28d3b56545716318772f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool found_window
    float32 width_angle
    float32 height_angle
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WindowAngleDir(null);
    if (msg.found_window !== undefined) {
      resolved.found_window = msg.found_window;
    }
    else {
      resolved.found_window = false
    }

    if (msg.width_angle !== undefined) {
      resolved.width_angle = msg.width_angle;
    }
    else {
      resolved.width_angle = 0.0
    }

    if (msg.height_angle !== undefined) {
      resolved.height_angle = msg.height_angle;
    }
    else {
      resolved.height_angle = 0.0
    }

    return resolved;
    }
};

module.exports = WindowAngleDir;
