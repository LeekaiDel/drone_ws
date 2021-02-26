// Auto-generated. Do not edit!

// (in-package drone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DronePose = require('./DronePose.js');

//-----------------------------------------------------------

class WindowPointDir {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.found_window = null;
      this.point = null;
    }
    else {
      if (initObj.hasOwnProperty('found_window')) {
        this.found_window = initObj.found_window
      }
      else {
        this.found_window = false;
      }
      if (initObj.hasOwnProperty('point')) {
        this.point = initObj.point
      }
      else {
        this.point = new DronePose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type WindowPointDir
    // Serialize message field [found_window]
    bufferOffset = _serializer.bool(obj.found_window, buffer, bufferOffset);
    // Serialize message field [point]
    bufferOffset = DronePose.serialize(obj.point, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type WindowPointDir
    let len;
    let data = new WindowPointDir(null);
    // Deserialize message field [found_window]
    data.found_window = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [point]
    data.point = DronePose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/WindowPointDir';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6e775f1bde836c88e4039d51e180cd67';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
     bool found_window
     drone_msgs/DronePose point
    
    ================================================================================
    MSG: drone_msgs/DronePose
    geometry_msgs/Point point
    float32 course
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new WindowPointDir(null);
    if (msg.found_window !== undefined) {
      resolved.found_window = msg.found_window;
    }
    else {
      resolved.found_window = false
    }

    if (msg.point !== undefined) {
      resolved.point = DronePose.Resolve(msg.point)
    }
    else {
      resolved.point = new DronePose()
    }

    return resolved;
    }
};

module.exports = WindowPointDir;
