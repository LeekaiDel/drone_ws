// Auto-generated. Do not edit!

// (in-package drone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DroneInfo = require('./DroneInfo.js');

//-----------------------------------------------------------

class DroneInfoArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drones = null;
    }
    else {
      if (initObj.hasOwnProperty('drones')) {
        this.drones = initObj.drones
      }
      else {
        this.drones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DroneInfoArray
    // Serialize message field [drones]
    // Serialize the length for message field [drones]
    bufferOffset = _serializer.uint32(obj.drones.length, buffer, bufferOffset);
    obj.drones.forEach((val) => {
      bufferOffset = DroneInfo.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DroneInfoArray
    let len;
    let data = new DroneInfoArray(null);
    // Deserialize message field [drones]
    // Deserialize array length for message field [drones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.drones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.drones[i] = DroneInfo.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.drones.forEach((val) => {
      length += DroneInfo.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/DroneInfoArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b567d9a3a60bda150b09c0bcd10bfc14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    drone_msgs/DroneInfo[] drones
    ================================================================================
    MSG: drone_msgs/DroneInfo
    int8 team_num   # number of team (0,1..n)
    int8 id_drone   # id of drone (0,1..n)
    int8 id_marker  # number of marker id
    float32 health  # health of drone (100%..0%)
    string ip
    drone_msgs/DronePose pose   # ENU position of drone
    
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
    const resolved = new DroneInfoArray(null);
    if (msg.drones !== undefined) {
      resolved.drones = new Array(msg.drones.length);
      for (let i = 0; i < resolved.drones.length; ++i) {
        resolved.drones[i] = DroneInfo.Resolve(msg.drones[i]);
      }
    }
    else {
      resolved.drones = []
    }

    return resolved;
    }
};

module.exports = DroneInfoArray;
