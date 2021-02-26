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

class DroneInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.team_num = null;
      this.id_drone = null;
      this.id_marker = null;
      this.health = null;
      this.ip = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('team_num')) {
        this.team_num = initObj.team_num
      }
      else {
        this.team_num = 0;
      }
      if (initObj.hasOwnProperty('id_drone')) {
        this.id_drone = initObj.id_drone
      }
      else {
        this.id_drone = 0;
      }
      if (initObj.hasOwnProperty('id_marker')) {
        this.id_marker = initObj.id_marker
      }
      else {
        this.id_marker = 0;
      }
      if (initObj.hasOwnProperty('health')) {
        this.health = initObj.health
      }
      else {
        this.health = 0.0;
      }
      if (initObj.hasOwnProperty('ip')) {
        this.ip = initObj.ip
      }
      else {
        this.ip = '';
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new DronePose();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DroneInfo
    // Serialize message field [team_num]
    bufferOffset = _serializer.int8(obj.team_num, buffer, bufferOffset);
    // Serialize message field [id_drone]
    bufferOffset = _serializer.int8(obj.id_drone, buffer, bufferOffset);
    // Serialize message field [id_marker]
    bufferOffset = _serializer.int8(obj.id_marker, buffer, bufferOffset);
    // Serialize message field [health]
    bufferOffset = _serializer.float32(obj.health, buffer, bufferOffset);
    // Serialize message field [ip]
    bufferOffset = _serializer.string(obj.ip, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = DronePose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DroneInfo
    let len;
    let data = new DroneInfo(null);
    // Deserialize message field [team_num]
    data.team_num = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [id_drone]
    data.id_drone = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [id_marker]
    data.id_marker = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [health]
    data.health = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ip]
    data.ip = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = DronePose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.ip.length;
    return length + 39;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/DroneInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '142d303be68c1b2c6c6b79486ae3db7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new DroneInfo(null);
    if (msg.team_num !== undefined) {
      resolved.team_num = msg.team_num;
    }
    else {
      resolved.team_num = 0
    }

    if (msg.id_drone !== undefined) {
      resolved.id_drone = msg.id_drone;
    }
    else {
      resolved.id_drone = 0
    }

    if (msg.id_marker !== undefined) {
      resolved.id_marker = msg.id_marker;
    }
    else {
      resolved.id_marker = 0
    }

    if (msg.health !== undefined) {
      resolved.health = msg.health;
    }
    else {
      resolved.health = 0.0
    }

    if (msg.ip !== undefined) {
      resolved.ip = msg.ip;
    }
    else {
      resolved.ip = ''
    }

    if (msg.pose !== undefined) {
      resolved.pose = DronePose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new DronePose()
    }

    return resolved;
    }
};

module.exports = DroneInfo;
