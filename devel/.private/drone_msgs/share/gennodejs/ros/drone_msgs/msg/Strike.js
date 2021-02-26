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

class Strike {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id_drone = null;
      this.team_num = null;
      this.shot = null;
    }
    else {
      if (initObj.hasOwnProperty('id_drone')) {
        this.id_drone = initObj.id_drone
      }
      else {
        this.id_drone = 0;
      }
      if (initObj.hasOwnProperty('team_num')) {
        this.team_num = initObj.team_num
      }
      else {
        this.team_num = 0;
      }
      if (initObj.hasOwnProperty('shot')) {
        this.shot = initObj.shot
      }
      else {
        this.shot = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Strike
    // Serialize message field [id_drone]
    bufferOffset = _serializer.int8(obj.id_drone, buffer, bufferOffset);
    // Serialize message field [team_num]
    bufferOffset = _serializer.int8(obj.team_num, buffer, bufferOffset);
    // Serialize message field [shot]
    bufferOffset = _serializer.float32(obj.shot, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Strike
    let len;
    let data = new Strike(null);
    // Deserialize message field [id_drone]
    data.id_drone = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [team_num]
    data.team_num = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [shot]
    data.shot = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 6;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/Strike';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc5465527f2e02efe558071ee95658cf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 id_drone   # the id of the drone that shoots (0,1..n)
    int8 team_num   # the number of team of the drone that shoots (0,1..n)
    float32 shot  # the force of the shot
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Strike(null);
    if (msg.id_drone !== undefined) {
      resolved.id_drone = msg.id_drone;
    }
    else {
      resolved.id_drone = 0
    }

    if (msg.team_num !== undefined) {
      resolved.team_num = msg.team_num;
    }
    else {
      resolved.team_num = 0
    }

    if (msg.shot !== undefined) {
      resolved.shot = msg.shot;
    }
    else {
      resolved.shot = 0.0
    }

    return resolved;
    }
};

module.exports = Strike;
