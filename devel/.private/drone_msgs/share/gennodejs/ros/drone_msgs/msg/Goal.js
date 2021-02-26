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

class Goal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ctr_type = null;
      this.pose = null;
    }
    else {
      if (initObj.hasOwnProperty('ctr_type')) {
        this.ctr_type = initObj.ctr_type
      }
      else {
        this.ctr_type = 0;
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
    // Serializes a message object of type Goal
    // Serialize message field [ctr_type]
    bufferOffset = _serializer.byte(obj.ctr_type, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = DronePose.serialize(obj.pose, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Goal
    let len;
    let data = new Goal(null);
    // Deserialize message field [ctr_type]
    data.ctr_type = _deserializer.byte(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = DronePose.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 29;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/Goal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf6e29cec64ab1c71dda19cb2e5b5f60';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    byte POSE=0
    byte VEL=1
    
    byte ctr_type
    drone_msgs/DronePose pose
    
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
    const resolved = new Goal(null);
    if (msg.ctr_type !== undefined) {
      resolved.ctr_type = msg.ctr_type;
    }
    else {
      resolved.ctr_type = 0
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

// Constants for message
Goal.Constants = {
  POSE: 0,
  VEL: 1,
}

module.exports = Goal;
