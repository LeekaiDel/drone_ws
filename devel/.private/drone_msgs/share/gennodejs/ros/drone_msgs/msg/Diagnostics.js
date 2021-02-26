// Auto-generated. Do not edit!

// (in-package drone_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Diagnostics {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.armed = null;
      this.init_home = null;
      this.init_origin = null;
      this.gps_send = null;
      this.status = null;
      this.mode = null;
      this.battery = null;
      this.health = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('armed')) {
        this.armed = initObj.armed
      }
      else {
        this.armed = false;
      }
      if (initObj.hasOwnProperty('init_home')) {
        this.init_home = initObj.init_home
      }
      else {
        this.init_home = false;
      }
      if (initObj.hasOwnProperty('init_origin')) {
        this.init_origin = initObj.init_origin
      }
      else {
        this.init_origin = false;
      }
      if (initObj.hasOwnProperty('gps_send')) {
        this.gps_send = initObj.gps_send
      }
      else {
        this.gps_send = false;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = new sensor_msgs.msg.NavSatStatus();
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = '';
      }
      if (initObj.hasOwnProperty('battery')) {
        this.battery = initObj.battery
      }
      else {
        this.battery = 0.0;
      }
      if (initObj.hasOwnProperty('health')) {
        this.health = initObj.health
      }
      else {
        this.health = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Diagnostics
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [armed]
    bufferOffset = _serializer.bool(obj.armed, buffer, bufferOffset);
    // Serialize message field [init_home]
    bufferOffset = _serializer.bool(obj.init_home, buffer, bufferOffset);
    // Serialize message field [init_origin]
    bufferOffset = _serializer.bool(obj.init_origin, buffer, bufferOffset);
    // Serialize message field [gps_send]
    bufferOffset = _serializer.bool(obj.gps_send, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = sensor_msgs.msg.NavSatStatus.serialize(obj.status, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.string(obj.mode, buffer, bufferOffset);
    // Serialize message field [battery]
    bufferOffset = _serializer.float32(obj.battery, buffer, bufferOffset);
    // Serialize message field [health]
    bufferOffset = _serializer.float32(obj.health, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Diagnostics
    let len;
    let data = new Diagnostics(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [armed]
    data.armed = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [init_home]
    data.init_home = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [init_origin]
    data.init_origin = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [gps_send]
    data.gps_send = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = sensor_msgs.msg.NavSatStatus.deserialize(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [battery]
    data.battery = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [health]
    data.health = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.mode.length;
    return length + 19;
  }

  static datatype() {
    // Returns string type for a message object
    return 'drone_msgs/Diagnostics';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2c87ff1e63a374108ac3dbac9530310f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    bool armed
    bool init_home
    bool init_origin
    bool gps_send
    sensor_msgs/NavSatStatus status
    
    
    string mode
    float32 battery
    
    float32 health  # health of drone (100%..0%)
    
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
    MSG: sensor_msgs/NavSatStatus
    # Navigation Satellite fix status for any Global Navigation Satellite System
    
    # Whether to output an augmented fix is determined by both the fix
    # type and the last time differential corrections were received.  A
    # fix is valid when status >= STATUS_FIX.
    
    int8 STATUS_NO_FIX =  -1        # unable to fix position
    int8 STATUS_FIX =      0        # unaugmented fix
    int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
    int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation
    
    int8 status
    
    # Bits defining which Global Navigation Satellite System signals were
    # used by the receiver.
    
    uint16 SERVICE_GPS =     1
    uint16 SERVICE_GLONASS = 2
    uint16 SERVICE_COMPASS = 4      # includes BeiDou.
    uint16 SERVICE_GALILEO = 8
    
    uint16 service
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Diagnostics(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.armed !== undefined) {
      resolved.armed = msg.armed;
    }
    else {
      resolved.armed = false
    }

    if (msg.init_home !== undefined) {
      resolved.init_home = msg.init_home;
    }
    else {
      resolved.init_home = false
    }

    if (msg.init_origin !== undefined) {
      resolved.init_origin = msg.init_origin;
    }
    else {
      resolved.init_origin = false
    }

    if (msg.gps_send !== undefined) {
      resolved.gps_send = msg.gps_send;
    }
    else {
      resolved.gps_send = false
    }

    if (msg.status !== undefined) {
      resolved.status = sensor_msgs.msg.NavSatStatus.Resolve(msg.status)
    }
    else {
      resolved.status = new sensor_msgs.msg.NavSatStatus()
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = ''
    }

    if (msg.battery !== undefined) {
      resolved.battery = msg.battery;
    }
    else {
      resolved.battery = 0.0
    }

    if (msg.health !== undefined) {
      resolved.health = msg.health;
    }
    else {
      resolved.health = 0.0
    }

    return resolved;
    }
};

module.exports = Diagnostics;
