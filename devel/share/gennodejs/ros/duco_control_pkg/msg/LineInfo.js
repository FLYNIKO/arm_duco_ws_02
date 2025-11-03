// Auto-generated. Do not edit!

// (in-package duco_control_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class LineInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_point = null;
      this.end_point = null;
      this.length = null;
      this.angle_deg = null;
      this.distance = null;
      this.type = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('start_point')) {
        this.start_point = initObj.start_point
      }
      else {
        this.start_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('end_point')) {
        this.end_point = initObj.end_point
      }
      else {
        this.end_point = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('length')) {
        this.length = initObj.length
      }
      else {
        this.length = 0.0;
      }
      if (initObj.hasOwnProperty('angle_deg')) {
        this.angle_deg = initObj.angle_deg
      }
      else {
        this.angle_deg = 0.0;
      }
      if (initObj.hasOwnProperty('distance')) {
        this.distance = initObj.distance
      }
      else {
        this.distance = 0.0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type LineInfo
    // Serialize message field [start_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.start_point, buffer, bufferOffset);
    // Serialize message field [end_point]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.end_point, buffer, bufferOffset);
    // Serialize message field [length]
    bufferOffset = _serializer.float32(obj.length, buffer, bufferOffset);
    // Serialize message field [angle_deg]
    bufferOffset = _serializer.float32(obj.angle_deg, buffer, bufferOffset);
    // Serialize message field [distance]
    bufferOffset = _serializer.float32(obj.distance, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.int32(obj.type, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type LineInfo
    let len;
    let data = new LineInfo(null);
    // Deserialize message field [start_point]
    data.start_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_point]
    data.end_point = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [length]
    data.length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [angle_deg]
    data.angle_deg = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [distance]
    data.distance = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'duco_control_pkg/LineInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c268946f4721487ff5d43b6e4957d2b0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # LineInfo.msg
    geometry_msgs/Point start_point
    geometry_msgs/Point end_point
    float32 length
    float32 angle_deg
    float32 distance
    int32 type
    int32 id
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
    const resolved = new LineInfo(null);
    if (msg.start_point !== undefined) {
      resolved.start_point = geometry_msgs.msg.Point.Resolve(msg.start_point)
    }
    else {
      resolved.start_point = new geometry_msgs.msg.Point()
    }

    if (msg.end_point !== undefined) {
      resolved.end_point = geometry_msgs.msg.Point.Resolve(msg.end_point)
    }
    else {
      resolved.end_point = new geometry_msgs.msg.Point()
    }

    if (msg.length !== undefined) {
      resolved.length = msg.length;
    }
    else {
      resolved.length = 0.0
    }

    if (msg.angle_deg !== undefined) {
      resolved.angle_deg = msg.angle_deg;
    }
    else {
      resolved.angle_deg = 0.0
    }

    if (msg.distance !== undefined) {
      resolved.distance = msg.distance;
    }
    else {
      resolved.distance = 0.0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

module.exports = LineInfo;
