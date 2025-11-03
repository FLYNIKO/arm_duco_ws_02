// Auto-generated. Do not edit!

// (in-package duco_control_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class ObstacleFlags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.left_front = null;
      this.left_mid = null;
      this.left_rear = null;
      this.right_front = null;
      this.right_mid = null;
      this.right_rear = null;
      this.center = null;
      this.up = null;
      this.down = null;
      this.stamp = null;
      this.safe_distance = null;
    }
    else {
      if (initObj.hasOwnProperty('left_front')) {
        this.left_front = initObj.left_front
      }
      else {
        this.left_front = false;
      }
      if (initObj.hasOwnProperty('left_mid')) {
        this.left_mid = initObj.left_mid
      }
      else {
        this.left_mid = false;
      }
      if (initObj.hasOwnProperty('left_rear')) {
        this.left_rear = initObj.left_rear
      }
      else {
        this.left_rear = false;
      }
      if (initObj.hasOwnProperty('right_front')) {
        this.right_front = initObj.right_front
      }
      else {
        this.right_front = false;
      }
      if (initObj.hasOwnProperty('right_mid')) {
        this.right_mid = initObj.right_mid
      }
      else {
        this.right_mid = false;
      }
      if (initObj.hasOwnProperty('right_rear')) {
        this.right_rear = initObj.right_rear
      }
      else {
        this.right_rear = false;
      }
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = false;
      }
      if (initObj.hasOwnProperty('up')) {
        this.up = initObj.up
      }
      else {
        this.up = false;
      }
      if (initObj.hasOwnProperty('down')) {
        this.down = initObj.down
      }
      else {
        this.down = false;
      }
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('safe_distance')) {
        this.safe_distance = initObj.safe_distance
      }
      else {
        this.safe_distance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleFlags
    // Serialize message field [left_front]
    bufferOffset = _serializer.bool(obj.left_front, buffer, bufferOffset);
    // Serialize message field [left_mid]
    bufferOffset = _serializer.bool(obj.left_mid, buffer, bufferOffset);
    // Serialize message field [left_rear]
    bufferOffset = _serializer.bool(obj.left_rear, buffer, bufferOffset);
    // Serialize message field [right_front]
    bufferOffset = _serializer.bool(obj.right_front, buffer, bufferOffset);
    // Serialize message field [right_mid]
    bufferOffset = _serializer.bool(obj.right_mid, buffer, bufferOffset);
    // Serialize message field [right_rear]
    bufferOffset = _serializer.bool(obj.right_rear, buffer, bufferOffset);
    // Serialize message field [center]
    bufferOffset = _serializer.bool(obj.center, buffer, bufferOffset);
    // Serialize message field [up]
    bufferOffset = _serializer.bool(obj.up, buffer, bufferOffset);
    // Serialize message field [down]
    bufferOffset = _serializer.bool(obj.down, buffer, bufferOffset);
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [safe_distance]
    bufferOffset = _serializer.float32(obj.safe_distance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleFlags
    let len;
    let data = new ObstacleFlags(null);
    // Deserialize message field [left_front]
    data.left_front = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_mid]
    data.left_mid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [left_rear]
    data.left_rear = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_front]
    data.right_front = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_mid]
    data.right_mid = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [right_rear]
    data.right_rear = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [center]
    data.center = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [up]
    data.up = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [down]
    data.down = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [safe_distance]
    data.safe_distance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'duco_control_pkg/ObstacleFlags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd0c938d9c5630fb2581830805df06b63';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ObstacleFlags.msg
    # 自定义消息类型，用于发布各区域的避障标志
    
    # 各区域避障标志
    bool left_front    # 左前区有障碍
    bool left_mid      # 左中区有障碍
    bool left_rear     # 左后区有障碍
    bool right_front   # 右前区有障碍  
    bool right_mid     # 右中区有障碍
    bool right_rear    # 右后区有障碍
    bool center        # 中区有障碍
    bool up            # 上区有障碍
    bool down          # 下区有障碍
    # 可选：添加时间戳和安全距离信息
    time stamp              # 检测时间戳
    float32 safe_distance   # 使用的安全距离
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObstacleFlags(null);
    if (msg.left_front !== undefined) {
      resolved.left_front = msg.left_front;
    }
    else {
      resolved.left_front = false
    }

    if (msg.left_mid !== undefined) {
      resolved.left_mid = msg.left_mid;
    }
    else {
      resolved.left_mid = false
    }

    if (msg.left_rear !== undefined) {
      resolved.left_rear = msg.left_rear;
    }
    else {
      resolved.left_rear = false
    }

    if (msg.right_front !== undefined) {
      resolved.right_front = msg.right_front;
    }
    else {
      resolved.right_front = false
    }

    if (msg.right_mid !== undefined) {
      resolved.right_mid = msg.right_mid;
    }
    else {
      resolved.right_mid = false
    }

    if (msg.right_rear !== undefined) {
      resolved.right_rear = msg.right_rear;
    }
    else {
      resolved.right_rear = false
    }

    if (msg.center !== undefined) {
      resolved.center = msg.center;
    }
    else {
      resolved.center = false
    }

    if (msg.up !== undefined) {
      resolved.up = msg.up;
    }
    else {
      resolved.up = false
    }

    if (msg.down !== undefined) {
      resolved.down = msg.down;
    }
    else {
      resolved.down = false
    }

    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.safe_distance !== undefined) {
      resolved.safe_distance = msg.safe_distance;
    }
    else {
      resolved.safe_distance = 0.0
    }

    return resolved;
    }
};

module.exports = ObstacleFlags;
