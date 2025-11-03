// Auto-generated. Do not edit!

// (in-package key_input_pkg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class KeyInput {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.keys = null;
    }
    else {
      if (initObj.hasOwnProperty('keys')) {
        this.keys = initObj.keys
      }
      else {
        this.keys = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KeyInput
    // Serialize message field [keys]
    bufferOffset = _arraySerializer.int32(obj.keys, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KeyInput
    let len;
    let data = new KeyInput(null);
    // Deserialize message field [keys]
    data.keys = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.keys.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'key_input_pkg/KeyInput';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '7baccca94a87d2d70852f2dc60f54b65';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] keys
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KeyInput(null);
    if (msg.keys !== undefined) {
      resolved.keys = msg.keys;
    }
    else {
      resolved.keys = []
    }

    return resolved;
    }
};

module.exports = KeyInput;
