// Auto-generated. Do not edit!

// (in-package mlmc.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PID {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.p = null;
      this.i = null;
      this.d = null;
      this.ffd0 = null;
      this.ffd1 = null;
    }
    else {
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = 0.0;
      }
      if (initObj.hasOwnProperty('i')) {
        this.i = initObj.i
      }
      else {
        this.i = 0.0;
      }
      if (initObj.hasOwnProperty('d')) {
        this.d = initObj.d
      }
      else {
        this.d = 0.0;
      }
      if (initObj.hasOwnProperty('ffd0')) {
        this.ffd0 = initObj.ffd0
      }
      else {
        this.ffd0 = 0.0;
      }
      if (initObj.hasOwnProperty('ffd1')) {
        this.ffd1 = initObj.ffd1
      }
      else {
        this.ffd1 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PID
    // Serialize message field [p]
    bufferOffset = _serializer.float32(obj.p, buffer, bufferOffset);
    // Serialize message field [i]
    bufferOffset = _serializer.float32(obj.i, buffer, bufferOffset);
    // Serialize message field [d]
    bufferOffset = _serializer.float32(obj.d, buffer, bufferOffset);
    // Serialize message field [ffd0]
    bufferOffset = _serializer.float32(obj.ffd0, buffer, bufferOffset);
    // Serialize message field [ffd1]
    bufferOffset = _serializer.float32(obj.ffd1, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PID
    let len;
    let data = new PID(null);
    // Deserialize message field [p]
    data.p = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [i]
    data.i = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [d]
    data.d = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ffd0]
    data.ffd0 = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [ffd1]
    data.ffd1 = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mlmc/PID';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a76881aad551f4c46d9753b2549d471b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 p
    float32 i
    float32 d
    float32 ffd0
    float32 ffd1
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PID(null);
    if (msg.p !== undefined) {
      resolved.p = msg.p;
    }
    else {
      resolved.p = 0.0
    }

    if (msg.i !== undefined) {
      resolved.i = msg.i;
    }
    else {
      resolved.i = 0.0
    }

    if (msg.d !== undefined) {
      resolved.d = msg.d;
    }
    else {
      resolved.d = 0.0
    }

    if (msg.ffd0 !== undefined) {
      resolved.ffd0 = msg.ffd0;
    }
    else {
      resolved.ffd0 = 0.0
    }

    if (msg.ffd1 !== undefined) {
      resolved.ffd1 = msg.ffd1;
    }
    else {
      resolved.ffd1 = 0.0
    }

    return resolved;
    }
};

module.exports = PID;
