// Auto-generated. Do not edit!

// (in-package ouster_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PacketMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.buf = null;
    }
    else {
      if (initObj.hasOwnProperty('buf')) {
        this.buf = initObj.buf
      }
      else {
        this.buf = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PacketMsg
    // Serialize message field [buf]
    bufferOffset = _arraySerializer.uint8(obj.buf, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PacketMsg
    let len;
    let data = new PacketMsg(null);
    // Deserialize message field [buf]
    data.buf = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.buf.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'ouster_ros/PacketMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f7b5949e76f86d01e96b0e33ba9b5e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8[] buf
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PacketMsg(null);
    if (msg.buf !== undefined) {
      resolved.buf = msg.buf;
    }
    else {
      resolved.buf = []
    }

    return resolved;
    }
};

module.exports = PacketMsg;
