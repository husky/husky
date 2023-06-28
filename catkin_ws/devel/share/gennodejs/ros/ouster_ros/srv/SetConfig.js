// Auto-generated. Do not edit!

// (in-package ouster_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetConfigRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config_file = null;
    }
    else {
      if (initObj.hasOwnProperty('config_file')) {
        this.config_file = initObj.config_file
      }
      else {
        this.config_file = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetConfigRequest
    // Serialize message field [config_file]
    bufferOffset = _serializer.string(obj.config_file, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetConfigRequest
    let len;
    let data = new SetConfigRequest(null);
    // Deserialize message field [config_file]
    data.config_file = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.config_file);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/SetConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90949894c75d4db440cc7a08c4bf47dd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string config_file
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetConfigRequest(null);
    if (msg.config_file !== undefined) {
      resolved.config_file = msg.config_file;
    }
    else {
      resolved.config_file = ''
    }

    return resolved;
    }
};

class SetConfigResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.config = null;
    }
    else {
      if (initObj.hasOwnProperty('config')) {
        this.config = initObj.config
      }
      else {
        this.config = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetConfigResponse
    // Serialize message field [config]
    bufferOffset = _serializer.string(obj.config, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetConfigResponse
    let len;
    let data = new SetConfigResponse(null);
    // Deserialize message field [config]
    data.config = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.config);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/SetConfigResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b3532af339db184b4a6a974d00ee4fe6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string config
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetConfigResponse(null);
    if (msg.config !== undefined) {
      resolved.config = msg.config;
    }
    else {
      resolved.config = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: SetConfigRequest,
  Response: SetConfigResponse,
  md5sum() { return '7c21f4ef724c955b8242aed00884d81a'; },
  datatype() { return 'ouster_ros/SetConfig'; }
};
