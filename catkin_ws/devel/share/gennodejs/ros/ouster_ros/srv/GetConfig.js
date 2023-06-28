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

class GetConfigRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetConfigRequest
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConfigRequest
    let len;
    let data = new GetConfigRequest(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'ouster_ros/GetConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd41d8cd98f00b204e9800998ecf8427e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetConfigRequest(null);
    return resolved;
    }
};

class GetConfigResponse {
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
    // Serializes a message object of type GetConfigResponse
    // Serialize message field [config]
    bufferOffset = _serializer.string(obj.config, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetConfigResponse
    let len;
    let data = new GetConfigResponse(null);
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
    return 'ouster_ros/GetConfigResponse';
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
    const resolved = new GetConfigResponse(null);
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
  Request: GetConfigRequest,
  Response: GetConfigResponse,
  md5sum() { return 'b3532af339db184b4a6a974d00ee4fe6'; },
  datatype() { return 'ouster_ros/GetConfig'; }
};
