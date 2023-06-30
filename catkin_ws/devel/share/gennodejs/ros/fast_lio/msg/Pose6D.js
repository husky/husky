// Auto-generated. Do not edit!

// (in-package fast_lio.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Pose6D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.offset_time = null;
      this.acc = null;
      this.gyr = null;
      this.vel = null;
      this.pos = null;
      this.rot = null;
    }
    else {
      if (initObj.hasOwnProperty('offset_time')) {
        this.offset_time = initObj.offset_time
      }
      else {
        this.offset_time = 0.0;
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('gyr')) {
        this.gyr = initObj.gyr
      }
      else {
        this.gyr = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('pos')) {
        this.pos = initObj.pos
      }
      else {
        this.pos = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('rot')) {
        this.rot = initObj.rot
      }
      else {
        this.rot = new Array(9).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Pose6D
    // Serialize message field [offset_time]
    bufferOffset = _serializer.float64(obj.offset_time, buffer, bufferOffset);
    // Check that the constant length array field [acc] has the right length
    if (obj.acc.length !== 3) {
      throw new Error('Unable to serialize array field acc - length must be 3')
    }
    // Serialize message field [acc]
    bufferOffset = _arraySerializer.float64(obj.acc, buffer, bufferOffset, 3);
    // Check that the constant length array field [gyr] has the right length
    if (obj.gyr.length !== 3) {
      throw new Error('Unable to serialize array field gyr - length must be 3')
    }
    // Serialize message field [gyr]
    bufferOffset = _arraySerializer.float64(obj.gyr, buffer, bufferOffset, 3);
    // Check that the constant length array field [vel] has the right length
    if (obj.vel.length !== 3) {
      throw new Error('Unable to serialize array field vel - length must be 3')
    }
    // Serialize message field [vel]
    bufferOffset = _arraySerializer.float64(obj.vel, buffer, bufferOffset, 3);
    // Check that the constant length array field [pos] has the right length
    if (obj.pos.length !== 3) {
      throw new Error('Unable to serialize array field pos - length must be 3')
    }
    // Serialize message field [pos]
    bufferOffset = _arraySerializer.float64(obj.pos, buffer, bufferOffset, 3);
    // Check that the constant length array field [rot] has the right length
    if (obj.rot.length !== 9) {
      throw new Error('Unable to serialize array field rot - length must be 9')
    }
    // Serialize message field [rot]
    bufferOffset = _arraySerializer.float64(obj.rot, buffer, bufferOffset, 9);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Pose6D
    let len;
    let data = new Pose6D(null);
    // Deserialize message field [offset_time]
    data.offset_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [gyr]
    data.gyr = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [vel]
    data.vel = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [pos]
    data.pos = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [rot]
    data.rot = _arrayDeserializer.float64(buffer, bufferOffset, 9)
    return data;
  }

  static getMessageSize(object) {
    return 176;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fast_lio/Pose6D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ab486e9c24704038320abf9ff59003d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # the preintegrated Lidar states at the time of IMU measurements in a frame
    float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point
    float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin
    float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin
    float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin
    float64[3] pos       # the preintegrated position (global frame) at the Lidar origin
    float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Pose6D(null);
    if (msg.offset_time !== undefined) {
      resolved.offset_time = msg.offset_time;
    }
    else {
      resolved.offset_time = 0.0
    }

    if (msg.acc !== undefined) {
      resolved.acc = msg.acc;
    }
    else {
      resolved.acc = new Array(3).fill(0)
    }

    if (msg.gyr !== undefined) {
      resolved.gyr = msg.gyr;
    }
    else {
      resolved.gyr = new Array(3).fill(0)
    }

    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = new Array(3).fill(0)
    }

    if (msg.pos !== undefined) {
      resolved.pos = msg.pos;
    }
    else {
      resolved.pos = new Array(3).fill(0)
    }

    if (msg.rot !== undefined) {
      resolved.rot = msg.rot;
    }
    else {
      resolved.rot = new Array(9).fill(0)
    }

    return resolved;
    }
};

module.exports = Pose6D;
