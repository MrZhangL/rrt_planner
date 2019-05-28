// Auto-generated. Do not edit!

// (in-package costmap_2d.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class DstarPoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DstarPoint
    // Serialize message field [index]
    bufferOffset = _arraySerializer.int32(obj.index, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DstarPoint
    let len;
    let data = new DstarPoint(null);
    // Deserialize message field [index]
    data.index = _arrayDeserializer.int32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.index.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'costmap_2d/DstarPoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'df62f201f3c7cdf0f85b4ec3adb4c299';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32[] index
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DstarPoint(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = []
    }

    return resolved;
    }
};

module.exports = DstarPoint;
