// Auto-generated. Do not edit!

// (in-package robot_move.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class moveFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.n_x = null;
      this.n_y = null;
      this.n_z = null;
    }
    else {
      if (initObj.hasOwnProperty('n_x')) {
        this.n_x = initObj.n_x
      }
      else {
        this.n_x = 0.0;
      }
      if (initObj.hasOwnProperty('n_y')) {
        this.n_y = initObj.n_y
      }
      else {
        this.n_y = 0.0;
      }
      if (initObj.hasOwnProperty('n_z')) {
        this.n_z = initObj.n_z
      }
      else {
        this.n_z = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type moveFeedback
    // Serialize message field [n_x]
    bufferOffset = _serializer.float32(obj.n_x, buffer, bufferOffset);
    // Serialize message field [n_y]
    bufferOffset = _serializer.float32(obj.n_y, buffer, bufferOffset);
    // Serialize message field [n_z]
    bufferOffset = _serializer.float32(obj.n_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type moveFeedback
    let len;
    let data = new moveFeedback(null);
    // Deserialize message field [n_x]
    data.n_x = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [n_y]
    data.n_y = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [n_z]
    data.n_z = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'robot_move/moveFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd56e8bdd8926891cb4120395ff64a44b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # Define a feedback message
    float32 n_x
    float32 n_y
    float32 n_z
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new moveFeedback(null);
    if (msg.n_x !== undefined) {
      resolved.n_x = msg.n_x;
    }
    else {
      resolved.n_x = 0.0
    }

    if (msg.n_y !== undefined) {
      resolved.n_y = msg.n_y;
    }
    else {
      resolved.n_y = 0.0
    }

    if (msg.n_z !== undefined) {
      resolved.n_z = msg.n_z;
    }
    else {
      resolved.n_z = 0.0
    }

    return resolved;
    }
};

module.exports = moveFeedback;