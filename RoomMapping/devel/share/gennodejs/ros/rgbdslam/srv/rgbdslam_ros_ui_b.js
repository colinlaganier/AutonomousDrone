// Auto-generated. Do not edit!

// (in-package rgbdslam.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class rgbdslam_ros_ui_bRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
      this.value = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = '';
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rgbdslam_ros_ui_bRequest
    // Serialize message field [command]
    bufferOffset = _serializer.string(obj.command, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.bool(obj.value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rgbdslam_ros_ui_bRequest
    let len;
    let data = new rgbdslam_ros_ui_bRequest(null);
    // Deserialize message field [command]
    data.command = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.command.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rgbdslam/rgbdslam_ros_ui_bRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95aa0151a35e3de365041ffa089ce8c7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string command
    bool value
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new rgbdslam_ros_ui_bRequest(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = ''
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = false
    }

    return resolved;
    }
};

class rgbdslam_ros_ui_bResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type rgbdslam_ros_ui_bResponse
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type rgbdslam_ros_ui_bResponse
    let len;
    let data = new rgbdslam_ros_ui_bResponse(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'rgbdslam/rgbdslam_ros_ui_bResponse';
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
    const resolved = new rgbdslam_ros_ui_bResponse(null);
    return resolved;
    }
};

module.exports = {
  Request: rgbdslam_ros_ui_bRequest,
  Response: rgbdslam_ros_ui_bResponse,
  md5sum() { return '95aa0151a35e3de365041ffa089ce8c7'; },
  datatype() { return 'rgbdslam/rgbdslam_ros_ui_b'; }
};
