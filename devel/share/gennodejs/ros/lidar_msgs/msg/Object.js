// Auto-generated. Do not edit!

// (in-package lidar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Point3 = require('./Point3.js');
let Point2 = require('./Point2.js');

//-----------------------------------------------------------

class Object {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.center = null;
      this.size = null;
      this.velocity = null;
      this.corners = null;
      this.contours = null;
      this.id = null;
      this.predict_covariance = null;
    }
    else {
      if (initObj.hasOwnProperty('center')) {
        this.center = initObj.center
      }
      else {
        this.center = new Point3();
      }
      if (initObj.hasOwnProperty('size')) {
        this.size = initObj.size
      }
      else {
        this.size = new Point3();
      }
      if (initObj.hasOwnProperty('velocity')) {
        this.velocity = initObj.velocity
      }
      else {
        this.velocity = new Point2();
      }
      if (initObj.hasOwnProperty('corners')) {
        this.corners = initObj.corners
      }
      else {
        this.corners = [];
      }
      if (initObj.hasOwnProperty('contours')) {
        this.contours = initObj.contours
      }
      else {
        this.contours = [];
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('predict_covariance')) {
        this.predict_covariance = initObj.predict_covariance
      }
      else {
        this.predict_covariance = new Array(36).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Object
    // Serialize message field [center]
    bufferOffset = Point3.serialize(obj.center, buffer, bufferOffset);
    // Serialize message field [size]
    bufferOffset = Point3.serialize(obj.size, buffer, bufferOffset);
    // Serialize message field [velocity]
    bufferOffset = Point2.serialize(obj.velocity, buffer, bufferOffset);
    // Serialize message field [corners]
    // Serialize the length for message field [corners]
    bufferOffset = _serializer.uint32(obj.corners.length, buffer, bufferOffset);
    obj.corners.forEach((val) => {
      bufferOffset = Point2.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [contours]
    // Serialize the length for message field [contours]
    bufferOffset = _serializer.uint32(obj.contours.length, buffer, bufferOffset);
    obj.contours.forEach((val) => {
      bufferOffset = Point2.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [id]
    bufferOffset = _serializer.int16(obj.id, buffer, bufferOffset);
    // Check that the constant length array field [predict_covariance] has the right length
    if (obj.predict_covariance.length !== 36) {
      throw new Error('Unable to serialize array field predict_covariance - length must be 36')
    }
    // Serialize message field [predict_covariance]
    bufferOffset = _arraySerializer.float64(obj.predict_covariance, buffer, bufferOffset, 36);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Object
    let len;
    let data = new Object(null);
    // Deserialize message field [center]
    data.center = Point3.deserialize(buffer, bufferOffset);
    // Deserialize message field [size]
    data.size = Point3.deserialize(buffer, bufferOffset);
    // Deserialize message field [velocity]
    data.velocity = Point2.deserialize(buffer, bufferOffset);
    // Deserialize message field [corners]
    // Deserialize array length for message field [corners]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.corners = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.corners[i] = Point2.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [contours]
    // Deserialize array length for message field [contours]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.contours = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.contours[i] = Point2.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [id]
    data.id = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [predict_covariance]
    data.predict_covariance = _arrayDeserializer.float64(buffer, bufferOffset, 36)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.corners.length;
    length += 8 * object.contours.length;
    return length + 330;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_msgs/Object';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'be0179af1f0ec1bf6324c4d7897b9aac';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Point3 center
    Point3 size
    Point2 velocity
    Point2[] corners
    Point2[] contours
    int16 id
    
    float64[36] predict_covariance
    
    
    
    ================================================================================
    MSG: lidar_msgs/Point3
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: lidar_msgs/Point2
    float32 x
    float32 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Object(null);
    if (msg.center !== undefined) {
      resolved.center = Point3.Resolve(msg.center)
    }
    else {
      resolved.center = new Point3()
    }

    if (msg.size !== undefined) {
      resolved.size = Point3.Resolve(msg.size)
    }
    else {
      resolved.size = new Point3()
    }

    if (msg.velocity !== undefined) {
      resolved.velocity = Point2.Resolve(msg.velocity)
    }
    else {
      resolved.velocity = new Point2()
    }

    if (msg.corners !== undefined) {
      resolved.corners = new Array(msg.corners.length);
      for (let i = 0; i < resolved.corners.length; ++i) {
        resolved.corners[i] = Point2.Resolve(msg.corners[i]);
      }
    }
    else {
      resolved.corners = []
    }

    if (msg.contours !== undefined) {
      resolved.contours = new Array(msg.contours.length);
      for (let i = 0; i < resolved.contours.length; ++i) {
        resolved.contours[i] = Point2.Resolve(msg.contours[i]);
      }
    }
    else {
      resolved.contours = []
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.predict_covariance !== undefined) {
      resolved.predict_covariance = msg.predict_covariance;
    }
    else {
      resolved.predict_covariance = new Array(36).fill(0)
    }

    return resolved;
    }
};

module.exports = Object;
