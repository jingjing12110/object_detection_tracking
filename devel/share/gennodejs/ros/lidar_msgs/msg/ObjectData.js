// Auto-generated. Do not edit!

// (in-package lidar_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Object = require('./Object.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ObjectData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.bbox = null;
      this.token = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('bbox')) {
        this.bbox = initObj.bbox
      }
      else {
        this.bbox = [];
      }
      if (initObj.hasOwnProperty('token')) {
        this.token = initObj.token
      }
      else {
        this.token = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObjectData
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [bbox]
    // Serialize the length for message field [bbox]
    bufferOffset = _serializer.uint32(obj.bbox.length, buffer, bufferOffset);
    obj.bbox.forEach((val) => {
      bufferOffset = Object.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [token]
    bufferOffset = _serializer.string(obj.token, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObjectData
    let len;
    let data = new ObjectData(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [bbox]
    // Deserialize array length for message field [bbox]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bbox = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bbox[i] = Object.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [token]
    data.token = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.bbox.forEach((val) => {
      length += Object.getMessageSize(val);
    });
    length += object.token.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'lidar_msgs/ObjectData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cce2fe89002b3c1acf48988217c33108';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    Object[] bbox
    string token
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: lidar_msgs/Object
    
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
    const resolved = new ObjectData(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.bbox !== undefined) {
      resolved.bbox = new Array(msg.bbox.length);
      for (let i = 0; i < resolved.bbox.length; ++i) {
        resolved.bbox[i] = Object.Resolve(msg.bbox[i]);
      }
    }
    else {
      resolved.bbox = []
    }

    if (msg.token !== undefined) {
      resolved.token = msg.token;
    }
    else {
      resolved.token = ''
    }

    return resolved;
    }
};

module.exports = ObjectData;
