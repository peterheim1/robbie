; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude SetJoints-request.msg.html

(defclass <SetJoints-request> (ros-message)
  ((joints
    :reader joints-val
    :initarg :joints
    :type sensor_msgs-msg:<JointState>
    :initform (make-instance 'sensor_msgs-msg:<JointState>)))
)
(defmethod serialize ((msg <SetJoints-request>) ostream)
  "Serializes a message object of type '<SetJoints-request>"
  (serialize (slot-value msg 'joints) ostream)
)
(defmethod deserialize ((msg <SetJoints-request>) istream)
  "Deserializes a message object of type '<SetJoints-request>"
  (deserialize (slot-value msg 'joints) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<SetJoints-request>)))
  "Returns string type for a service object of type '<SetJoints-request>"
  "arbotix/SetJointsRequest")
(defmethod md5sum ((type (eql '<SetJoints-request>)))
  "Returns md5sum for a message object of type '<SetJoints-request>"
  "13b568889983e6c4080c58d8e7c2c89c")
(defmethod message-definition ((type (eql '<SetJoints-request>)))
  "Returns full string definition for message of type '<SetJoints-request>"
  (format nil "sensor_msgs/JointState joints~%~%"))
(defmethod serialization-length ((msg <SetJoints-request>))
  (+ 0
     (serialization-length (slot-value msg 'joints))
))
(defmethod ros-message-to-list ((msg <SetJoints-request>))
  "Converts a ROS message object to a list"
  (list '<SetJoints-request>
    (cons ':joints (joints-val msg))
))
;//! \htmlinclude SetJoints-response.msg.html

(defclass <SetJoints-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetJoints-response>) ostream)
  "Serializes a message object of type '<SetJoints-response>"
)
(defmethod deserialize ((msg <SetJoints-response>) istream)
  "Deserializes a message object of type '<SetJoints-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetJoints-response>)))
  "Returns string type for a service object of type '<SetJoints-response>"
  "arbotix/SetJointsResponse")
(defmethod md5sum ((type (eql '<SetJoints-response>)))
  "Returns md5sum for a message object of type '<SetJoints-response>"
  "13b568889983e6c4080c58d8e7c2c89c")
(defmethod message-definition ((type (eql '<SetJoints-response>)))
  "Returns full string definition for message of type '<SetJoints-response>"
  (format nil "~%~%================================================================================~%MSG: sensor_msgs/JointState~%# This is a message that holds data to describe the state of a set of torque controlled joints. ~%#~%# The state of each joint is defined by:~%#  * the position of the joint, ~%#  * the velocity of the joint and ~%#  * the effort that is applied in the joint.~%#~%# Each joint is uniquely identified by its name~%# The header specifies the time at which the joint states were recorded. All the joint states~%# in one message have to be recorded at the same time.~%#~%# This message consists of a multiple arrays, one for each part of the joint state. ~%# The goal is to make each of the fields optional. When e.g. your joints have no~%# effort associated with them, you can leave the effort array empty. ~%#~%# All arrays in this message should have the same size, or be empty.~%# This is the only way to uniquely associate the joint name with the correct~%# states.~%~%~%Header header~%~%string[] name~%float64[] position~%float64[] velocity~%float64[] effort~%~%================================================================================~%MSG: roslib/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(defmethod serialization-length ((msg <SetJoints-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetJoints-response>))
  "Converts a ROS message object to a list"
  (list '<SetJoints-response>
))
(defmethod service-request-type ((msg (eql 'SetJoints)))
  '<SetJoints-request>)
(defmethod service-response-type ((msg (eql 'SetJoints)))
  '<SetJoints-response>)
(defmethod ros-datatype ((msg (eql 'SetJoints)))
  "Returns string type for a service object of type '<SetJoints>"
  "arbotix/SetJoints")
