; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude SetAngle-request.msg.html

(defclass <SetAngle-request> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <SetAngle-request>) ostream)
  "Serializes a message object of type '<SetAngle-request>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'angle))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <SetAngle-request>) istream)
  "Deserializes a message object of type '<SetAngle-request>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<SetAngle-request>)))
  "Returns string type for a service object of type '<SetAngle-request>"
  "arbotix/SetAngleRequest")
(defmethod md5sum ((type (eql '<SetAngle-request>)))
  "Returns md5sum for a message object of type '<SetAngle-request>"
  "4edb55038e2b888976a0c0c56935341c")
(defmethod message-definition ((type (eql '<SetAngle-request>)))
  "Returns full string definition for message of type '<SetAngle-request>"
  (format nil "float64 angle~%~%"))
(defmethod serialization-length ((msg <SetAngle-request>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <SetAngle-request>))
  "Converts a ROS message object to a list"
  (list '<SetAngle-request>
    (cons ':angle (angle-val msg))
))
;//! \htmlinclude SetAngle-response.msg.html

(defclass <SetAngle-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetAngle-response>) ostream)
  "Serializes a message object of type '<SetAngle-response>"
)
(defmethod deserialize ((msg <SetAngle-response>) istream)
  "Deserializes a message object of type '<SetAngle-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetAngle-response>)))
  "Returns string type for a service object of type '<SetAngle-response>"
  "arbotix/SetAngleResponse")
(defmethod md5sum ((type (eql '<SetAngle-response>)))
  "Returns md5sum for a message object of type '<SetAngle-response>"
  "4edb55038e2b888976a0c0c56935341c")
(defmethod message-definition ((type (eql '<SetAngle-response>)))
  "Returns full string definition for message of type '<SetAngle-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <SetAngle-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetAngle-response>))
  "Converts a ROS message object to a list"
  (list '<SetAngle-response>
))
(defmethod service-request-type ((msg (eql 'SetAngle)))
  '<SetAngle-request>)
(defmethod service-response-type ((msg (eql 'SetAngle)))
  '<SetAngle-response>)
(defmethod ros-datatype ((msg (eql 'SetAngle)))
  "Returns string type for a service object of type '<SetAngle>"
  "arbotix/SetAngle")
