; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude GetAngle-request.msg.html

(defclass <GetAngle-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetAngle-request>) ostream)
  "Serializes a message object of type '<GetAngle-request>"
)
(defmethod deserialize ((msg <GetAngle-request>) istream)
  "Deserializes a message object of type '<GetAngle-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetAngle-request>)))
  "Returns string type for a service object of type '<GetAngle-request>"
  "arbotix/GetAngleRequest")
(defmethod md5sum ((type (eql '<GetAngle-request>)))
  "Returns md5sum for a message object of type '<GetAngle-request>"
  "4edb55038e2b888976a0c0c56935341c")
(defmethod message-definition ((type (eql '<GetAngle-request>)))
  "Returns full string definition for message of type '<GetAngle-request>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <GetAngle-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetAngle-request>))
  "Converts a ROS message object to a list"
  (list '<GetAngle-request>
))
;//! \htmlinclude GetAngle-response.msg.html

(defclass <GetAngle-response> (ros-message)
  ((angle
    :reader angle-val
    :initarg :angle
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <GetAngle-response>) ostream)
  "Serializes a message object of type '<GetAngle-response>"
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
(defmethod deserialize ((msg <GetAngle-response>) istream)
  "Deserializes a message object of type '<GetAngle-response>"
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
(defmethod ros-datatype ((msg (eql '<GetAngle-response>)))
  "Returns string type for a service object of type '<GetAngle-response>"
  "arbotix/GetAngleResponse")
(defmethod md5sum ((type (eql '<GetAngle-response>)))
  "Returns md5sum for a message object of type '<GetAngle-response>"
  "4edb55038e2b888976a0c0c56935341c")
(defmethod message-definition ((type (eql '<GetAngle-response>)))
  "Returns full string definition for message of type '<GetAngle-response>"
  (format nil "float64 angle~%~%~%"))
(defmethod serialization-length ((msg <GetAngle-response>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <GetAngle-response>))
  "Converts a ROS message object to a list"
  (list '<GetAngle-response>
    (cons ':angle (angle-val msg))
))
(defmethod service-request-type ((msg (eql 'GetAngle)))
  '<GetAngle-request>)
(defmethod service-response-type ((msg (eql 'GetAngle)))
  '<GetAngle-response>)
(defmethod ros-datatype ((msg (eql 'GetAngle)))
  "Returns string type for a service object of type '<GetAngle>"
  "arbotix/GetAngle")
