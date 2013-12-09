; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude GetVoltage-request.msg.html

(defclass <GetVoltage-request> (ros-message)
  ()
)
(defmethod serialize ((msg <GetVoltage-request>) ostream)
  "Serializes a message object of type '<GetVoltage-request>"
)
(defmethod deserialize ((msg <GetVoltage-request>) istream)
  "Deserializes a message object of type '<GetVoltage-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<GetVoltage-request>)))
  "Returns string type for a service object of type '<GetVoltage-request>"
  "arbotix/GetVoltageRequest")
(defmethod md5sum ((type (eql '<GetVoltage-request>)))
  "Returns md5sum for a message object of type '<GetVoltage-request>"
  "cd1e97d74e6d797b46bc5a51e820e6ae")
(defmethod message-definition ((type (eql '<GetVoltage-request>)))
  "Returns full string definition for message of type '<GetVoltage-request>"
  (format nil "~%"))
(defmethod serialization-length ((msg <GetVoltage-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <GetVoltage-request>))
  "Converts a ROS message object to a list"
  (list '<GetVoltage-request>
))
;//! \htmlinclude GetVoltage-response.msg.html

(defclass <GetVoltage-response> (ros-message)
  ((voltage
    :reader voltage-val
    :initarg :voltage
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <GetVoltage-response>) ostream)
  "Serializes a message object of type '<GetVoltage-response>"
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'voltage))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
)
(defmethod deserialize ((msg <GetVoltage-response>) istream)
  "Deserializes a message object of type '<GetVoltage-response>"
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'voltage) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<GetVoltage-response>)))
  "Returns string type for a service object of type '<GetVoltage-response>"
  "arbotix/GetVoltageResponse")
(defmethod md5sum ((type (eql '<GetVoltage-response>)))
  "Returns md5sum for a message object of type '<GetVoltage-response>"
  "cd1e97d74e6d797b46bc5a51e820e6ae")
(defmethod message-definition ((type (eql '<GetVoltage-response>)))
  "Returns full string definition for message of type '<GetVoltage-response>"
  (format nil "float64 voltage~%~%~%"))
(defmethod serialization-length ((msg <GetVoltage-response>))
  (+ 0
     8
))
(defmethod ros-message-to-list ((msg <GetVoltage-response>))
  "Converts a ROS message object to a list"
  (list '<GetVoltage-response>
    (cons ':voltage (voltage-val msg))
))
(defmethod service-request-type ((msg (eql 'GetVoltage)))
  '<GetVoltage-request>)
(defmethod service-response-type ((msg (eql 'GetVoltage)))
  '<GetVoltage-response>)
(defmethod ros-datatype ((msg (eql 'GetVoltage)))
  "Returns string type for a service object of type '<GetVoltage>"
  "arbotix/GetVoltage")
