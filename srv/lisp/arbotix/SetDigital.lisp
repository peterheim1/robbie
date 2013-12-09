; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude SetDigital-request.msg.html

(defclass <SetDigital-request> (ros-message)
  ((pin
    :reader pin-val
    :initarg :pin
    :type fixnum
    :initform 0)
   (dir
    :reader dir-val
    :initarg :dir
    :type fixnum
    :initform 0)
   (value
    :reader value-val
    :initarg :value
    :type fixnum
    :initform 0))
)
(defmethod serialize ((msg <SetDigital-request>) ostream)
  "Serializes a message object of type '<SetDigital-request>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'pin)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'dir)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'value)) ostream)
)
(defmethod deserialize ((msg <SetDigital-request>) istream)
  "Deserializes a message object of type '<SetDigital-request>"
  (setf (ldb (byte 8 0) (slot-value msg 'pin)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'dir)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'value)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<SetDigital-request>)))
  "Returns string type for a service object of type '<SetDigital-request>"
  "arbotix/SetDigitalRequest")
(defmethod md5sum ((type (eql '<SetDigital-request>)))
  "Returns md5sum for a message object of type '<SetDigital-request>"
  "76cc68f0131aad3d0f5016da5d272467")
(defmethod message-definition ((type (eql '<SetDigital-request>)))
  "Returns full string definition for message of type '<SetDigital-request>"
  (format nil "uint8 pin~%uint8 dir~%uint8 value~%~%"))
(defmethod serialization-length ((msg <SetDigital-request>))
  (+ 0
     1
     1
     1
))
(defmethod ros-message-to-list ((msg <SetDigital-request>))
  "Converts a ROS message object to a list"
  (list '<SetDigital-request>
    (cons ':pin (pin-val msg))
    (cons ':dir (dir-val msg))
    (cons ':value (value-val msg))
))
;//! \htmlinclude SetDigital-response.msg.html

(defclass <SetDigital-response> (ros-message)
  ()
)
(defmethod serialize ((msg <SetDigital-response>) ostream)
  "Serializes a message object of type '<SetDigital-response>"
)
(defmethod deserialize ((msg <SetDigital-response>) istream)
  "Deserializes a message object of type '<SetDigital-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<SetDigital-response>)))
  "Returns string type for a service object of type '<SetDigital-response>"
  "arbotix/SetDigitalResponse")
(defmethod md5sum ((type (eql '<SetDigital-response>)))
  "Returns md5sum for a message object of type '<SetDigital-response>"
  "76cc68f0131aad3d0f5016da5d272467")
(defmethod message-definition ((type (eql '<SetDigital-response>)))
  "Returns full string definition for message of type '<SetDigital-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <SetDigital-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <SetDigital-response>))
  "Converts a ROS message object to a list"
  (list '<SetDigital-response>
))
(defmethod service-request-type ((msg (eql 'SetDigital)))
  '<SetDigital-request>)
(defmethod service-response-type ((msg (eql 'SetDigital)))
  '<SetDigital-response>)
(defmethod ros-datatype ((msg (eql 'SetDigital)))
  "Returns string type for a service object of type '<SetDigital>"
  "arbotix/SetDigital")
