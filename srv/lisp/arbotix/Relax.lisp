; Auto-generated. Do not edit!


(in-package arbotix-srv)


;//! \htmlinclude Relax-request.msg.html

(defclass <Relax-request> (ros-message)
  ()
)
(defmethod serialize ((msg <Relax-request>) ostream)
  "Serializes a message object of type '<Relax-request>"
)
(defmethod deserialize ((msg <Relax-request>) istream)
  "Deserializes a message object of type '<Relax-request>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Relax-request>)))
  "Returns string type for a service object of type '<Relax-request>"
  "arbotix/RelaxRequest")
(defmethod md5sum ((type (eql '<Relax-request>)))
  "Returns md5sum for a message object of type '<Relax-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Relax-request>)))
  "Returns full string definition for message of type '<Relax-request>"
  (format nil "~%~%"))
(defmethod serialization-length ((msg <Relax-request>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Relax-request>))
  "Converts a ROS message object to a list"
  (list '<Relax-request>
))
;//! \htmlinclude Relax-response.msg.html

(defclass <Relax-response> (ros-message)
  ()
)
(defmethod serialize ((msg <Relax-response>) ostream)
  "Serializes a message object of type '<Relax-response>"
)
(defmethod deserialize ((msg <Relax-response>) istream)
  "Deserializes a message object of type '<Relax-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<Relax-response>)))
  "Returns string type for a service object of type '<Relax-response>"
  "arbotix/RelaxResponse")
(defmethod md5sum ((type (eql '<Relax-response>)))
  "Returns md5sum for a message object of type '<Relax-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(defmethod message-definition ((type (eql '<Relax-response>)))
  "Returns full string definition for message of type '<Relax-response>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <Relax-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <Relax-response>))
  "Converts a ROS message object to a list"
  (list '<Relax-response>
))
(defmethod service-request-type ((msg (eql 'Relax)))
  '<Relax-request>)
(defmethod service-response-type ((msg (eql 'Relax)))
  '<Relax-response>)
(defmethod ros-datatype ((msg (eql 'Relax)))
  "Returns string type for a service object of type '<Relax>"
  "arbotix/Relax")
