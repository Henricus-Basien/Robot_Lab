; Auto-generated. Do not edit!


(cl:in-package navigation-msg)


;//! \htmlinclude MotorCommand.msg.html

(cl:defclass <MotorCommand> (roslisp-msg-protocol:ros-message)
  ((gain1
    :reader gain1
    :initarg :gain1
    :type cl:integer
    :initform 0)
   (gain2
    :reader gain2
    :initarg :gain2
    :type cl:integer
    :initform 0))
)

(cl:defclass MotorCommand (<MotorCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotorCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotorCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation-msg:<MotorCommand> is deprecated: use navigation-msg:MotorCommand instead.")))

(cl:ensure-generic-function 'gain1-val :lambda-list '(m))
(cl:defmethod gain1-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:gain1-val is deprecated.  Use navigation-msg:gain1 instead.")
  (gain1 m))

(cl:ensure-generic-function 'gain2-val :lambda-list '(m))
(cl:defmethod gain2-val ((m <MotorCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation-msg:gain2-val is deprecated.  Use navigation-msg:gain2 instead.")
  (gain2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotorCommand>) ostream)
  "Serializes a message object of type '<MotorCommand>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain1)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain2)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain2)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotorCommand>) istream)
  "Deserializes a message object of type '<MotorCommand>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain1)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gain2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gain2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gain2)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gain2)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotorCommand>)))
  "Returns string type for a message object of type '<MotorCommand>"
  "navigation/MotorCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotorCommand)))
  "Returns string type for a message object of type 'MotorCommand"
  "navigation/MotorCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotorCommand>)))
  "Returns md5sum for a message object of type '<MotorCommand>"
  "35e44d0055f2e6c6e49e1eb21a4d3474")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotorCommand)))
  "Returns md5sum for a message object of type 'MotorCommand"
  "35e44d0055f2e6c6e49e1eb21a4d3474")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotorCommand>)))
  "Returns full string definition for message of type '<MotorCommand>"
  (cl:format cl:nil "#> cat Drive.msg ~%uint32 gain1~%uint32 gain2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotorCommand)))
  "Returns full string definition for message of type 'MotorCommand"
  (cl:format cl:nil "#> cat Drive.msg ~%uint32 gain1~%uint32 gain2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotorCommand>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotorCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'MotorCommand
    (cl:cons ':gain1 (gain1 msg))
    (cl:cons ':gain2 (gain2 msg))
))
