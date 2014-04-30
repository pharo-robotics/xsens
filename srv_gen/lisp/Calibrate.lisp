; Auto-generated. Do not edit!


(cl:in-package xsens-srv)


;//! \htmlinclude Calibrate-request.msg.html

(cl:defclass <Calibrate-request> (roslisp-msg-protocol:ros-message)
  ((kind
    :reader kind
    :initarg :kind
    :type cl:integer
    :initform 0)
   (samples
    :reader samples
    :initarg :samples
    :type cl:integer
    :initform 0))
)

(cl:defclass Calibrate-request (<Calibrate-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Calibrate-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Calibrate-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xsens-srv:<Calibrate-request> is deprecated: use xsens-srv:Calibrate-request instead.")))

(cl:ensure-generic-function 'kind-val :lambda-list '(m))
(cl:defmethod kind-val ((m <Calibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens-srv:kind-val is deprecated.  Use xsens-srv:kind instead.")
  (kind m))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <Calibrate-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens-srv:samples-val is deprecated.  Use xsens-srv:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Calibrate-request>)))
    "Constants for message type '<Calibrate-request>"
  '((:ORIENTATION . 1)
    (:DRIFT . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Calibrate-request)))
    "Constants for message type 'Calibrate-request"
  '((:ORIENTATION . 1)
    (:DRIFT . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Calibrate-request>) ostream)
  "Serializes a message object of type '<Calibrate-request>"
  (cl:let* ((signed (cl:slot-value msg 'kind)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'samples)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Calibrate-request>) istream)
  "Deserializes a message object of type '<Calibrate-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'kind) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'samples) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Calibrate-request>)))
  "Returns string type for a service object of type '<Calibrate-request>"
  "xsens/CalibrateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibrate-request)))
  "Returns string type for a service object of type 'Calibrate-request"
  "xsens/CalibrateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Calibrate-request>)))
  "Returns md5sum for a message object of type '<Calibrate-request>"
  "728ba486b30d3957849534789a026b96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Calibrate-request)))
  "Returns md5sum for a message object of type 'Calibrate-request"
  "728ba486b30d3957849534789a026b96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Calibrate-request>)))
  "Returns full string definition for message of type '<Calibrate-request>"
  (cl:format cl:nil "~%int32 ORIENTATION=1~%int32 DRIFT=2~%~%int32 kind~%int32 samples~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Calibrate-request)))
  "Returns full string definition for message of type 'Calibrate-request"
  (cl:format cl:nil "~%int32 ORIENTATION=1~%int32 DRIFT=2~%~%int32 kind~%int32 samples~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Calibrate-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Calibrate-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Calibrate-request
    (cl:cons ':kind (kind msg))
    (cl:cons ':samples (samples msg))
))
;//! \htmlinclude Calibrate-response.msg.html

(cl:defclass <Calibrate-response> (roslisp-msg-protocol:ros-message)
  ((compensation
    :reader compensation
    :initarg :compensation
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Calibrate-response (<Calibrate-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Calibrate-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Calibrate-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name xsens-srv:<Calibrate-response> is deprecated: use xsens-srv:Calibrate-response instead.")))

(cl:ensure-generic-function 'compensation-val :lambda-list '(m))
(cl:defmethod compensation-val ((m <Calibrate-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader xsens-srv:compensation-val is deprecated.  Use xsens-srv:compensation instead.")
  (compensation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Calibrate-response>) ostream)
  "Serializes a message object of type '<Calibrate-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'compensation) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Calibrate-response>) istream)
  "Deserializes a message object of type '<Calibrate-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'compensation) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Calibrate-response>)))
  "Returns string type for a service object of type '<Calibrate-response>"
  "xsens/CalibrateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibrate-response)))
  "Returns string type for a service object of type 'Calibrate-response"
  "xsens/CalibrateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Calibrate-response>)))
  "Returns md5sum for a message object of type '<Calibrate-response>"
  "728ba486b30d3957849534789a026b96")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Calibrate-response)))
  "Returns md5sum for a message object of type 'Calibrate-response"
  "728ba486b30d3957849534789a026b96")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Calibrate-response>)))
  "Returns full string definition for message of type '<Calibrate-response>"
  (cl:format cl:nil "geometry_msgs/Vector3 compensation~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Calibrate-response)))
  "Returns full string definition for message of type 'Calibrate-response"
  (cl:format cl:nil "geometry_msgs/Vector3 compensation~%~%~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Calibrate-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'compensation))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Calibrate-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Calibrate-response
    (cl:cons ':compensation (compensation msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Calibrate)))
  'Calibrate-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Calibrate)))
  'Calibrate-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Calibrate)))
  "Returns string type for a service object of type '<Calibrate>"
  "xsens/Calibrate")