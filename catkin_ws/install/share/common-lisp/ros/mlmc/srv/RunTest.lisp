; Auto-generated. Do not edit!


(cl:in-package mlmc-srv)


;//! \htmlinclude RunTest-request.msg.html

(cl:defclass <RunTest-request> (roslisp-msg-protocol:ros-message)
  ((start
    :reader start
    :initarg :start
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RunTest-request (<RunTest-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunTest-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunTest-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlmc-srv:<RunTest-request> is deprecated: use mlmc-srv:RunTest-request instead.")))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <RunTest-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-srv:start-val is deprecated.  Use mlmc-srv:start instead.")
  (start m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunTest-request>) ostream)
  "Serializes a message object of type '<RunTest-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'start) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunTest-request>) istream)
  "Deserializes a message object of type '<RunTest-request>"
    (cl:setf (cl:slot-value msg 'start) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunTest-request>)))
  "Returns string type for a service object of type '<RunTest-request>"
  "mlmc/RunTestRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunTest-request)))
  "Returns string type for a service object of type 'RunTest-request"
  "mlmc/RunTestRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunTest-request>)))
  "Returns md5sum for a message object of type '<RunTest-request>"
  "db324d824adf6dca3d2931e8d8ebd4ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunTest-request)))
  "Returns md5sum for a message object of type 'RunTest-request"
  "db324d824adf6dca3d2931e8d8ebd4ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunTest-request>)))
  "Returns full string definition for message of type '<RunTest-request>"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunTest-request)))
  "Returns full string definition for message of type 'RunTest-request"
  (cl:format cl:nil "bool start~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunTest-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunTest-request>))
  "Converts a ROS message object to a list"
  (cl:list 'RunTest-request
    (cl:cons ':start (start msg))
))
;//! \htmlinclude RunTest-response.msg.html

(cl:defclass <RunTest-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass RunTest-response (<RunTest-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RunTest-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RunTest-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlmc-srv:<RunTest-response> is deprecated: use mlmc-srv:RunTest-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <RunTest-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-srv:finished-val is deprecated.  Use mlmc-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RunTest-response>) ostream)
  "Serializes a message object of type '<RunTest-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RunTest-response>) istream)
  "Deserializes a message object of type '<RunTest-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RunTest-response>)))
  "Returns string type for a service object of type '<RunTest-response>"
  "mlmc/RunTestResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunTest-response)))
  "Returns string type for a service object of type 'RunTest-response"
  "mlmc/RunTestResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RunTest-response>)))
  "Returns md5sum for a message object of type '<RunTest-response>"
  "db324d824adf6dca3d2931e8d8ebd4ef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RunTest-response)))
  "Returns md5sum for a message object of type 'RunTest-response"
  "db324d824adf6dca3d2931e8d8ebd4ef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RunTest-response>)))
  "Returns full string definition for message of type '<RunTest-response>"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RunTest-response)))
  "Returns full string definition for message of type 'RunTest-response"
  (cl:format cl:nil "bool finished~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RunTest-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RunTest-response>))
  "Converts a ROS message object to a list"
  (cl:list 'RunTest-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'RunTest)))
  'RunTest-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'RunTest)))
  'RunTest-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RunTest)))
  "Returns string type for a service object of type '<RunTest>"
  "mlmc/RunTest")