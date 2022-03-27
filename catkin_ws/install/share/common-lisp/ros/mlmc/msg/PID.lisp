; Auto-generated. Do not edit!


(cl:in-package mlmc-msg)


;//! \htmlinclude PID.msg.html

(cl:defclass <PID> (roslisp-msg-protocol:ros-message)
  ((p
    :reader p
    :initarg :p
    :type cl:float
    :initform 0.0)
   (i
    :reader i
    :initarg :i
    :type cl:float
    :initform 0.0)
   (d
    :reader d
    :initarg :d
    :type cl:float
    :initform 0.0)
   (ffd0
    :reader ffd0
    :initarg :ffd0
    :type cl:float
    :initform 0.0)
   (ffd1
    :reader ffd1
    :initarg :ffd1
    :type cl:float
    :initform 0.0))
)

(cl:defclass PID (<PID>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PID>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PID)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mlmc-msg:<PID> is deprecated: use mlmc-msg:PID instead.")))

(cl:ensure-generic-function 'p-val :lambda-list '(m))
(cl:defmethod p-val ((m <PID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-msg:p-val is deprecated.  Use mlmc-msg:p instead.")
  (p m))

(cl:ensure-generic-function 'i-val :lambda-list '(m))
(cl:defmethod i-val ((m <PID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-msg:i-val is deprecated.  Use mlmc-msg:i instead.")
  (i m))

(cl:ensure-generic-function 'd-val :lambda-list '(m))
(cl:defmethod d-val ((m <PID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-msg:d-val is deprecated.  Use mlmc-msg:d instead.")
  (d m))

(cl:ensure-generic-function 'ffd0-val :lambda-list '(m))
(cl:defmethod ffd0-val ((m <PID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-msg:ffd0-val is deprecated.  Use mlmc-msg:ffd0 instead.")
  (ffd0 m))

(cl:ensure-generic-function 'ffd1-val :lambda-list '(m))
(cl:defmethod ffd1-val ((m <PID>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mlmc-msg:ffd1-val is deprecated.  Use mlmc-msg:ffd1 instead.")
  (ffd1 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PID>) ostream)
  "Serializes a message object of type '<PID>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'i))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ffd0))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'ffd1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PID>) istream)
  "Deserializes a message object of type '<PID>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'i) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'd) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ffd0) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ffd1) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PID>)))
  "Returns string type for a message object of type '<PID>"
  "mlmc/PID")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PID)))
  "Returns string type for a message object of type 'PID"
  "mlmc/PID")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PID>)))
  "Returns md5sum for a message object of type '<PID>"
  "a76881aad551f4c46d9753b2549d471b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PID)))
  "Returns md5sum for a message object of type 'PID"
  "a76881aad551f4c46d9753b2549d471b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PID>)))
  "Returns full string definition for message of type '<PID>"
  (cl:format cl:nil "float32 p~%float32 i~%float32 d~%float32 ffd0~%float32 ffd1~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PID)))
  "Returns full string definition for message of type 'PID"
  (cl:format cl:nil "float32 p~%float32 i~%float32 d~%float32 ffd0~%float32 ffd1~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PID>))
  (cl:+ 0
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PID>))
  "Converts a ROS message object to a list"
  (cl:list 'PID
    (cl:cons ':p (p msg))
    (cl:cons ':i (i msg))
    (cl:cons ':d (d msg))
    (cl:cons ':ffd0 (ffd0 msg))
    (cl:cons ':ffd1 (ffd1 msg))
))
