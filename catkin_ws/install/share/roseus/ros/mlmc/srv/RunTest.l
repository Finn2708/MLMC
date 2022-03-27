;; Auto-generated. Do not edit!


(when (boundp 'mlmc::RunTest)
  (if (not (find-package "MLMC"))
    (make-package "MLMC"))
  (shadow 'RunTest (find-package "MLMC")))
(unless (find-package "MLMC::RUNTEST")
  (make-package "MLMC::RUNTEST"))
(unless (find-package "MLMC::RUNTESTREQUEST")
  (make-package "MLMC::RUNTESTREQUEST"))
(unless (find-package "MLMC::RUNTESTRESPONSE")
  (make-package "MLMC::RUNTESTRESPONSE"))

(in-package "ROS")





(defclass mlmc::RunTestRequest
  :super ros::object
  :slots (_start ))

(defmethod mlmc::RunTestRequest
  (:init
   (&key
    ((:start __start) nil)
    )
   (send-super :init)
   (setq _start __start)
   self)
  (:start
   (&optional (__start :null))
   (if (not (eq __start :null)) (setq _start __start)) _start)
  (:serialization-length
   ()
   (+
    ;; bool _start
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _start
       (if _start (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _start
     (setq _start (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mlmc::RunTestResponse
  :super ros::object
  :slots (_finished ))

(defmethod mlmc::RunTestResponse
  (:init
   (&key
    ((:finished __finished) nil)
    )
   (send-super :init)
   (setq _finished __finished)
   self)
  (:finished
   (&optional (__finished :null))
   (if (not (eq __finished :null)) (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; bool _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mlmc::RunTest
  :super ros::object
  :slots ())

(setf (get mlmc::RunTest :md5sum-) "db324d824adf6dca3d2931e8d8ebd4ef")
(setf (get mlmc::RunTest :datatype-) "mlmc/RunTest")
(setf (get mlmc::RunTest :request) mlmc::RunTestRequest)
(setf (get mlmc::RunTest :response) mlmc::RunTestResponse)

(defmethod mlmc::RunTestRequest
  (:response () (instance mlmc::RunTestResponse :init)))

(setf (get mlmc::RunTestRequest :md5sum-) "db324d824adf6dca3d2931e8d8ebd4ef")
(setf (get mlmc::RunTestRequest :datatype-) "mlmc/RunTestRequest")
(setf (get mlmc::RunTestRequest :definition-)
      "bool start
---
bool finished
")

(setf (get mlmc::RunTestResponse :md5sum-) "db324d824adf6dca3d2931e8d8ebd4ef")
(setf (get mlmc::RunTestResponse :datatype-) "mlmc/RunTestResponse")
(setf (get mlmc::RunTestResponse :definition-)
      "bool start
---
bool finished
")



(provide :mlmc/RunTest "db324d824adf6dca3d2931e8d8ebd4ef")


