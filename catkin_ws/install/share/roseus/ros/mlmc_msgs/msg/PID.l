;; Auto-generated. Do not edit!


(when (boundp 'mlmc_msgs::PID)
  (if (not (find-package "MLMC_MSGS"))
    (make-package "MLMC_MSGS"))
  (shadow 'PID (find-package "MLMC_MSGS")))
(unless (find-package "MLMC_MSGS::PID")
  (make-package "MLMC_MSGS::PID"))

(in-package "ROS")
;;//! \htmlinclude PID.msg.html


(defclass mlmc_msgs::PID
  :super ros::object
  :slots (_p _i _d _ffd0 _ffd1 ))

(defmethod mlmc_msgs::PID
  (:init
   (&key
    ((:p __p) 0.0)
    ((:i __i) 0.0)
    ((:d __d) 0.0)
    ((:ffd0 __ffd0) 0.0)
    ((:ffd1 __ffd1) 0.0)
    )
   (send-super :init)
   (setq _p (float __p))
   (setq _i (float __i))
   (setq _d (float __d))
   (setq _ffd0 (float __ffd0))
   (setq _ffd1 (float __ffd1))
   self)
  (:p
   (&optional __p)
   (if __p (setq _p __p)) _p)
  (:i
   (&optional __i)
   (if __i (setq _i __i)) _i)
  (:d
   (&optional __d)
   (if __d (setq _d __d)) _d)
  (:ffd0
   (&optional __ffd0)
   (if __ffd0 (setq _ffd0 __ffd0)) _ffd0)
  (:ffd1
   (&optional __ffd1)
   (if __ffd1 (setq _ffd1 __ffd1)) _ffd1)
  (:serialization-length
   ()
   (+
    ;; float32 _p
    4
    ;; float32 _i
    4
    ;; float32 _d
    4
    ;; float32 _ffd0
    4
    ;; float32 _ffd1
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _p
       (sys::poke _p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _i
       (sys::poke _i (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _d
       (sys::poke _d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ffd0
       (sys::poke _ffd0 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _ffd1
       (sys::poke _ffd1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _p
     (setq _p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _i
     (setq _i (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _d
     (setq _d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ffd0
     (setq _ffd0 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _ffd1
     (setq _ffd1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mlmc_msgs::PID :md5sum-) "a76881aad551f4c46d9753b2549d471b")
(setf (get mlmc_msgs::PID :datatype-) "mlmc_msgs/PID")
(setf (get mlmc_msgs::PID :definition-)
      "float32 p
float32 i
float32 d
float32 ffd0
float32 ffd1
")



(provide :mlmc_msgs/PID "a76881aad551f4c46d9753b2549d471b")


