;; Auto-generated. Do not edit!


(when (boundp 'fast_lio::Pose6D)
  (if (not (find-package "FAST_LIO"))
    (make-package "FAST_LIO"))
  (shadow 'Pose6D (find-package "FAST_LIO")))
(unless (find-package "FAST_LIO::POSE6D")
  (make-package "FAST_LIO::POSE6D"))

(in-package "ROS")
;;//! \htmlinclude Pose6D.msg.html


(defclass fast_lio::Pose6D
  :super ros::object
  :slots (_offset_time _acc _gyr _vel _pos _rot ))

(defmethod fast_lio::Pose6D
  (:init
   (&key
    ((:offset_time __offset_time) 0.0)
    ((:acc __acc) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:gyr __gyr) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:vel __vel) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:pos __pos) (make-array 3 :initial-element 0.0 :element-type :float))
    ((:rot __rot) (make-array 9 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _offset_time (float __offset_time))
   (setq _acc __acc)
   (setq _gyr __gyr)
   (setq _vel __vel)
   (setq _pos __pos)
   (setq _rot __rot)
   self)
  (:offset_time
   (&optional __offset_time)
   (if __offset_time (setq _offset_time __offset_time)) _offset_time)
  (:acc
   (&optional __acc)
   (if __acc (setq _acc __acc)) _acc)
  (:gyr
   (&optional __gyr)
   (if __gyr (setq _gyr __gyr)) _gyr)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:rot
   (&optional __rot)
   (if __rot (setq _rot __rot)) _rot)
  (:serialization-length
   ()
   (+
    ;; float64 _offset_time
    8
    ;; float64[3] _acc
    (* 8    3)
    ;; float64[3] _gyr
    (* 8    3)
    ;; float64[3] _vel
    (* 8    3)
    ;; float64[3] _pos
    (* 8    3)
    ;; float64[9] _rot
    (* 8    9)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _offset_time
       (sys::poke _offset_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[3] _acc
     (dotimes (i 3)
       (sys::poke (elt _acc i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _gyr
     (dotimes (i 3)
       (sys::poke (elt _gyr i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _vel
     (dotimes (i 3)
       (sys::poke (elt _vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[3] _pos
     (dotimes (i 3)
       (sys::poke (elt _pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[9] _rot
     (dotimes (i 9)
       (sys::poke (elt _rot i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _offset_time
     (setq _offset_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[3] _acc
   (dotimes (i (length _acc))
     (setf (elt _acc i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _gyr
   (dotimes (i (length _gyr))
     (setf (elt _gyr i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _vel
   (dotimes (i (length _vel))
     (setf (elt _vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[3] _pos
   (dotimes (i (length _pos))
     (setf (elt _pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[9] _rot
   (dotimes (i (length _rot))
     (setf (elt _rot i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get fast_lio::Pose6D :md5sum-) "ab486e9c24704038320abf9ff59003d2")
(setf (get fast_lio::Pose6D :datatype-) "fast_lio/Pose6D")
(setf (get fast_lio::Pose6D :definition-)
      "# the preintegrated Lidar states at the time of IMU measurements in a frame
float64  offset_time # the offset time of IMU measurement w.r.t the first lidar point
float64[3] acc       # the preintegrated total acceleration (global frame) at the Lidar origin
float64[3] gyr       # the unbiased angular velocity (body frame) at the Lidar origin
float64[3] vel       # the preintegrated velocity (global frame) at the Lidar origin
float64[3] pos       # the preintegrated position (global frame) at the Lidar origin
float64[9] rot       # the preintegrated rotation (global frame) at the Lidar origin
")



(provide :fast_lio/Pose6D "ab486e9c24704038320abf9ff59003d2")


