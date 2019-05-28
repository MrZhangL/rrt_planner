;; Auto-generated. Do not edit!


(when (boundp 'costmap_2d::DstarPoint)
  (if (not (find-package "COSTMAP_2D"))
    (make-package "COSTMAP_2D"))
  (shadow 'DstarPoint (find-package "COSTMAP_2D")))
(unless (find-package "COSTMAP_2D::DSTARPOINT")
  (make-package "COSTMAP_2D::DSTARPOINT"))

(in-package "ROS")
;;//! \htmlinclude DstarPoint.msg.html


(defclass costmap_2d::DstarPoint
  :super ros::object
  :slots (_index ))

(defmethod costmap_2d::DstarPoint
  (:init
   (&key
    ((:index __index) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _index __index)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:serialization-length
   ()
   (+
    ;; int32[] _index
    (* 4    (length _index)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _index
     (write-long (length _index) s)
     (dotimes (i (length _index))
       (write-long (elt _index i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get costmap_2d::DstarPoint :md5sum-) "df62f201f3c7cdf0f85b4ec3adb4c299")
(setf (get costmap_2d::DstarPoint :datatype-) "costmap_2d/DstarPoint")
(setf (get costmap_2d::DstarPoint :definition-)
      "int32[] index
")



(provide :costmap_2d/DstarPoint "df62f201f3c7cdf0f85b4ec3adb4c299")


