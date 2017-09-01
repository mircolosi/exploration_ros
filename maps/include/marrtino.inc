include "include/hokuyo.inc"

define marrtino position
(
  size [0.400 0.400 0.250]
  
  # This block approximates a circular shape of a Robot
   
  block( points 8
        point[0] [ 0.0 0.04 ]
        point[1] [ 0.0 0.36 ]
        point[2] [ 0.04 0.4 ]
        point[3] [ 0.36 0.4 ]
        point[4] [ 0.4 0.36 ]
        point[5] [ 0.4 0.04 ]
        point[6] [ 0.36 0.0 ]
        point[7] [ 0.04 0.0 ]
        z [0 1]
  ) 
  origin [-0.050 0 0 0]
  gui_nose 1
  drive "diff"
  localization "odom"
  odom_error [ 0.01 0.01 0.0 0.1 ]
  utm30(pose [ 0.050 0 -0.1 0 ])
)

