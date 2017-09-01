define box model
(
  size [0.400 0.400 1.00]
  gui_nose 0
)


define door model
(
  size [1.000 0.200 1.00]
  gui_nose 0
)

define circle model
(
  size [0.600 0.300 0.80]
  
  block(
       points 8
       point[0] [-0.2 0.12]
       point[1] [-0.2 -0.12]
       point[2] [-0.12 -0.2555]
       point[3] [0.12 -0.2555]
       point[4] [0.2 -0.12]
       point[5] [0.2 0.12]
       point[6] [0.12 0.2555]
       point[7] [-0.12 0.2555]
       z [0 1.0]
  )
  
  obstacle_return 1
  ranger_return 1.0
  gui_outline 1
  gui_nose 0
)

define circle16 model
(
  size [0.600 0.300 0.80]
  
  block( points 16
        point[0] [ 0.225 0.000 ]
        point[1] [ 0.208 0.086 ]
        point[2] [ 0.159 0.159 ]
        point[3] [ 0.086 0.208 ]
        point[4] [ 0.000 0.225 ]
        point[5] [ -0.086 0.208 ]
        point[6] [ -0.159 0.159 ]
        point[7] [ -0.208 0.086 ]
        point[8] [ -0.225 0.000 ]
        point[9] [ -0.208 -0.086 ]
        point[10] [ -0.159 -0.159 ]
        point[11] [ -0.086 -0.208 ]
        point[12] [ -0.000 -0.225 ]
        point[13] [ 0.086 -0.208 ]
        point[14] [ 0.159 -0.159 ]
        point[15] [ 0.208 -0.086 ]
        z [0 1]
  )

  gui_nose 0
)

