include "include/floorplan.inc"
include "include/box.inc"
include "include/erratic.inc" 
include "include/segway.inc" 
include "include/marrtino.inc" 

# set the resolution of the underlying raytrace model in meters
resolution 0.05

interval_sim 100  # simulation timestep in milliseconds

window ( 
  size   [ 600 400 1 ]         
  rotate [ 0.000 0.000 ]    
  center [ 6.0 11.0 ]   
  scale 23
  show_data 0
)

floorplan (
  size [30.2 29.1 1]
  pose [15.1 14.55 0 0]
  bitmap "saman.png"
)

# boxes
box( pose [ 7.02 21.34 0.1 0 ] color "blue")
box( pose [ 7.0 3.3 0 45 ] color "red")
box( pose [ 6.62 17.52 0.1 0 ] color "green")

door( pose [ 7.04 19.36 0.1 45 ] color "red")
