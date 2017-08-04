add_executable(frontier_planner
  frontier_planner.cpp
  ../exploration/fake_projector.cpp
  ../exploration/frontier_detector.cpp
  ../exploration/goal_planner.cpp
  ../exploration/paths_rollout.cpp
)
target_link_libraries(frontier_planner
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}    
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(frontier_planner exploration_ros_generate_messages_cpp)

add_executable(slam_node
  slam_node.cpp
  ../sr_ros_utils/ros_handler_sr.cpp
  ../map_creation/grid_line_traversal.cpp
  ../map_creation/frequency_map.cpp
  ../map_creation/graph2occupancy.cpp
  ../map_creation/occupancy_map_server.cpp

)
target_link_libraries(slam_node  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}    
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
)
add_dependencies(slam_node exploration_ros_generate_messages_cpp)

add_executable(mapper_node
  mapper_node.cpp
  ../sr_ros_utils/ros_handler_sr.cpp
  ../map_creation/grid_line_traversal.cpp
  ../map_creation/frequency_map.cpp
  ../map_creation/graph2occupancy.cpp
  ../map_creation/occupancy_map_server.cpp
)
target_link_libraries(mapper_node  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}    
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}

)
add_dependencies(mapper_node exploration_ros_generate_messages_cpp)

add_executable(goal_check
  goal_check.cpp
)
target_link_libraries(goal_check  
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}    
  ${CSPARSE_LIBRARY}
  ${G2O_TYPES_DATA}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}

)
add_dependencies(goal_check exploration_ros_generate_messages_cpp)


