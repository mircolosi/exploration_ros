add_executable(exploration_action_node ExplorerAction.cpp
  ../exploration/fake_projector.cpp
  ../exploration/frontier_detector.cpp
  ../exploration/goal_planner.cpp
  ../exploration/paths_rollout.cpp)

target_link_libraries(exploration_action_node
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

add_dependencies(exploration_action_node
	 exploration_ros_generate_messages_cpp
)

add_executable(exploration_action_client ExplorerClient.cpp)

target_link_libraries(exploration_action_client
  ${catkin_LIBRARIES}
)

add_dependencies(exploration_action_client
   exploration_ros_generate_messages_cpp
)