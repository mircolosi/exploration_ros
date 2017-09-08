add_executable(exploration_action_node ExplorerAction.cpp
  ../exploration/fake_projector.cpp
  ../exploration/frontier_detector.cpp
  ../exploration/goal_planner.cpp
  ../exploration/paths_rollout.cpp)

target_link_libraries(exploration_action_node
  ${catkin_LIBRARIES}
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

add_executable(exploration_node explorer_server_node.cpp
  explorer_server.cpp
  ../exploration/fake_projector.cpp
  ../exploration/frontier_detector.cpp
  ../exploration/goal_planner.cpp
  ../exploration/paths_rollout.cpp)

target_link_libraries(exploration_node
  ${catkin_LIBRARIES}
)

add_dependencies(exploration_node
   exploration_ros_generate_messages_cpp
)
