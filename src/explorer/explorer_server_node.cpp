#include "explorer_server.h"

int main(int argc, char** argv) {

  ros::init(argc, argv, "explorer_server");

  ros::NodeHandle n;
  ros::NodeHandle private_nh("~");

  ExplorerServer* action_server= new ExplorerServer(n);

  ros::spin();

  return 0;
}


