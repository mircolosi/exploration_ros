#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exploration_ros/ExplorerAction.h>

#ifndef COLORS
#define COLORS

#define RESET  "\x1B[0m"
#define RED  "\x1B[31m"
#define GREEN  "\x1B[32m"

#endif 

typedef actionlib::SimpleActionClient<exploration_ros::ExplorerAction> ExplorerActionClient;

int main (int argc, char **argv)
{
  class callbackObject {
  public:
    callbackObject() {

      ros::NodeHandle this_node;
      std::string action;
      std::string prefix(ros::this_node::getName()+"/");
      this_node.getParam(this_node.resolveName(prefix+"action", true), action);

      _exp_client = new ExplorerActionClient(action, true); 

      // wait for the action server to start
      while(!_exp_client->waitForServer(ros::Duration(5.0))){
        std::cerr << GREEN << "Waiting for the explorer action server to come up" << std::endl;
        std::cerr << RESET;
      }
      ROS_INFO("Action server started, sending goal.");
    }

    ~callbackObject() {
      delete _exp_client;
    }

    inline void explorationCB(const ros::TimerEvent& event) {
      _exp_client->cancelAllGoals();

      std::cerr << GREEN << "EXPLORATION message" << std::endl;
      std::cerr << RESET;

      exploration_ros::ExplorerGoal msg;
      msg.goal.action = "exploration";

      actionlib::SimpleClientGoalState goalState = _exp_client->getState();
      std::cerr << GREEN << "GOAL state: " << goalState.toString() << std::endl;
      std::cerr << RESET;
      _exp_client->sendGoal(msg);
      _exp_client->waitForResult();

      // this->waitForGoal();
    }

    inline void targetCB(const ros::TimerEvent& event) {
      _exp_client->cancelAllGoals();

      std::cerr << GREEN << "TARGET message" << std::endl;
      std::cerr << RESET;
      // send a goal to the action
      exploration_ros::ExplorerGoal msg;

      msg.goal.action = "target";
      msg.goal.target_pose.position.x = 160;
      msg.goal.target_pose.position.y = 160;

      actionlib::SimpleClientGoalState goalState = _exp_client->getState();
      std::cerr << GREEN << "GOAL state: " << goalState.toString() << std::endl;
      std::cerr << RESET;
      _exp_client->sendGoal(msg);
      _exp_client->waitForResult();

      // this->waitForGoal();
    }

    inline void waitForGoal() {
      ros::Rate loop_rate1(15);
      ros::Rate loop_rate2(2);

      while(_exp_client->getState() != actionlib::SimpleClientGoalState::ACTIVE){
        loop_rate1.sleep();
      }
      bool reached = false;
      while (!reached){
        reached = isGoalReached();
      }
      _exp_client->cancelAllGoals();

    }

    inline bool isGoalReached() {

      actionlib::SimpleClientGoalState goalState = _exp_client->getState(); 

      if (goalState == actionlib::SimpleClientGoalState::SUCCEEDED){
        std::cerr << GREEN << "GOAL state: " << goalState.toString() << std::endl;
        printf("ExplorerActionClient: The goal SUCCEEDED\n");
        std::cerr << RESET;
        return true;
      }

      if (goalState == actionlib::SimpleClientGoalState::ABORTED){
        _exp_client->cancelAllGoals();
        std::cerr << GREEN << "GOAL state: " << goalState.toString() << std::endl;
        printf("ExplorerActionClient: The goal has been ABORTED\n");
        std::cerr << RESET;
        return true;
      }

      if ((goalState == actionlib::SimpleClientGoalState::RECALLED) || (goalState == actionlib::SimpleClientGoalState::PREEMPTED)){
        std::cerr << GREEN << "GOAL state: " << goalState.toString() << std::endl;
        printf("ExplorerActionClient: The goal has been PREEMPTED\n<");
        std::cerr << RESET;
        return true;
      }

      return false;


    }


  private: 
    ExplorerActionClient* _exp_client;
  };

  ros::init(argc, argv, "explorer_client");
  ros::NodeHandle timers_handler;

  callbackObject* cb = new callbackObject();

  ros::Timer timer1 = timers_handler.createTimer(ros::Duration(0.1), &callbackObject::explorationCB, cb);
  // ros::Timer timer2 = timers_handler.createTimer(ros::Duration(20.0), &callbackObject::targetCB, cb, true);

  ros::spin();


  delete cb;
  // //exit
  return 0;
}
