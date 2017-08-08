#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <exploration_ros/ExplorerAction.h>

typedef actionlib::SimpleActionClient<exploration_ros::ExplorerAction> ExplorerAction;

int main (int argc, char **argv)
{
  class callbackObject {
  public:
    callbackObject() : _exp_action(new ExplorerAction("explorer", true)) {
      ROS_INFO("Waiting for action server to start.");
      // wait for the action server to start
      while(!_exp_action->waitForServer(ros::Duration(5.0))){
        std::cerr << "Waiting for the move_base action server to come up" << std::endl;
      }
      ROS_INFO("Action server started, sending goal.");
    }
    ~callbackObject() {delete _exp_action;}

    inline void explorationCB(const ros::TimerEvent& event) {
      _exp_action->cancelAllGoals();

      std::cerr << "Sending exploration message" << std::endl;
      exploration_ros::ExplorerGoal msg;
      msg.goal.action = "exploration";

      _exp_action->sendGoal(msg);
      _exp_action->waitForResult();
    }

    inline void targetCB(const ros::TimerEvent& event) {
      _exp_action->cancelAllGoals();

      std::cerr << "Sending goal" << std::endl;
      // send a goal to the action
      exploration_ros::ExplorerGoal msg;

      msg.goal.action = "target";
      msg.goal.target_pose.position.x = 160;
      msg.goal.target_pose.position.y = 160;

      _exp_action->sendGoal(msg);
      _exp_action->waitForResult();

    }

    inline ExplorerAction* getExpAction() {return _exp_action;}

  private: 
    ExplorerAction* _exp_action;
  };

  ros::init(argc, argv, "test_explorer");
  ros::NodeHandle timers_handler;

  callbackObject* cb = new callbackObject();

  ros::Timer timer1 = timers_handler.createTimer(ros::Duration(0.1), &callbackObject::explorationCB, cb);
  ros::Timer timer2 = timers_handler.createTimer(ros::Duration(20.0), &callbackObject::targetCB, cb, true);

  ros::spin();


  delete cb;
  // //exit
  return 0;
}
