#include <ros/ros.h>
#include <lab_tutorials/goToGoalAction.h> 
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<lab_tutorials::goToGoalAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "goToGoal_client");
  Client client("goToGoal", true); 
  client.waitForServer();
  lab_tutorials::goToGoalGoal goal;
  double x,y;
  std::cout << "Enter x:";
  std::cin >> x;
  std::cout << "Enter y:";
  std::cin >> y;
  goal.x = x;
  goal.y = y;
  client.sendGoal(goal);
  client.waitForResult(ros::Duration(10.0));
  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    printf("Turtle reached the goal!");
  printf(" Current State: %s\n", client.getState().toString().c_str());
  return 0;
}