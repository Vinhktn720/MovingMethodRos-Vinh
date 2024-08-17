#include <ros/ros.h>
#include <lab_tutorials/goToGoalAction.h> 
#include <actionlib/server/simple_action_server.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

class goToGoalAction
{
    protected:
        ros::NodeHandle n;
        ros::Subscriber turtle_pose_lis;
        ros::Publisher turtle_vel_pub;
        actionlib::SimpleActionServer<lab_tutorials::goToGoalAction> as_;
        std::string action_name_;
        lab_tutorials::goToGoalFeedback feedback_;
        lab_tutorials::goToGoalResult result_;
    public:
        goToGoalAction(std::string name) : as_(n, name, boost::bind(&goToGoalAction::executeCB, this, _1), false),action_name_(name)
        {
            as_.start();
        }
        ~goToGoalAction();
        void executeCB(const lab_tutorials::goToGoalGoalConstPtr &goal);
        bool init();
};
