#include "Header.h"

turtlesim::Pose turtle_pose;
bool place = false;

void pose_post(const turtlesim::Pose::ConstPtr& msg)
{
    turtle_pose.x = msg->x;
    turtle_pose.y = msg->y;
    turtle_pose.theta = msg->theta;
    place = true;
} 

class goToGoalAction
{
    protected:
        ros::NodeHandle n;
        ros::Subscriber turtle_pose_lis = n.subscribe("/turtle1/pose", 1000, pose_post);
        ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
        actionlib::SimpleActionServer<lab_tutorials::goToGoalAction> as_;
        std::string action_name_;
        lab_tutorials::goToGoalFeedback feedback_;
        lab_tutorials::goToGoalResult result_;
    public:
        goToGoalAction(std::string name) : as_(n, name, boost::bind(&goToGoalAction::executeCB, this, _1), false),action_name_(name)
        {
            as_.start();
        }
        ~goToGoalAction(void)
        {
        }

        void executeCB(const lab_tutorials::goToGoalGoalConstPtr &goal)
        {
            ros::Rate loop_rate(100);
            bool success = true;

            ROS_INFO("%s: Executing, Going to (%f, %f)", action_name_.c_str(), goal->x, goal->y);

            geometry_msgs::Twist vel_msg;
            while(place == false) {
                ros::spinOnce();
            }
            double K;
            double r;
            double p;
            double tx = turtle_pose.x;
            double ty = turtle_pose.y;
            do {
                if (as_.isPreemptRequested() || !ros::ok())
                {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    as_.setPreempted();
                    success = false;
                    break;
                }
                ros::spinOnce();
                K = 0.5;
                r = sqrt(pow(abs(turtle_pose.x - goal->x),2) + pow(abs(turtle_pose.y - goal->y),2));
                vel_msg.linear.x = K*abs(r);
                K = 4;
                ROS_INFO("distance: %f", p);
                p = abs(atan2(goal->y - turtle_pose.y,goal->x - turtle_pose.x) - turtle_pose.theta);
                if (p > 0.1){
                    vel_msg.angular.z = K*abs(p);   
                } else {
                    vel_msg.angular.z = 0;
                }
                turtle_vel_pub.publish(vel_msg);
                feedback_.x=turtle_pose.x;
                feedback_.y=turtle_pose.y;
                // publish the feedback
                as_.publishFeedback(feedback_);
                loop_rate.sleep();

            } while (r > 0.1);
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            turtle_vel_pub.publish(vel_msg);

            if(success)
            {
            result_.x = r;
            ROS_INFO("%s: Succeeded", action_name_.c_str());
            // set the action state to succeeded
            as_.setSucceeded(result_);
            }
        }


};
