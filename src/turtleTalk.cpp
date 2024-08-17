#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <sstream>

ros::Subscriber turtle_pose_lis;
ros::Publisher turtle_vel_pub;
turtlesim::Pose turtle_pose;
bool place = false;
double r;

double distanceCal(double x, double y);
void pose_post(const turtlesim::Pose::ConstPtr& msg);
void move(double distance, double Kp, bool isForward);
void moveTo(double x, double y, double Kp);
void rotate(double x, double y, double Kp, bool isForward);
void goToGoal(double x, double y);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtleTalk");
    ros::NodeHandle n;
    turtle_pose_lis = n.subscribe("/turtle1/pose", 1000, pose_post);
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
    while(place == false) {
        ros::spinOnce();
    }
    double distance, angle, Kp, x, y;
    bool isForward;
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(100);
    std::cout << "STARTING PROCESS:" << std::endl;

    // std::cout << "Enter x to move: ";
    // std::cin >> x;
    // std::cout << "Enter y to move: ";
    // std::cin >> y;
    // std::cout << "Enter the Kp value: ";
    // std::cin >> Kp;
    // std::cout << "Enter the direction (true for forward, false for backward): ";
    // std::cin >> isForward;
    // std::cout << atan2(y - turtle_pose.y,x - turtle_pose.x) << std::endl;
    // rotate(x,y,Kp,isForward);
    // moveTo(x,y,Kp);

    std::cout << "Enter x to move: ";
    std::cin >> x;
    std::cout << "Enter y to move: ";
    std::cin >> y;
    goToGoal(x,y);
}

void pose_post(const turtlesim::Pose::ConstPtr& msg)
{
    turtle_pose.x = msg->x;
    turtle_pose.y = msg->y;
    turtle_pose.theta = msg->theta;
    place = true;
} 

void move(double distance, double Kp, bool isForward)
{
    ros::Rate loop_rate(10);
    geometry_msgs::Twist vel_msg;
    double K;
    if(isForward)
    {
        K = abs(Kp);
    } else {
        K = -abs(Kp);
    }
    double r0 = turtle_pose.x;
    double r = distance;
    while(r > 0.1){
        ros::spinOnce();
        r =distance - abs(turtle_pose.x - r0);
        vel_msg.linear.x = K*abs(r);
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("distance: %f", r);
        loop_rate.sleep();
    }
    vel_msg.linear.x = 0;
    turtle_vel_pub.publish(vel_msg);
}
void moveTo(double x, double y, double Kp)
{
    ros::Rate loop_rate(100);
    geometry_msgs::Twist vel_msg;
    double r0 = distanceCal(x, y);
    double tx = turtle_pose.x;
    double ty = turtle_pose.y;
    do {
        ros::spinOnce();
        r = r0 - distanceCal(tx,ty);
        vel_msg.linear.x = abs(Kp)*abs(r);
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("distance: %f", r);
        loop_rate.sleep();
    } while (r > 0.01);
    vel_msg.linear.x = 0;
    turtle_vel_pub.publish(vel_msg);
}
void rotate(double x, double y, double Kp, bool isForward)
{
    ros::Rate loop_rate(100);
    geometry_msgs::Twist vel_msg;
    double K;
    if(isForward)
    {
        K = abs(Kp);
    } else {
        K = -abs(Kp);
    }

    double r0 = atan2(y - turtle_pose.y,x - turtle_pose.x);
    ROS_INFO("Angle: %f", r0);
    double r = abs(r0 - turtle_pose.theta);
    while(r > 0.01){
        ros::spinOnce();
        r = abs(r0 - turtle_pose.theta);
        vel_msg.angular.z = -K*abs(r);
        turtle_vel_pub.publish(vel_msg);
        ROS_INFO("Angle: %f", turtle_pose.theta);
        loop_rate.sleep();
    }
    vel_msg.angular.z = 0;
    turtle_vel_pub.publish(vel_msg);
}
double distanceCal(double x, double y)
{
    return abs(sqrt(turtle_pose.x*turtle_pose.x + turtle_pose.y*turtle_pose.y) - sqrt(x*x + y*y));
}
void goToGoal(double x, double y)
{
    ros::Rate loop_rate(100);
    geometry_msgs::Twist vel_msg;
    double K;
    double r;
    double p;
    double E;
    // double p0 = atan2(y - turtle_pose.y,x - turtle_pose.x);
    // double r0 = sqrt(pow(abs(turtle_pose.x - x),2) + pow(abs(turtle_pose.y - y),2));
    double tx = turtle_pose.x;
    double ty = turtle_pose.y;
    // ROS_INFO("distance: %f", r0);
    ROS_INFO("distance: %f, %f", tx, ty);
    do {
        // E = 0;
        ros::spinOnce();
        K = 0.5;
        r = sqrt(pow(abs(turtle_pose.x - x),2) + pow(abs(turtle_pose.y - y),2));
        // E = r;
        vel_msg.linear.x = K*abs(r);
        K = 4;
        ROS_INFO("distance: %f", p);
        // E = E + p;
        p = abs(atan2(y - turtle_pose.y,x - turtle_pose.x) - turtle_pose.theta);
        if (p > 0.1){
            vel_msg.angular.z = K*abs(p);   
        } else {
            vel_msg.angular.z = 0;
        }
        turtle_vel_pub.publish(vel_msg);
        loop_rate.sleep();

    } while (r > 0.1);
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    turtle_vel_pub.publish(vel_msg);
}
