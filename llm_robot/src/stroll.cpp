#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <random>

int main(int argc, char** argv){

    ros::init(argc, argv, "stroll_node");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Publisher>("/move_base/goal", 10);

    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped goal;

    goal.header.frame_id = "map";
    goal.header.stamp = ros::Time::now();

    goal.pose.position.x = 2.0;
    goal.pose.position.y = 3.0;



}



#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "send_goal_node");
    ros::NodeHandle nh;

    ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base/goal", 10);

    // Wait for the publisher to connect to any subscribers
    ros::Duration(1.0).sleep();

    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "map";  // or "odom", depending on your setup
    goal.header.stamp = ros::Time::now();

    // Set position (example: x=2.0, y=3.0)
    goal.pose.position.x = 2.0;
    goal.pose.position.y = 3.0;
    goal.pose.position.z = 0.0;

    // Set orientation (quaternion for 0 radians yaw)
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = 0.0;
    goal.pose.orientation.w = 1.0;

    ROS_INFO("Publishing goal to /move_base/goal...");
    goal_pub.publish(goal);

    ros::spinOnce();
    ros::Duration(0.5).sleep();  // Allow time for message to be sent

    return 0;
}
