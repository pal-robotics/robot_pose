
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "tf/transform_listener.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle node;
  ros::Publisher pose_pub =
      node.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);

  tf::TransformListener listener;

  ros::Rate rate(100);

  geometry_msgs::PoseWithCovarianceStamped robot_pose_msg;

  while (node.ok())
  {
    tf::StampedTransform transform;

    try
    {
      listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(1.0));
      listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    robot_pose_msg.header.frame_id = "/map";
    robot_pose_msg.header.stamp = ros::Time::now();

    robot_pose_msg.pose.pose.position.x = transform.getOrigin().x();
    robot_pose_msg.pose.pose.position.y = transform.getOrigin().y();
    robot_pose_msg.pose.pose.position.z = transform.getOrigin().z();

    robot_pose_msg.pose.pose.orientation.x = transform.getRotation().x();
    robot_pose_msg.pose.pose.orientation.y = transform.getRotation().y();
    robot_pose_msg.pose.pose.orientation.z = transform.getRotation().z();
    robot_pose_msg.pose.pose.orientation.w = transform.getRotation().w();

    pose_pub.publish(robot_pose_msg);

    rate.sleep();
  }
  return 0;
}
