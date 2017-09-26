
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_pose_publisher");
  ros::NodeHandle node;
  ros::Publisher pose_pub =
      node.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener listener(tf_buffer);

  ros::Rate rate(100);

  geometry_msgs::PoseWithCovarianceStamped robot_pose_msg;

  while (node.ok())
  {
    geometry_msgs::TransformStamped map_to_base;

    try
    {
      map_to_base = tf_buffer.lookupTransform("map", "base_footprint", ros::Time(0),
                                              ros::Duration(1.0));
    }
    catch (tf2::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    robot_pose_msg.header.frame_id = "/map";
    robot_pose_msg.header.stamp = ros::Time::now();

    robot_pose_msg.pose.pose.position.x = map_to_base.transform.translation.x;
    robot_pose_msg.pose.pose.position.y = map_to_base.transform.translation.y;
    robot_pose_msg.pose.pose.position.z = map_to_base.transform.translation.z;

    robot_pose_msg.pose.pose.orientation.x = map_to_base.transform.rotation.x;
    robot_pose_msg.pose.pose.orientation.y = map_to_base.transform.rotation.y;
    robot_pose_msg.pose.pose.orientation.z = map_to_base.transform.rotation.z;
    robot_pose_msg.pose.pose.orientation.w = map_to_base.transform.rotation.w;

    pose_pub.publish(robot_pose_msg);

    rate.sleep();
  }
  return 0;
}
