#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_broadcaster.h>

class OdomToTfConverter
{
public:
  OdomToTfConverter()
  {
    std::string root_frame_param_name = ros::this_node::getName() + "/root_frame";
    std::string child_frame_param_name = ros::this_node::getName() + "/child_frame";

    n.getParam(root_frame_param_name, root_frame);
    n.getParam(child_frame_param_name, child_frame);

    odom_sub = n.subscribe("/input_odom", 1, &OdomToTfConverter::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
  {
    tf::Transform odom_trans;

    odom_trans.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    tf::Quaternion q(msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z,
                     msg->pose.pose.orientation.w);
    odom_trans.setRotation(q);
    ros::Time time = msg->header.stamp;

    broadcaster.sendTransform(tf::StampedTransform(odom_trans, time, root_frame, child_frame));
  }

private:
  ros::NodeHandle n;
  tf::TransformBroadcaster broadcaster;
  ros::Subscriber odom_sub;
  std::string root_frame;
  std::string child_frame;
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_to_tf2");
  OdomToTfConverter OdomToTfConverter;
  ros::spin();
  return 0;
}