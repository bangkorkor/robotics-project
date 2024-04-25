#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomToTfConverter
{
public:
    OdomToTfConverter()
    {
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");

        // Get parameters for root and child frames
        private_nh.param<std::string>("root_frame", root_frame, "world");
        private_nh.param<std::string>("child_frame", child_frame, "base_link");

        // Subscriber to Odometry messages, topic is remapped in the launch file
        odom_sub = nh.subscribe("input_odom", 10, &OdomToTfConverter::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = msg->header.stamp;
        odom_trans.header.frame_id = root_frame;
        odom_trans.child_frame_id = child_frame;

        odom_trans.transform.translation.x = msg->pose.pose.position.x;
        odom_trans.transform.translation.y = msg->pose.pose.position.y;
        odom_trans.transform.translation.z = msg->pose.pose.position.z;
        odom_trans.transform.rotation = msg->pose.pose.orientation;

        broadcaster.sendTransform(odom_trans);
    }

private:
    ros::Subscriber odom_sub;
    tf::TransformBroadcaster broadcaster;
    std::string root_frame;
    std::string child_frame;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_tf");
    OdomToTfConverter converter;
    ros::spin();
    return 0;
}
