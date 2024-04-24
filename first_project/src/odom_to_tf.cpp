#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTFConverter
{
public:
    OdomToTFConverter() : nh_("~")
    {
        // Read parameters
        nh_.param<std::string>("root_frame", root_frame_, "world");
        nh_.param<std::string>("child_frame", child_frame_, "wheel_odom");

        // Subscribe to odometry message
        odom_sub_ = nh_.subscribe("input_odom", 1, &OdomToTFConverter::odomCallback, this);

        // Initialize TF broadcaster
        tf_broadcaster_ = new tf2_ros::TransformBroadcaster();
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // Create transform message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = root_frame_;
        transformStamped.child_frame_id = child_frame_;
        transformStamped.transform.translation.x = msg->pose.pose.position.x;
        transformStamped.transform.translation.y = msg->pose.pose.position.y;
        transformStamped.transform.translation.z = msg->pose.pose.position.z;
        transformStamped.transform.rotation = msg->pose.pose.orientation;

        // Broadcast transform
        tf_broadcaster_->sendTransform(transformStamped);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster *tf_broadcaster_;
    std::string root_frame_;
    std::string child_frame_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_to_tf");
    OdomToTFConverter converter;
    ros::spin();
    return 0;
}
