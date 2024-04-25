#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <first_project/LidarFrameConfig.h>

class LidarRemap
{
public:
    LidarRemap() : nh_("~")
    {
        // Initialize the publisher and subscriber
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 1);
        sub_ = nh_.subscribe("/os_cloud_node/points", 1, &LidarRemap::pointCloudCallback, this);

        // Dynamic reconfigure server
        dynamic_reconfigure::Server<first_project::LidarFrameConfig>::CallbackType f;
        f = boost::bind(&LidarRemap::configCallback, this, _1, _2);
        server_.setCallback(f);
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        sensor_msgs::PointCloud2 cloud_modified = *cloud;
        cloud_modified.header.frame_id = frame_id_;
        pub_.publish(cloud_modified);
    }

    void configCallback(first_project::LidarFrameConfig &config, uint32_t level)
    {
        frame_id_ = config.frame_id;
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    dynamic_reconfigure::Server<first_project::LidarFrameConfig> server_;
    std::string frame_id_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_remap");
    LidarRemap lidar_remap;
    ros::spin();
    return 0;
}
