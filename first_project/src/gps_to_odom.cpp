#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>

ros::Publisher odom_pub; // Global publisher object

const double a = 6378137; //[m] semi major axis
const double b = 6356752; //[m] semi minor axis



// Convert degrees to radians
inline double deg2rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

// Convert LLA to ECEF coordinates
void LLAToECEF(double latitude, double longitude, double altitude, double &x, double &y, double &z)
{
    latitude = deg2rad(latitude);
    longitude = deg2rad(longitude);
    const double E2 = 1 - (b*b)/(a*a); // Square of eccentricity
    double N = a / sqrt(1 - E2 * sin(latitude) * sin(latitude));

    x = (N + altitude) * cos(latitude) * cos(longitude);
    y = (N + altitude) * cos(latitude) * sin(longitude);
    z = ((1 - E2) * N + altitude) * sin(latitude);
}

// ECEF to ENU conversion
void ECEFToENU(double x, double y, double z, double latRef, double lonRef, double altRef, double &e, double &n, double &u)
{
    double xRef, yRef, zRef;
    LLAToECEF(latRef, lonRef, altRef, xRef, yRef, zRef);

    double dx = x - xRef;
    double dy = y - yRef;
    double dz = z - zRef;

    latRef = deg2rad(latRef);
    lonRef = deg2rad(lonRef);

    e = -sin(lonRef) * dx + cos(lonRef) * dy;
    n = -sin(latRef) * cos(lonRef) * dx - sin(latRef) * sin(lonRef) * dy + cos(latRef) * dz;
    u = cos(latRef) * cos(lonRef) * dx + cos(latRef) * sin(lonRef) * dy + sin(latRef) * dz;
}

// GPS callback function
void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    double e, n, u;
    // Convert GPS coordinates to ECEF
    double x, y, z;
    LLAToECEF(msg->latitude, msg->longitude, msg->altitude, x, y, z);

    // Assume reference coordinates (lat_r, lon_r, alt_r) are defined globally or passed as parameters
    double lat_r = 0.0; // Example reference latitude
    double lon_r = 0.0; // Example reference longitude
    double alt_r = 0.0; // Example reference altitude

    // Convert ECEF coordinates to ENU
    ECEFToENU(x, y, z, lat_r, lon_r, alt_r, e, n, u);

    // Create an odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "enu";

    // Set position in the odometry message
    odom_msg.pose.pose.position.x = e;
    odom_msg.pose.pose.position.y = n;
    odom_msg.pose.pose.position.z = u;

    // Publish the message
    odom_pub.publish(odom_msg);

    ROS_INFO("Published ENU Coordinates as Odometry: E: %f, N: %f, U: %f", e, n, u);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gps_to_odom");
    ros::NodeHandle nh;

    // Initialize the publisher
    odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 50); // Topic name and queue size

    // Subscribe to GPS data
    ros::Subscriber gps_sub = nh.subscribe("fix", 1000, gpsCallback);

    ros::spin();
    return 0;
}
