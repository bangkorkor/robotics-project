#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <ros/package.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point
{
    double x;
    double y;
    double z;
};

// Function to read goals from a CSV file
std::vector<move_base_msgs::MoveBaseGoal> readGoalsFromCSV(const std::string file_path)
{
    std::vector<move_base_msgs::MoveBaseGoal> goals;
    std::fstream file;
    file.open(file_path, std::ios::in);

    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return goals;
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string x_str, y_str, w_str;

        if (std::getline(ss, x_str, ',') &&
            std::getline(ss, y_str, ',') &&
            std::getline(ss, w_str, ','))
        {
            move_base_msgs::MoveBaseGoal goal;

            goal.target_pose.pose.position.x = std::stod(x_str);
            goal.target_pose.pose.position.y = std::stod(y_str);
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.w = std::stod(w_str);

            goals.push_back(goal);
        }
    }

    file.close();
    return goals;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_sequence_client");
    ros::NodeHandle nh;

    std::string file_path = ros::package::getPath("second_project") + "/config/waypoints.csv";

    std::vector<move_base_msgs::MoveBaseGoal> goals = readGoalsFromCSV(file_path);

    MoveBaseClient ac("move_base", true);

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server...");
    }
    ROS_INFO("move_base action server connected.");

    for (size_t i = 0; i < goals.size(); ++i)
    {
        move_base_msgs::MoveBaseGoal &goal = goals[i];

        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Goal %zu reached!", i);
        }
        else if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            ROS_WARN("Goal %zu aborted, sending next goal.", i);
        }
        else
        {
            ROS_WARN("Goal %zu did not reach, sending next goal.", i);
        }
    }

    return 0;
}