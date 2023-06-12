#ifndef MULTI_ROBOT_PLANNER_H
#define MULTI_ROBOT_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
#include <fstream>
#include <string>
// TODO(@VineetTambe: replace unordered_map with boost:flatmap for efficiency)
#include <unordered_map>
// #include "ddg_multi_robot_msgs/srv/GetMultiPlan.hpp"
// /home/vineet/ddg_mfi/mp_400_ws/src/ddg_multi_robot_planner/srv/GetMultiPlan.srv

namespace multi_robot_planner
{

    // Define any other necessary includes

    class MultiRobotPlanner : public rclcpp::Node
    {
    private:
        // ROS2 vars
        // rclcpp::Node::SharedPtr multi_robot_planner_node;
        // Map update subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_update_subscriber;
        // Map service client
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_srv_client;

        // void handleServiceRequest(
        //     const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        //     std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);

        // rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::vector<std::vector<char>> occupancy_map;
        std::vector<std::string> robot_namespaces;
        std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> robot_curr_poses;

        // TODO CBS object

    public:
        // Constructor
        MultiRobotPlanner();

        // Destructor
        ~MultiRobotPlanner();

        void updateNamespaces(std::vector<std::string> &robot_namespaces);
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void writeMapToFile(std::vector<std::vector<char>> &occupancy_map, const std::string &file_path);
        void parseMap(std::vector<std::vector<char>> &occupancy_map, const nav_msgs::msg::OccupancyGrid &map);
        void printMap(std::vector<std::vector<char>> &map);
        void handle_response(nav_msgs::srv::GetMap::Response::SharedPtr response);
        bool updateMap();
        bool updateRobotPoses();
        bool getRobotPose(std::string &robot_namespace, geometry_msgs::msg::PoseStamped &robot_pose);
        // Plan paths for all robots
        std::unordered_map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> planPaths(std::unordered_map<std::string, geometry_msgs::msg::PoseStamped> robot_goal_poses);

        // TODO(@VineetTambe: updated the data structure of the argument)
        std::vector<geometry_msgs::msg::PoseStamped> convertPathToPoseStamped(std::vector<std::pair<int, int>> path);
    };

} // namespace multi_robot_planner

#endif // MULTI_ROBOT_PLANNER_H
