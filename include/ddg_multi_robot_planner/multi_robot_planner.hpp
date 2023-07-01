#ifndef MULTI_ROBOT_PLANNER_H
#define MULTI_ROBOT_PLANNER_H

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// TODO(@VineetTambe: replace unordered_map with boost:flatmap for efficiency)
#include "ddg_multi_robot_srvs/srv/get_multi_plan.hpp"
#include <unordered_map>
// #include <boost/program_options.hpp>
// #include <boost/tokenizer.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <CBS.h>

namespace multi_robot_planner
{

    // Define any other necessary includes

    class MultiRobotPlanner : public rclcpp::Node
    {
    public:
        // Constructor
        MultiRobotPlanner();

        // Destructor
        ~MultiRobotPlanner();

        void updateNamespaces(std::vector<std::string> &robot_namespaces);
        void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        void writeMapToFile(std::vector<std::vector<char>> &occupancy_map,
                            const std::string &file_path);
        void parseMap(std::vector<std::vector<char>> &occupancy_map,
                      const nav_msgs::msg::OccupancyGrid &map);
        void printMap(std::vector<std::vector<char>> &map);
        void handle_response(nav_msgs::srv::GetMap::Response::SharedPtr response);
        bool updateMap();
        bool updateRobotPoses();
        bool getRobotPose(std::string &robot_namespace,
                          geometry_msgs::msg::PoseStamped &robot_pose);
        // Plan paths for all robots
        bool planPaths(std::vector<std::string> &robot_namespaces,
                       std::vector<geometry_msgs::msg::Pose> &robot_start_poses,
                       std::vector<geometry_msgs::msg::Pose> &robot_goal_poses,
                       std::vector<nav_msgs::msg::Path> &planned_paths);

        // TODO(@VineetTambe: updated the data structure of the argument)
        std::vector<geometry_msgs::msg::PoseStamped>
        convertPathToPoseStamped(std::vector<std::pair<int, int>> path);

    private:
        // ROS2 vars

        // Map update subscriber
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
            map_update_subscriber;
        // Map service client
        rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_srv_client;
        // Map service request
        rclcpp::Service<ddg_multi_robot_srvs::srv::GetMultiPlan>::SharedPtr
            get_multi_plan_service_;

        void handleGetMultiPlanServiceRequest(
            const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
                request,
            std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
                response);

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        std::vector<std::vector<char>> occupancy_map;
        std::vector<std::string> robot_namespaces;
        std::unordered_map<std::string, geometry_msgs::msg::PoseStamped>
            robot_curr_poses;

        // TODO CBS object
        std::string _map_file_name =
            "/home/admin/ddg_mfi/mp_400_ws/src/ddg_multi_robot_planner/maps/"
            "svd_demo-parsed-map.txt";
        std::string _agent_scen_file_name =
            "/home/admin/ddg_mfi/mp_400_ws/src/ddg_multi_robot_planner/maps/"
            "svd_demo-scen.txt";
        std::string _planned_path_file_name =
            "/home/admin/ddg_mfi/mp_400_ws/src/ddg_multi_robot_planner/maps/"
            "planned-paths.txt";
        int _map_height;
        int _map_width;
        bool createAgentScenarioFile(
            std::vector<geometry_msgs::msg::Pose> &robot_start_poses,
            std::vector<geometry_msgs::msg::Pose> &robot_goal_poses,
            std::string &map_file_name, int &map_height, int &map_width,
            std::string &file_path);

        bool readPlannedPathFromFile(const std::string &filename,
                                     std::vector<nav_msgs::msg::Path> &planned_paths);

        // CBS object
        // CBS cbs;
        int _agentNum = 5;          // number of agents | default 5
        std::string _agentIdx = ""; // customize the indices of the agents (e.g., \"0,1\")
        int _rows = 0;              // number of rows | default 0
        int _cols = 0;              // number of cols | default 0
        int _num_obstacles = 0;     // number of obstacles | default 0
        int _warehouseWidth = 0;    // width of working stations on both sides, for generating instances | default 0
        int _max_runs = 1;
        bool _use_sipp = false;            // using sipp as the single agent solver | default false
        int _display_output_on_screen = 0; // screen option (0: none; 1: results; 2:all)

        bool _prioritizingConflicts = true; // conflict priortization. If true, conflictSelection is used as a tie-breaking rule.
        bool _disjointSplitting = false;    // disjoint splitting. If true, disjoint splitting is used.
        bool _bypass = true;
        heuristics_type _heuristics = heuristics_type::WDG;              // heuristics for the high-level search (Zero, CG,DG, WDG)| default WDG
        rectangle_strategy _rectangleReasoning = rectangle_strategy::GR; // generalized rectangle reasoning
        corridor_strategy _corridorReasoning = corridor_strategy::GC;
        bool _targetReasoning = true;
        bool _mutexReasoning = false;
        bool _saving_stats = false;
        int _nodeLimit = MAX_NODES;

        double _cutoffTime = 60.0; // cutoff time in seconds | default 60.0
        void configure();
    };

} // namespace multi_robot_planner

#endif // MULTI_ROBOT_PLANNER_H
