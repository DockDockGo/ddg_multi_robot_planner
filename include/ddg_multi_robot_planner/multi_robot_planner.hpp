#ifndef MULTI_ROBOT_PLANNER_H
#define MULTI_ROBOT_PLANNER_H

#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
// TODO(@VineetTambe: replace unordered_map with boost:flatmap for efficiency)
#include <unordered_map>

#include "ddg_multi_robot_srvs/srv/get_multi_plan.hpp"
// #include <boost/program_options.hpp>
// #include <boost/tokenizer.hpp>
#include <chrono>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "CBS.h"

using namespace std::chrono_literals;

#define AgentState std::pair<int, int>
#define StatePath std::vector<std::pair<int, int>>
#define PosePath std::vector<geometry_msgs::msg::Pose>

#define AHEAD_TIME 0
#define EPS 0.8
#define WAITSTEP 2

// struct AgentState{
//     int x;
//     int y;
// };

// AgentState GLOBAL_START1(8, 7);
// AgentState GLOBAL_START2(8, 9);
// AgentState GLOBAL_GOAL1(24, 8);
// AgentState GLOBAL_GOAL2(26, 8);

namespace multi_robot_planner {

// Define any other necessary includes

class MultiRobotPlanner : public rclcpp::Node {
  // public functions
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
  bool planPaths(
      // std::vector<std::string> &robot_namespaces,
      std::vector<geometry_msgs::msg::PoseStamped> &robot_start_poses,
      std::vector<geometry_msgs::msg::PoseStamped> &robot_goal_poses,
      std::vector<nav_msgs::msg::Path> &planned_paths);
  void convertPathToPosePath(StatePath &state_path, PosePath &pose_path);
  bool Initialize();
  AgentState coordToCBS(geometry_msgs::msg::Pose robot_pose);
  geometry_msgs::msg::Pose coordToGazebo(AgentState &agent_state);

  void callCBS(std::vector<StatePath> &planned_paths);
  void printStatePath(StatePath agent_path);
  void printPosePath(PosePath robot_path);
  void publishWaypoint(geometry_msgs::msg::Pose &waypoint, int agent_id);
  double twoPoseDist(geometry_msgs::msg::Pose &p1,
                     geometry_msgs::msg::Pose p2) {
    return abs(p1.position.x - p2.position.x) +
           abs(p1.position.y - p2.position.y);
  }
  // Private functions
 private:
  void handleGetMultiPlanServiceRequest(
      const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
          request,
      std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
          response);
  bool createAgentScenarioFile(
      std::vector<geometry_msgs::msg::Pose> &robot_start_poses,
      std::vector<geometry_msgs::msg::Pose> &robot_goal_poses,
      std::string &map_file_name, int &map_height, int &map_width,
      std::string &file_path);
  bool readPlannedPathFromFile(const std::string &filename,
                               std::vector<nav_msgs::msg::Path> &planned_paths);
  void configure();
  void publishPlannedPaths(std::vector<nav_msgs::msg::Path> &planned_paths);
  void publishPlannedPaths(std::vector<std::pair<int, int>> &planned_paths);
  void timer_callback();
  //   void RobotPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  //   void RobotPoseCallback(
  //       const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void RobotPoseCallback();
  void RobotGoalPoseCallback(
      const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  bool updateRobotPlan(std::vector<StatePath> &robot_state_paths);
  void PublishCBSPath(int agent_idx, StatePath &agent_path);
  void PublishSingleMarker(int agent_idx, std::string marker_type);
  void PublishMarker();

  // Public variables
 public:
  int _agentNum = 2;  // number of agents | default 2
  std::vector<std::deque<geometry_msgs::msg::Pose>> robots_paths;
  std::vector<geometry_msgs::msg::Pose> robot_waypoints;
  std::vector<geometry_msgs::msg::Pose> robot_curr_poses;
  // std::unordered_map<std::string, geometry_msgs::msg::PoseStamped>
  //     robot_curr_poses;
  std::vector<geometry_msgs::msg::Pose> robot_start_poses;
  std::vector<geometry_msgs::msg::Pose> robot_goal_poses;

  // next state that has been published
  std::vector<AgentState> agent_curr_states;
  std::vector<AgentState> agent_start_states;
  std::vector<AgentState> agent_goal_states;

  std::vector<std::string> robot_namespaces;

  std::shared_ptr<Instance> instance_ptr;

  // Private variables
 private:
  // ROS2 vars
  rclcpp::Clock::SharedPtr clock_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
      agents_pub_pose;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
      agents_pub_goal_marker;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
      agents_pub_start_marker;

  std::vector<rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr>
      agents_pub_path;
  //   std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr>
  //       agents_sub_pose;

  std::vector<rclcpp::Subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr>
      agents_sub_pose;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr>
      agents_sub_target_goal_pose;
  std::vector<int> at_goal_wait;
  std::vector<bool> trip_directions;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr robot_pose_timer_;

  std::vector<std::pair<int, int>> next_round_goal;
  std::vector<std::pair<int, int>> next_round_start;

  bool planner_initialized = false;
  int path_counter_ = 0;

  std::vector<AgentState> GLOBAL_START;
  std::vector<AgentState> GLOBAL_GOAL;
  //   std::vector<geometry_msgs::msg::Pose> target_pose;
  //   std::vector<geometry_msgs::msg::Pose> GLOBAL_GOAL_WORLD_COORD;
  // test only
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;

  // Map update subscriber
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      map_update_subscriber;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr printPoseSub;
  void printPose(geometry_msgs::msg::PoseStamped::SharedPtr robot_state);

  // Map service client
  rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr get_map_srv_client;
  // Map service request
  rclcpp::Service<ddg_multi_robot_srvs::srv::GetMultiPlan>::SharedPtr
      get_multi_plan_service_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::vector<std::vector<char>> occupancy_map;

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

  // CBS object
  // CBS cbs;
  std::string _agentIdx =
      "";         // customize the indices of the agents (e.g., \"0,1\")
  int _rows = 0;  // number of rows | default 0
  int _cols = 0;  // number of cols | default 0
  int _num_obstacles = 0;   // number of obstacles | default 0
  int _warehouseWidth = 0;  // width of working stations on both sides, for
                            // generating instances | default 0
  int _max_runs = 1;
  bool _use_sipp =
      false;  // using sipp as the single agent solver | default false
  int _display_output_on_screen =
      0;  // screen option (0: none; 1: results; 2:all)

  bool _prioritizingConflicts =
      true;  // conflict priortization. If true, conflictSelection is used as a
             // tie-breaking rule.
  bool _disjointSplitting =
      false;  // disjoint splitting. If true, disjoint splitting is used.
  bool _bypass = true;
  heuristics_type _heuristics =
      heuristics_type::WDG;  // heuristics for the high-level search (Zero,
                             // CG,DG, WDG)| default WDG
  rectangle_strategy _rectangleReasoning =
      rectangle_strategy::GR;  // generalized rectangle reasoning
  corridor_strategy _corridorReasoning = corridor_strategy::GC;
  bool _targetReasoning = true;
  bool _mutexReasoning = false;
  bool _saving_stats = false;
  int _nodeLimit = MAX_NODES;

  double _cutoffTime = 120.0;  // cutoff time in seconds | default 60.0

  double orignal_map_resolution = 0.05;          // every cell is 0.05m
  std::vector<double> origin_ = {-3.96, -3.26};  // origin of the original map
  std::vector<int> original_map_size_ = {443,
                                         149};  // origin of the original map
  std::vector<double> offset_ = {0.2, 0.7};     // origin of the original map
  //   double downsampling_factor = 10.0;            // downsampling factor of
  //   10

  double downsampling_factor = 20.0;  // downsampling factor of 10
  AgentState dummy_state = {0, 0};
};

}  // namespace multi_robot_planner

#endif  // MULTI_ROBOT_PLANNER_H
