#include "multi_robot_planner.hpp"

#include <vector>

#include "CBS.h"
// #include <CBS.h>
namespace multi_robot_planner {
MultiRobotPlanner::MultiRobotPlanner() : Node("multi_robot_planner_node") {
  clock_ = get_clock();

  timer_ = this->create_wall_timer(
      50ms, std::bind(&MultiRobotPlanner::timer_callback, this));  // 2Hz

  robot_pose_timer_ = this->create_wall_timer(
      50ms, std::bind(&MultiRobotPlanner::RobotPoseCallback, this));  // 20Hz

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "Start multi robot planner!");

  printPoseSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/test_pose", 10,
      std::bind(&MultiRobotPlanner::printPose, this, std::placeholders::_1));

  for (int i = 0; i < _agentNum; i++) {
    std::string tmp_robot_name = "robot" + std::to_string(i);
    robot_namespaces.push_back(tmp_robot_name);

    std::string tmp_goal_topic_name = "/" + tmp_robot_name + "/goal_pose";
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "The topic to publish goal pose for agent is: " << tmp_goal_topic_name);
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
        tmp_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            tmp_goal_topic_name, 10);
    agents_pub_pose.push_back(tmp_publisher);

    std::string tmp_path_topic_name = "/" + tmp_robot_name + "/cbs_path";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "The topic to publish computed path for agent is: "
                           << tmp_path_topic_name);
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tmp_path_publisher =
        this->create_publisher<nav_msgs::msg::Path>(tmp_path_topic_name, 10);
    agents_pub_path.push_back(tmp_path_publisher);

    tmp_path_topic_name = "/" + tmp_robot_name + "/start_marker";
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "The topic to publish start markers for RVIZ: " << tmp_path_topic_name);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        tmp_start_marker_publisher =
            this->create_publisher<visualization_msgs::msg::Marker>(
                tmp_path_topic_name, 10);
    agents_pub_start_marker.push_back(tmp_start_marker_publisher);

    tmp_path_topic_name = "/" + tmp_robot_name + "/goal_marker";
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "The topic to publish goal markers for RVIZ: " << tmp_path_topic_name);
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
        tmp_goal_marker_publisher =
            this->create_publisher<visualization_msgs::msg::Marker>(
                tmp_path_topic_name, 10);
    agents_pub_goal_marker.push_back(tmp_goal_marker_publisher);

    std::string tmp_pose_topic_name = "/" + tmp_robot_name + "/map_pose";
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "The topic to subscribe for agent odom is: " << tmp_pose_topic_name);

    // TODO @VineetTambe:  Convert this to a service
    std::string tmp_agent_goal_pose_sub_name =
        "/" + tmp_robot_name + "/cbs_path/goal_pose";
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "The topic to subscribe agent target goal pose is: "
                           << tmp_agent_goal_pose_sub_name);
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
        tmp_target_goal_pose_sub =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                tmp_agent_goal_pose_sub_name, 10,
                std::bind(&MultiRobotPlanner::RobotGoalPoseCallback, this,
                          std::placeholders::_1));
    agents_sub_target_goal_pose.push_back(tmp_target_goal_pose_sub);
  }

  Initialize();

  printf("Started multi robot planner!\n");

  // // Initialize your variables and subscribers here
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // get_map_srv_client =
  //     this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // // Not implemented properly
  // map_update_subscriber =
  //     this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  //         "/map", 10,
  //         std::bind(&MultiRobotPlanner::mapCallback, this,
  //                   std::placeholders::_1));

  // ros2 service call to call planPaths
  get_multi_plan_service_ =
      create_service<ddg_multi_robot_srvs::srv::GetMultiPlan>(
          "/multi_robot_planner/get_plan",
          [this](
              const std::shared_ptr<
                  ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
                  request,
              std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
                  response) {
            handleGetMultiPlanServiceRequest(request, response);
          });
}

bool MultiRobotPlanner::Initialize() {
  geometry_msgs::msg::Pose tempPose;

  // instance_ptr = std::make_shared<Instance>(
  //     "./src/ddg_multi_robot_planner/maps/svddemo-14-44-2.map");

  instance_ptr = std::make_shared<Instance>(
      "./src/ddg_multi_robot_planner/maps/downsampled-map/res20/"
      "svd_demo-downsampled.map");

  AgentState tmp_state;
  tempPose.position.x = -1.6;
  tempPose.position.y = 1.4;

  robot_curr_poses.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_start_states.push_back(tmp_state);
  tmp_state = coordToCBS(tempPose);
  GLOBAL_START.push_back(tmp_state);
  tempPose.position.x = -1.6;
  tempPose.position.y = -2.2;
  robot_curr_poses.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_start_states.push_back(tmp_state);

  tmp_state = coordToCBS(tempPose);
  GLOBAL_START.push_back(tmp_state);

  tempPose.position.x = 8.0;
  tempPose.position.y = -2.0;
  tmp_state = coordToCBS(tempPose);
  GLOBAL_GOAL.push_back(tmp_state);
  next_round_goal.push_back(tmp_state);

  tempPose.position.x = 8.0;
  tempPose.position.y = -0.75;

  tmp_state = coordToCBS(tempPose);
  GLOBAL_GOAL.push_back(tmp_state);
  next_round_goal.push_back(tmp_state);

  robots_paths.resize(_agentNum);
  robot_waypoints.resize(_agentNum);
  next_round_start.resize(_agentNum);

  instance_ptr->printMap();

  for (int i = 0; i < _agentNum; i++) {
    trip_directions.push_back(false);
    at_goal_wait.push_back(WAITSTEP);
  }
  planner_initialized = true;
  // goal_received = true;
}

void MultiRobotPlanner::resizeStateVars(int resize_size) {
  robot_curr_poses.resize(resize_size);
  agent_start_states.resize(resize_size);
  agent_goal_states.resize(resize_size);
  next_round_goal.resize(resize_size);
  next_round_start.resize(resize_size);
  GLOBAL_GOAL.resize(resize_size);
  GLOBAL_START.resize(resize_size);
  robot_waypoints.resize(resize_size);
}

void MultiRobotPlanner::timer_callback() {
  if (!planner_initialized || !goal_received) {
    return;
  }
  std::vector<StatePath> planned_paths;

  // int agent_finish_count = 0;
  for (int i = 0; i < _agentNum; i++) {
    if (robots_paths[i].empty()) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Agent %d reached its goal", i);
      if (at_goal_wait[i] > 0) {
        at_goal_wait[i]--;
        continue;
      } else {
        at_goal_wait[i] = WAITSTEP;
      }
      for (int agent_idx = 0; agent_idx < _agentNum; agent_idx++) {
        geometry_msgs::msg::Pose next_tmp_pose = robot_curr_poses[agent_idx];
        std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
        if (agent_idx == 1) {  // TODO make this more general - currently only
                               // valid for 2 robots
          instance_ptr->validCoord(next_tmp_state, next_round_start[0]);
        }
        next_round_start[agent_idx] = next_tmp_state;
      }
      next_round_goal[i] = GLOBAL_GOAL[i];
      GLOBAL_START[i] = next_round_start[i];
    } else {
      geometry_msgs::msg::Pose next_tmp_pose = robots_paths[i][AHEAD_TIME];
      std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
      next_round_start[i] = next_tmp_state;
    }
  }
  instance_ptr->updateAgents(_agentNum, next_round_start, next_round_goal);
  callCBS(planned_paths);
  bool ret_success = updateRobotPlan(planned_paths);
  if (ret_success) {
    PublishMarker();
    for (unsigned int i = 0; i < _agentNum; i++) {
      PublishCBSPath(i, planned_paths[i]);
    }
  }
}

void MultiRobotPlanner::PublishMarker() {
  for (int i = 0; i < _agentNum; i++) {
    PublishSingleMarker(i, "start");
    PublishSingleMarker(i, "goal");
  }
}

void MultiRobotPlanner::PublishSingleMarker(int agent_idx,
                                            std::string marker_type) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = clock_->now();
  marker.ns = marker_type;
  marker.id = 0;
  marker.action = visualization_msgs::msg::Marker::ADD;
  // marker.pose.position.y = 1;
  // marker.pose.position.z = 1;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 0.1;
  marker.color.a = 0.3;  // Don't forget to set the alpha!

  if (marker_type == "start") {
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    geometry_msgs::msg::Pose tmp_robot_pose =
        coordToGazebo(GLOBAL_START[agent_idx]);
    tmp_robot_pose.orientation.w = 1.0;
    marker.pose = tmp_robot_pose;
    agents_pub_start_marker[agent_idx]->publish(marker);
  } else if (marker_type == "goal") {
    marker.type = visualization_msgs::msg::Marker::CUBE;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    geometry_msgs::msg::Pose tmp_robot_pose =
        coordToGazebo(GLOBAL_GOAL[agent_idx]);
    tmp_robot_pose.orientation.w = 1.0;
    marker.pose = tmp_robot_pose;
    agents_pub_goal_marker[agent_idx]->publish(marker);
  }
}

void MultiRobotPlanner::PublishCBSPath(int agent_idx, StatePath &agent_path) {
  nav_msgs::msg::Path pose_path;
  pose_path.header.frame_id = "map";
  pose_path.header.stamp = clock_->now();
  for (auto entry : agent_path) {
    geometry_msgs::msg::PoseStamped tmp_pose;
    geometry_msgs::msg::Pose tmp_tmp_pose;
    tmp_tmp_pose = coordToGazebo(entry);
    tmp_pose.pose = tmp_tmp_pose;
    tmp_pose.header.frame_id = "map";
    tmp_pose.header.stamp = clock_->now();
    pose_path.poses.push_back(tmp_pose);
  }
  agents_pub_path[agent_idx]->publish(pose_path);
}

void MultiRobotPlanner::printStatePath(StatePath agent_path) {
  for (auto entry : agent_path) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entry is: (%d, %d)", entry.first,
                entry.second);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End of the path\n\n");
}

void MultiRobotPlanner::printPosePath(PosePath robot_path) {
  for (auto entry : robot_path) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose is: (%f, %f)",
                entry.position.x, entry.position.y);
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "End of the path\n\n");
}

void MultiRobotPlanner::RobotGoalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::string agent_name = msg->header.frame_id;
  unsigned int agent_idx = 0;
  bool correct_msg = false;
  try {
    // TODO add check for missing frame id
    for (unsigned int i = 0; i < _agentNum; i++) {
      if (agent_name.find(robot_namespaces[i]) != std::string::npos) {
        agent_idx = i;
        correct_msg = true;
        break;
      }
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing Goal Pose message: %s",
                 ex.what());
  }
  if (correct_msg) {
    RCLCPP_INFO(this->get_logger(), "Got goal for agent: %s", agent_name);
    GLOBAL_GOAL[agent_idx] = coordToCBS(msg->pose);
    // GLOBAL_GOAL_WORLD_COORD[agent_idx] = msg->pose;
    for (int i = 0; i < _agentNum; i++) {
      robots_paths[i].clear();
    }
    if (agent_idx == 1) goal_received = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing Goal Pose message: %s",
                 agent_name);
  }
}

void MultiRobotPlanner::RobotPoseCallback() {
  if (!planner_initialized) {
    return;
  }
  vector<geometry_msgs::msg::PoseStamped> robot_poses(2);
  try {
    for (unsigned int agent_idx = 0; agent_idx < _agentNum; agent_idx++) {
      getRobotPose(robot_namespaces[agent_idx], robot_poses[agent_idx]);
      robot_curr_poses[agent_idx].position.x =
          robot_poses[agent_idx].pose.position.x;
      robot_curr_poses[agent_idx].position.y =
          robot_poses[agent_idx].pose.position.y;
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing tf tree: %s", ex.what());
  }
  if (!goal_received) {
    return;
  }

  for (unsigned int agent_idx = 0; agent_idx < _agentNum; agent_idx++) {
    if (robots_paths[agent_idx].empty()) continue;

    if (twoPoseDist(robot_curr_poses[agent_idx], robot_waypoints[agent_idx]) <
        EPS) {
      robot_waypoints[agent_idx] = robots_paths[agent_idx][0];
      robots_paths[agent_idx].pop_front();
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "Publish waypoint for agent %d, at (%f, %f)", agent_idx,
                  robot_waypoints[agent_idx].position.x,
                  robot_waypoints[agent_idx].position.y);
    }
    publishWaypoint(robot_waypoints[agent_idx], agent_idx);
  }
}

void MultiRobotPlanner::publishPlannedPaths(
    std::vector<std::pair<int, int>> &planned_paths) {
  for (unsigned int i = 0; i < planned_paths.size(); i++) {
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = clock_->now();
    goal_pose.pose.position.x = planned_paths[i].first * 1.0;
    goal_pose.pose.position.y = planned_paths[i].second * 1.0;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.w = 0.0;
    agents_pub_pose[i]->publish(goal_pose);
    // RCLCPP_DEBUG(get_logger(), "Publish message");
    // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Publish message");
  }
}

void MultiRobotPlanner::publishWaypoint(geometry_msgs::msg::Pose &waypoint,
                                        int agent_id) {
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = clock_->now();
  goal_pose.pose = waypoint;
  agents_pub_pose[agent_id]->publish(goal_pose);
}

AgentState MultiRobotPlanner::coordToCBS(geometry_msgs::msg::Pose robot_pose) {
  AgentState robot_state;

  // worldToMap

  robot_state.first =

      (int)round((original_map_size_[1] -
                  1 * round((robot_pose.position.y - offset_[1] - origin_[1]) /
                            (orignal_map_resolution))) /
                 downsampling_factor) -
      1;

  robot_state.second =
      (int)round(round((robot_pose.position.x - offset_[0] - origin_[0]) /
                       orignal_map_resolution) /
                 downsampling_factor);

  try {
    instance_ptr->validCoord(robot_state, dummy_state);
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing robot pose: %s",
                 ex.what());
  }
  return robot_state;
}

void MultiRobotPlanner::printPose(
    geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  AgentState robot_state = coordToCBS(msg->pose);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "calculated pose is: (%d, %d)",
              robot_state.first, robot_state.second);

  auto pose = coordToGazebo(robot_state);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "decalculated pose is: (%f, %f)",
              pose.position.x, pose.position.y);

  instance_ptr->printAgentTargets(robot_state);
}

geometry_msgs::msg::Pose MultiRobotPlanner::coordToGazebo(
    AgentState &agent_state) {
  geometry_msgs::msg::Pose agent_pose;
  agent_pose.position.x =
      (orignal_map_resolution * downsampling_factor) * (agent_state.second) +
      origin_[0] + offset_[0];

  agent_pose.position.y = -1.0 *
                              ((agent_state.first + 1) * downsampling_factor -
                               original_map_size_[1]) *
                              orignal_map_resolution +
                          origin_[1] + offset_[1];
  return agent_pose;
}

MultiRobotPlanner::~MultiRobotPlanner() {
  // Perform any cleanup or resource deallocation here
}

void MultiRobotPlanner::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // TODO (@VineetTambe: change this to an actual mpa update this function)
  // Print out the width and height of the occupancy grid
  std::cout << "Received map with width=" << msg->info.width
            << " and height=" << msg->info.height << std::endl;
}

void MultiRobotPlanner::writeMapToFile(
    std::vector<std::vector<char>> &occupancy_map,
    const std::string &file_path) {
  std::ofstream file(file_path);
  if (!file.is_open()) {
    // RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Failed to open file
    // for writing");
    return;
  }
  file << "type octile" << std::endl;
  file << "height " << (int)occupancy_map.size() << std::endl;
  file << "width " << (int)occupancy_map[0].size() << std::endl;
  file << "map" << std::endl;

  // Write map data to file
  for (int i = 0; i < (int)occupancy_map.size(); ++i) {
    for (int j = 0; j < (int)occupancy_map[0].size(); ++j) {
      // int index = i * map.info.width + j;
      file << static_cast<char>(occupancy_map[i][j]);
    }
    file << std::endl;
  }

  file.close();

  RCLCPP_INFO(rclcpp::get_logger("service_client"), "Map written to file: %s",
              file_path.c_str());
}

void MultiRobotPlanner::parseMap(std::vector<std::vector<char>> &occupancy_map,
                                 const nav_msgs::msg::OccupancyGrid &map) {
  for (int i = 0; i < (int)map.info.height; i++) {
    for (int j = 0; j < (int)map.info.width; j++) {
      if (map.data[i * (int)map.info.width + j] > 50) {
        // RCLCPP_INFO(rclcpp::get_logger("service_client"), "HERE");
        occupancy_map[i][j] = '@';
      } else {
        occupancy_map[i][j] = '.';
      }
    }
  }
  // TODO change file naming method
  writeMapToFile(occupancy_map, _map_file_name);
}

void MultiRobotPlanner::printMap(std::vector<std::vector<char>> &map) {
  for (int i = 0; i < (int)map.size(); i++) {
    for (int j = 0; j < (int)map[0].size(); j++) {
      std::cout << map[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

void MultiRobotPlanner::handle_response(
    nav_msgs::srv::GetMap::Response::SharedPtr response) {
  std::vector<std::vector<char>> occupancy_map;
  RCLCPP_INFO(this->get_logger(), "inside handle map update response ...");

  try {
    occupancy_map.resize(response->map.info.height,
                         std::vector<char>(response->map.info.width, 0));
    _map_height = response->map.info.height;
    _map_width = response->map.info.width;
    // occupancy_map.resize(response->map.info.height,
    // std::vector<char>(response->map.info.width, 0));
    parseMap(occupancy_map, response->map);

    RCLCPP_INFO(this->get_logger(), "Successfully received map");

    // TODO use timers and return a flag var here

    std::vector<geometry_msgs::msg::Pose> robot_start_poses;
    std::vector<geometry_msgs::msg::Pose> robot_goal_poses;
    // test code
    geometry_msgs::msg::Pose tempPose;
    tempPose.position.x = 1.0;
    tempPose.position.y = 2.0;
    robot_start_poses.push_back(tempPose);
    tempPose.position.x = 2.0;
    tempPose.position.y = 3.0;
    robot_start_poses.push_back(tempPose);
    tempPose.position.x = 4.0;
    tempPose.position.y = 5.0;
    robot_start_poses.push_back(tempPose);
    tempPose.position.x = 6.0;
    tempPose.position.y = 7.0;
    robot_start_poses.push_back(tempPose);
    tempPose.position.x = 8.0;
    tempPose.position.y = 9.0;
    robot_start_poses.push_back(tempPose);

    tempPose.position.x = 8.0;
    tempPose.position.y = 9.0;
    robot_goal_poses.push_back(tempPose);
    tempPose.position.x = 10.0;
    tempPose.position.y = 11.0;
    robot_goal_poses.push_back(tempPose);
    tempPose.position.x = 12.0;
    tempPose.position.y = 13.0;
    robot_goal_poses.push_back(tempPose);
    tempPose.position.x = 14.0;
    tempPose.position.y = 15.0;
    robot_goal_poses.push_back(tempPose);
    tempPose.position.x = 16.0;
    tempPose.position.y = 17.0;
    robot_goal_poses.push_back(tempPose);

    bool res = createAgentScenarioFile(robot_start_poses, robot_goal_poses,
                                       _map_file_name, _map_height, _map_width,
                                       _agent_scen_file_name);

    // TODO add some sort of mutex here
    if (!res) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create scenario file");
      // return false;
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Scenario file created running the cbs solver");

    //////////////////////////////////////////////////////////////////////
    /// run
    //////////////////////////////////////////////////////////////////////

    Instance instance(_map_file_name, _agent_scen_file_name, _agentNum,
                      _agentIdx, _rows, _cols, _num_obstacles, _warehouseWidth);

    CBS cbs(instance, _use_sipp, _display_output_on_screen);
    cbs.setPrioritizeConflicts(_prioritizingConflicts);
    cbs.setDisjointSplitting(_disjointSplitting);
    cbs.setBypass(_bypass);
    cbs.setRectangleReasoning(_rectangleReasoning);
    cbs.setCorridorReasoning(_corridorReasoning);
    cbs.setHeuristicType(_heuristics);
    cbs.setTargetReasoning(_targetReasoning);
    cbs.setMutexReasoning(_mutexReasoning);
    cbs.setSavingStats(_saving_stats);
    cbs.setNodeLimit(_nodeLimit);

    double runtime = 0;
    int min_f_val = 0;
    for (int i = 0; i < _max_runs; i++) {
      cbs.clear();
      cbs.solve(_cutoffTime, min_f_val);
      runtime += cbs.runtime;
      if (cbs.solution_found) break;
      min_f_val = (int)cbs.min_f_val;
      cbs.randomRoot = true;
    }
    cbs.runtime = runtime;
    if (cbs.solution_found) {
      RCLCPP_INFO(this->get_logger(),
                  "Succesfully found a path! Saving it to file");

      // TODO Replace this with a function to get paths from cbs
      cbs.savePaths(_planned_path_file_name);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to find a path!");
    }
    cbs.clearSearchEngines();

    // load paths and return them
    std::vector<nav_msgs::msg::Path> planned_paths;
    if (readPlannedPathFromFile(_planned_path_file_name, planned_paths)) {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully read planned path from file");
      RCLCPP_INFO(this->get_logger(), "Yay!");
      // TODO publish the planned paths
      // return;
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to read planned path from file!");
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing occupancy grid: %s",
                 ex.what());
  }
}

void MultiRobotPlanner::callCBS(std::vector<StatePath> &planned_paths) {
  RCLCPP_INFO(this->get_logger(), "Running the cbs solver [planned paths]");
  try {
    //////////////////////////////////////////////////////////////////////
    /// run
    //////////////////////////////////////////////////////////////////////
    CBS cbs(*instance_ptr, _use_sipp, 0);
    cbs.setPrioritizeConflicts(_prioritizingConflicts);
    cbs.setDisjointSplitting(_disjointSplitting);
    cbs.setBypass(_bypass);
    cbs.setRectangleReasoning(_rectangleReasoning);
    cbs.setCorridorReasoning(_corridorReasoning);
    cbs.setHeuristicType(_heuristics);
    cbs.setTargetReasoning(_targetReasoning);
    cbs.setMutexReasoning(_mutexReasoning);
    cbs.setSavingStats(_saving_stats);
    cbs.setNodeLimit(_nodeLimit);

    double runtime = 0;
    int min_f_val = 0;
    for (int i = 0; i < _max_runs; i++) {
      cbs.clear();
      cbs.solve(_cutoffTime, min_f_val);
      runtime += cbs.runtime;
      if (cbs.solution_found) break;
      min_f_val = (int)cbs.min_f_val;
      cbs.randomRoot = true;
    }
    cbs.runtime = runtime;

    //////////////////////////////////////////////////////////////////////
    /// write results to files
    //////////////////////////////////////////////////////////////////////
    if (cbs.solution_found) {
      //   RCLCPP_INFO(this->get_logger(), "Succesfully found a path! Saving
      //   it to file");

      // TODO Replace this with a function to get paths from cbs
      cbs.getSolvedPaths(planned_paths);
      cbs.clearSearchEngines();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to find a path!");
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in cbs solve: %s", ex.what());
  }
}

// Function to read line from a text file and store coordinates in an array
bool MultiRobotPlanner::readPlannedPathFromFile(
    const std::string &filename,
    std::vector<nav_msgs::msg::Path> &planned_paths) {
  std::ifstream file(filename);
  std::string line;

  auto time_stamp = this->get_clock()->now();

  if (file.is_open()) {
    while (std::getline(file, line)) {
      std::string delimiter = ")->";
      size_t pos = 0;
      std::string token;

      // Read the agent number
      int agen_num = line[6] - 48;
      std::cout << agen_num << std::endl;
      line.erase(0, 7);

      nav_msgs::msg::Path path;

      path.header.frame_id = "map";
      path.header.stamp = time_stamp;

      while ((pos = line.find(delimiter)) != std::string::npos) {
        // This can be further optimized by moving this outside the
        // loop and not creating a new object everytime
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;

        token = line.substr(0, pos);
        // Remove any non-digit characters except commas
        for (char &c : token) {
          if (!isdigit(c) && c != ',') c = ' ';
        }

        // Use string stream to extract the numbers separated by commas
        std::stringstream ss(token);
        std::string number;
        std::getline(ss, number, ',');
        pose.pose.position.x = std::stoi(number);
        getline(ss, number, ',');
        pose.pose.position.y = std::stoi(number);
        line.erase(0, pos + delimiter.length());

        pose.pose.position.z = 0;
        path.poses.push_back(pose);
      }
      planned_paths.push_back(path);
    }
    file.close();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s",
                 filename.c_str());
    return false;
  }

  return true;
}

bool MultiRobotPlanner::updateMap() {
  // Wait for the service to be available
  while (!get_map_srv_client->wait_for_service(std::chrono::seconds(1))) {
    // TODO add a counter to prevent this loop from blocking the thread.
    if (!rclcpp::ok()) {
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    return false;
  }
  RCLCPP_INFO(this->get_logger(),
              "making a service request to /map_server/map !");

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  using ServiceResponseFuture =
      rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    try {
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "inside the handle response callback !");
      handle_response(response);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      return false;
    }
  };
  auto future_result = get_map_srv_client->async_send_request(
      request, response_received_callback);
  return true;
  // Update the map by calling the GetMap service or any other mechanism
  // You can use the get_map_srv_client to send a request and receive a
  // response
}

bool MultiRobotPlanner::updateRobotPlan(
    std::vector<StatePath> &robot_state_paths) {
  try {
    if (robot_state_paths.empty()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Empty path received for all agent skipping update");
      return false;
    }
    bool ret = true;
    //  Not handeled properly -> TODO fix this
    // This will return false even if one of the paths is empty - we don't want
    // that!
    for (int agent_idx = 0; agent_idx < _agentNum; agent_idx++) {
      if (robot_state_paths[agent_idx].empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Empty path received for agent %d, skipping", agent_idx);
        ret = false;
      }
      PosePath tmp_pose_path;
      convertPathToPosePath(robot_state_paths[agent_idx], tmp_pose_path);
      robots_paths[agent_idx].clear();
      robot_waypoints[agent_idx] = tmp_pose_path[0];
      for (auto path_entry : tmp_pose_path) {
        robots_paths[agent_idx].push_back(path_entry);
      }
      // robot_waypoints[agent_idx] = robots_paths[agent_idx][0];
      // robots_paths[agent_idx].pop_front();
    }
    return ret;
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in updating robot plan: %s",
                 ex.what());
    return false;
  }
}

bool MultiRobotPlanner::getRobotPose(
    std::string &robot_namespace, geometry_msgs::msg::PoseStamped &robot_pose) {
  std::string target_frame = "map";  // The frame you want the pose in
  std::string source_frame =
      // robot_namespace + "/" + "base_link";  // The robotX's base frame
      robot_namespace + "base_link";  // The robotX's base frame
  // TODO @VineetTambe add is simulation tag here.
  // Get the current pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame,
                                                    tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "Failed to obtain robot pose: %s", ex.what());
    return false;
  }
  // Create a PoseStamped message
  robot_pose.header.frame_id = target_frame;
  robot_pose.header.stamp = transform_stamped.header.stamp;
  robot_pose.pose.position.x = transform_stamped.transform.translation.x;
  robot_pose.pose.position.y = transform_stamped.transform.translation.y;
  robot_pose.pose.position.z = transform_stamped.transform.translation.z;
  robot_pose.pose.orientation = transform_stamped.transform.rotation;
  return true;
}

bool MultiRobotPlanner::createAgentScenarioFile(
    std::vector<geometry_msgs::msg::Pose> &robot_start_poses,
    std::vector<geometry_msgs::msg::Pose> &robot_goal_poses,
    std::string &map_file_name, int &map_height, int &map_width,
    std::string &file_path) {
  // Create a scenario file for the CBS algorithm
  std::ofstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s",
                 file_path.c_str());
    return false;
  }

  file << "version 1" << std::endl;

  int bucket = 0;
  int optimal_length = 1;

  for (int i = 0; i < robot_start_poses.size(); i++) {
    file << bucket << "\t" << map_file_name << "\t" << map_height << "\t"
         << map_width << "\t" << robot_start_poses[i].position.x << "\t"
         << robot_start_poses[i].position.y << "\t"
         << robot_goal_poses[i].position.x << "\t"
         << robot_goal_poses[i].position.y << "\t" << optimal_length
         << std::endl;
  }

  file.close();

  RCLCPP_INFO(this->get_logger(), "Agent Scen file written to: %s",
              file_path.c_str());
  return true;
}

bool MultiRobotPlanner::planPaths(
    // std::vector<std::string> &robot_namespaces,
    std::vector<geometry_msgs::msg::PoseStamped> &agent_start_poses,
    std::vector<geometry_msgs::msg::PoseStamped> &agent_goal_poses,
    std::vector<nav_msgs::msg::Path> &cbs_planned_paths) {
  // updateMap();
  int tmp_agent_num = agent_start_poses.size();
  std::vector<AgentState> tmp_agent_goal_states;
  std::vector<AgentState> tmp_agent_start_states;

  RCLCPP_INFO(this->get_logger(), "Number of agents: %d", tmp_agent_num);

  for (int i = 0; i < tmp_agent_num; i++) {
    tmp_agent_start_states.push_back(coordToCBS(agent_start_poses[i].pose));
    tmp_agent_goal_states.push_back(coordToCBS(agent_goal_poses[i].pose));
  }

  instance_ptr->printMap();

  RCLCPP_INFO(this->get_logger(), "Setting states!");

  instance_ptr->updateAgents(tmp_agent_num, tmp_agent_start_states,
                             tmp_agent_goal_states);

  RCLCPP_INFO(this->get_logger(), "Planning Paths!");
  std::vector<StatePath> planned_paths;
  callCBS(planned_paths);

  RCLCPP_INFO(this->get_logger(), "returning paths!");
  for (int i = 0; i < tmp_agent_num; i++) {
    nav_msgs::msg::Path tmp_pose_path;
    // TODO update this
    tmp_pose_path.header.frame_id = "robot" + std::to_string(i);

    for (auto state_pose : planned_paths[i]) {
      geometry_msgs::msg::PoseStamped tmp_robot_pose;
      tmp_robot_pose.header.frame_id = "map";
      tmp_robot_pose.pose = coordToGazebo(state_pose);
      tmp_pose_path.poses.push_back(tmp_robot_pose);
    }
    cbs_planned_paths.push_back(tmp_pose_path);
  }
  return true;
}

// Example:
// std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_path;
// ... convert the path to PoseStamped messages ...
// return pose_stamped_path;
void MultiRobotPlanner::convertPathToPosePath(StatePath &state_path,
                                              PosePath &pose_path) {
  // pose_path.clear();
  for (auto entry : state_path) {
    geometry_msgs::msg::Pose tmp_robot_pose = coordToGazebo(entry);
    // tmp_robot_pose.orientation.w = 1.0;
    pose_path.push_back(tmp_robot_pose);
  }
}

void MultiRobotPlanner::handleGetMultiPlanServiceRequest(
    const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
        request,
    std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
        response) {
  try {
    response->success =
        planPaths(request->start, request->goal, response->plan);
  } catch (const std::exception &ex) {
    // Handle the exception
    response->success = false;
    RCLCPP_ERROR(this->get_logger(), "Failed to plan paths: %s", ex.what());
  }
  return;
}

}  // namespace multi_robot_planner

int main(int argc, char *argv[]) {
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_robot_planner::MultiRobotPlanner>());

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}