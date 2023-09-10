#include "multi_robot_planner.hpp"

#include <vector>

#include "CBS.h"
// #include <CBS.h>
namespace multi_robot_planner {
MultiRobotPlanner::MultiRobotPlanner() : Node("multi_robot_planner_node") {
  clock_ = get_clock();

  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(
      1500ms, std::bind(&MultiRobotPlanner::timer_callback, this));

  RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                     "Start multi robot planner!");

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

    std::string tmp_pose_topic_name = "/" + tmp_robot_name + "/odom";
    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("rclcpp"),
        "The topic to subscribe for agent odom is: " << tmp_pose_topic_name);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr tmp_pose_sub =
        this->create_subscription<nav_msgs::msg::Odometry>(
            tmp_pose_topic_name, 10,
            std::bind(&MultiRobotPlanner::RobotPoseCallback, this,
                      std::placeholders::_1));
    agents_sub_pose.push_back(tmp_pose_sub);

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

  std::vector<std::pair<int, int>> planned_paths;
  std::pair<int, int> tmp_point;
  tmp_point.first = 2.0;
  tmp_point.second = 2.0;
  planned_paths.push_back(tmp_point);
  tmp_point.first = 1.0;
  tmp_point.second = 3.0;
  planned_paths.push_back(tmp_point);

  publishPlannedPaths(planned_paths);
  printf("Finish multi robot planner!\n");

  // // Initialize your variables and subscribers here
  // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // get_map_srv_client =
  //     this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // // Not implemented properly
  // map_update_subscriber =
  //     this->create_subscription<nav_msgs::msg::OccupancyGrid>(
  //         "/map", 10,
  //         std::bind(&MultiRobotPlanner::mapCallback, this,
  //                   std::placeholders::_1));

  // // create a ros2 service call to call planPaths
  // get_multi_plan_service_ =
  //     create_service<ddg_multi_robot_srvs::srv::GetMultiPlan>(
  //         "/multi_robot_planner/get_plan",
  //         [this](
  //             const std::shared_ptr<
  //                 ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
  //                 request,
  //             std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
  //                 response)
  //         {
  //           handleGetMultiPlanServiceRequest(request, response);
  //         });

  // // create a ros2 service call to call updateNamespaces
  // // integrate ros2 params from launch files to get the robotnamespaces

  // // CBS object initialization
  // // configure();
}

bool MultiRobotPlanner::Initialize() {
  // This is just dummy code for testing -> replace this with the actual robot
  // pose test code
  geometry_msgs::msg::Pose tempPose;
  AgentState tmp_state;
  tempPose.position.x = 0.25;
  tempPose.position.y = -0.75;
  robot_start_poses.push_back(tempPose);
  robot_curr_poses.push_back(tempPose);
  robot_waypoints.push_back(tempPose);
  robot_waypoints.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_start_states.push_back(tmp_state);
  agent_curr_states.push_back(tmp_state);

  tempPose.position.x = 4.75;
  tempPose.position.y = -1.25;
  tmp_state = coordToCBS(tempPose);
  GLOBAL_START.push_back(tmp_state);

  tempPose.position.x = 0.25;
  tempPose.position.y = 0.25;
  robot_start_poses.push_back(tempPose);
  robot_curr_poses.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_start_states.push_back(tmp_state);
  agent_curr_states.push_back(tmp_state);

  tempPose.position.x = 1.25;
  tempPose.position.y = -1.25;
  tmp_state = coordToCBS(tempPose);
  GLOBAL_START.push_back(tmp_state);

  tempPose.position.x = 9.75;
  tempPose.position.y = -0.25;
  robot_goal_poses.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_goal_states.push_back(tmp_state);
  GLOBAL_GOAL.push_back(tmp_state);
  next_round_goal.push_back(tmp_state);

  tempPose.position.x = 8.25;
  tempPose.position.y = -0.25;
  robot_goal_poses.push_back(tempPose);
  tmp_state = coordToCBS(tempPose);
  agent_goal_states.push_back(tmp_state);
  GLOBAL_GOAL.push_back(tmp_state);
  next_round_goal.push_back(tmp_state);

  // instance_ptr = std::make_shared<Instance>(
  //     "./src/ddg_multi_robot_planner/maps/svddemo-14-44-2.map");

  instance_ptr = std::make_shared<Instance>(
      "./src/ddg_multi_robot_planner/maps/downsampled-map/"
      "svd_demo-downsampled.map");

  // instance_ptr = std::make_shared<Instance>(_map_file_name);

  instance_ptr->updateAgents(_agentNum, agent_start_states, agent_goal_states);
  instance_ptr->printMap();
  // instance.printAgents();

  robots_paths.resize(_agentNum);
  robot_waypoints.resize(_agentNum);

  std::vector<StatePath> planned_paths;
  callCBS(planned_paths);
  updateRobotPlan(planned_paths);
  PublishMarker();

  // next_round_goal.resize(_agentNum);
  next_round_start.resize(_agentNum);

  for (int i = 0; i < _agentNum; i++) {
    trip_directions.push_back(false);
    at_goal_wait.push_back(WAITSTEP);
  }
  planner_initialized = true;
}

void MultiRobotPlanner::timer_callback() {
  if (!planner_initialized) {
    return;
  }
  std::vector<StatePath> planned_paths;

  // int agent_finish_count = 0;
  for (int i = 0; i < _agentNum; i++) {
    if (robots_paths[i].empty()) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Agent %d reaches its goal", i);
      if (at_goal_wait[i] > 0) {
        at_goal_wait[i]--;
        continue;
      } else {
        at_goal_wait[i] = WAITSTEP;
      }
      geometry_msgs::msg::Pose next_tmp_pose = robot_curr_poses[i];
      std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
      next_round_start[i] = next_tmp_state;
      next_round_goal[i] = GLOBAL_GOAL[i];
      // agent_finish_count++;

      // if (!trip_directions[i]) {
      //     next_round_goal[i] = GLOBAL_START[i];
      //     geometry_msgs::msg::Pose next_tmp_pose = robot_curr_poses[i];
      //     std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
      //     next_round_start[i] = next_tmp_state;
      //     trip_directions[i] = true;
      // } else {
      //     next_round_goal[i] = GLOBAL_GOAL[i];
      //     geometry_msgs::msg::Pose next_tmp_pose = robot_curr_poses[i];
      //     std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
      //     next_round_start[i] = next_tmp_state;
      //     trip_directions[i] = false;
      // }
    } else {
      geometry_msgs::msg::Pose next_tmp_pose = robots_paths[i][AHEAD_TIME];
      // geometry_msgs::msg::Pose next_tmp_pose = robot_curr_poses[i];
      std::pair<int, int> next_tmp_state = coordToCBS(next_tmp_pose);
      next_round_start[i] = next_tmp_state;
    }
  }
  instance_ptr->updateStarts(next_round_start);
  instance_ptr->updateGoals(next_round_goal);
  callCBS(planned_paths);
  updateRobotPlan(planned_paths);
  PublishMarker();
  for (unsigned int i = 0; i < _agentNum; i++) {
    PublishCBSPath(i, planned_paths[i]);
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

  // //only if using a MESH_RESOURCE marker type:
  // marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  // vis_pub.publish( marker );
}

void MultiRobotPlanner::PublishCBSPath(int agent_idx, StatePath &agent_path) {
  nav_msgs::msg::Path pose_path;
  // pose_path.header.seq = path_counter_;
  // path_counter_++;
  pose_path.header.frame_id = "map";
  pose_path.header.stamp = clock_->now();
  for (auto entry : agent_path) {
    geometry_msgs::msg::PoseStamped tmp_pose;
    geometry_msgs::msg::Pose tmp_tmp_pose;
    tmp_tmp_pose = coordToGazebo(entry);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pose is: (%f, %f)",
    //     tmp_tmp_pose.position.x, tmp_tmp_pose.position.y);
    tmp_pose.pose = tmp_tmp_pose;
    tmp_pose.header.frame_id = "map";
    tmp_pose.header.stamp = clock_->now();
    pose_path.poses.push_back(tmp_pose);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "StampPose is: (%f, %f)",
    //     tmp_pose.pose.position.x, tmp_pose.pose.position.y);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Entry is: (%f, %f)",
    //     entry.first, entry.second);
  }
  agents_pub_path[agent_idx]->publish(pose_path);
}

void MultiRobotPlanner::printStatePath(StatePath agent_path) {
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The path for agent is: %d", i);
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
    robots_paths[agent_idx].clear();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing Goal Pose message: %s",
                 agent_name);
  }
}

void MultiRobotPlanner::RobotPoseCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  if (!planner_initialized) {
    return;
  }
  // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Sub odom: " <<
  // msg->header.frame_id);
  std::string frame_header = msg->header.frame_id;

  unsigned int agent_idx = 0;
  bool correct_msg = false;
  try {
    for (unsigned int i = 0; i < _agentNum; i++) {
      if (frame_header.find(robot_namespaces[i]) != std::string::npos) {
        agent_idx = i;
        // RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "The agent is: " <<
        // agent_idx);
        correct_msg = true;
        break;
      }
    }
  } catch (const std::exception &ex) {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing odometry: %s",
                 ex.what());
  }

  // RCLCPP_INFO(this->get_logger(), "The agent is: %d, at (%f, %f)",
  //     agent_idx, robot_curr_poses[agent_idx].position.x,
  //     robot_curr_poses[agent_idx].position.y);
  if (correct_msg) {
    robot_curr_poses[agent_idx].position.x = msg->pose.pose.position.x;
    robot_curr_poses[agent_idx].position.y = msg->pose.pose.position.y;

    return;

    if (twoPoseDist(robot_curr_poses[agent_idx], robot_waypoints[agent_idx]) <
        EPS) {
      if (robots_paths[agent_idx].empty()) {
        // RCLCPP_WARN(this->get_logger(), "Path for agent: %d is empty!",
        // agent_idx);
        return;
      } else {
        robot_waypoints[agent_idx] = robots_paths[agent_idx][0];
        robots_paths[agent_idx].pop_front();
        // publishWaypoint(robot_waypoints[agent_idx], agent_idx);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "Publish waypoint for agent %d, at (%f, %f)", agent_idx,
                    robot_waypoints[agent_idx].position.x,
                    robot_waypoints[agent_idx].position.y);
      }
    } else {
      // RCLCPP_INFO(this->get_logger(), "Distance for agent: %d bigger than
      // EPS, curr pose: (%f, %f), waypoint: (%f, %f)!",
      //     agent_idx, robot_curr_poses[agent_idx].position.x,
      //     robot_curr_poses[agent_idx].position.y,
      //     robot_waypoints[agent_idx].position.x,
      //     robot_waypoints[agent_idx].position.y);
      ;
    }
    publishWaypoint(robot_waypoints[agent_idx], agent_idx);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Error in parsing odometry: %s",
                 frame_header);
  }

  // RCLCPP_INFO(this->get_logger(), "The agent is: %d, at (%f, %f)",
  //     agent_idx, robot_curr_poses[agent_idx].position.x,
  //     robot_curr_poses[agent_idx].position.y);

  // exit(0);
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
  // goal_pose.pose.position.x = planned_paths[i].first * 1.0;
  // goal_pose.pose.position.y = planned_paths[i].second * 1.0;
  // goal_pose.pose.position.z = 0.0;
  // goal_pose.pose.orientation.w = 0.0;
  agents_pub_pose[agent_id]->publish(goal_pose);
  // RCLCPP_DEBUG(get_logger(), "Publish message");
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publish waypoint for agent %d,
  // at (%f, %f)",
  //     agent_id, goal_pose.pose.position.x, goal_pose.pose.position.y);
}

/**
 * TODO@Jingtian
 * transform between coordinates
 */
AgentState MultiRobotPlanner::coordToCBS(geometry_msgs::msg::Pose robot_pose) {
  // TODO @VineetTambe ask Jingtian how was this transform calculateds
  AgentState robot_state;

  // robot_state.first = (int)round(7.5 - 2 * robot_pose.position.y);
  // robot_state.second = (int)round(7.5 + 2 * robot_pose.position.x);
  float x_offset = 4.2;
  float y_offset = -5.4;
  float scale = 0.60;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The robot pose is: (%f, %f)",
              robot_pose.position.x, robot_pose.position.y);

  robot_state.first =
      (int)round(-1.0 * (robot_pose.position.y + y_offset) / scale);

  robot_state.second = (int)round((robot_pose.position.x + x_offset) / scale);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "The calculated robot state is: (%d, %d)", robot_state.first,
              robot_state.second);

  return robot_state;
}

geometry_msgs::msg::Pose MultiRobotPlanner::coordToGazebo(
    AgentState &agent_state) {
  geometry_msgs::msg::Pose agent_pose;
  // agent_pose.position.x = 0.5 * agent_state.second - 3.75;
  // agent_pose.position.y = -0.5 * agent_state.first + 3.75;

  float x_offset = 4.2;
  float y_offset = -5.4;
  float scale = 0.60;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "The agent state is: (%d, %d)",
              agent_state.first, agent_state.second);

  agent_pose.position.x = -1.0 * scale * agent_state.second - y_offset;
  agent_pose.position.y = scale * agent_state.first - x_offset;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Calculated agent pose is: (%f, %f)", agent_pose.position.x,
              agent_pose.position.y);

  return agent_pose;
}

void MultiRobotPlanner::configure() {
  // TODO Replace this with a function to pass the maps and location of agents
  // rather than files

  // Instance instance(_map_file_name, _agent_scen_file_name,
  //                   _agentNum, _agentIdx,
  //                   _rows, _cols, _num_obstacles, _warehouseWidth);

  // //////////////////////////////////////////////////////////////////////
  // /// initialize the solver
  // //////////////////////////////////////////////////////////////////////
  // CBS cbs(instance, _use_sipp, _display_output_on_screen);
  // cbs.setPrioritizeConflicts(_prioritizingConflicts);
  // cbs.setDisjointSplitting(_disjointSplitting);
  // cbs.setBypass(_bypass);
  // cbs.setRectangleReasoning(_rectangleReasoning);
  // cbs.setCorridorReasoning(_corridorReasoning);
  // cbs.setHeuristicType(_heuristics);
  // cbs.setTargetReasoning(_targetReasoning);
  // cbs.setMutexReasoning(_mutexReasoning);
  // cbs.setSavingStats(_saving_stats);
  // cbs.setNodeLimit(_nodeLimit);
}

MultiRobotPlanner::~MultiRobotPlanner() {
  // Perform any cleanup or resource deallocation here
}

void MultiRobotPlanner::updateNamespaces(
    std::vector<std::string> &robot_namespaces) {
  // Update the robot namespaces
  this->robot_namespaces = robot_namespaces;
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

    //////////////////////////////////////////////////////////////////////
    /// write results to files
    //////////////////////////////////////////////////////////////////////
    // if (vm.count("output"))
    //   cbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>() +
    //   ":" + vm["agentIdx"].as<string>());
    // // cbs.saveCT(vm["output"].as<string>() + ".tree"); // for debug
    // if (vm["stats"].as<bool>())
    // {
    //   cbs.saveStats(vm["output"].as<string>(), vm["agents"].as<string>() +
    //   ":" + vm["agentIdx"].as<string>());
    // }
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
  RCLCPP_INFO(this->get_logger(), "Running the cbs solver");

  //////////////////////////////////////////////////////////////////////
  /// run
  //////////////////////////////////////////////////////////////////////

  // TODO@Jingtian update the instance every time calls this function
  // instance_ptr->updateAgents(_agentNum, agent_start_states,
  // agent_goal_states);

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
    //   RCLCPP_INFO(this->get_logger(), "Succesfully found a path! Saving it to
    //   file");

    // TODO Replace this with a function to get paths from cbs
    cbs.getSolvedPaths(planned_paths);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to find a path!");
  }
  cbs.clearSearchEngines();

  // // load paths and return them
  // // std::vector<nav_msgs::msg::Path> planned_paths;
  // if (readPlannedPathFromFile(_planned_path_file_name, planned_paths))
  // {
  //   RCLCPP_INFO(this->get_logger(), "Successfully read planned path from
  //   file"); RCLCPP_INFO(this->get_logger(), "Yay!");
  //   // TODO publish the planned paths
  //   // return;
  // } else {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to read planned path from
  //   file!");
  // }
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
      // RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the
      // service");
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
  // You can use the get_map_srv_client to send a request and receive a response
}

bool MultiRobotPlanner::updateRobotPlan(
    std::vector<StatePath> &robot_state_paths) {
  for (int i = 0; i < _agentNum; i++) {
    PosePath tmp_pose_path;
    convertPathToPosePath(robot_state_paths[i], tmp_pose_path);
    robots_paths[i].clear();
    for (auto path_entry : tmp_pose_path) {
      robots_paths[i].push_back(path_entry);
    }
  }
  return true;
}

bool MultiRobotPlanner::getRobotPose(
    std::string &robot_namespace, geometry_msgs::msg::PoseStamped &robot_pose) {
  // Wait for the transformation to become available
  std::string target_frame = "map";  // The frame you want the pose in
  std::string source_frame =
      robot_namespace + "/" + "base_link";  // The robotX's base frame
  try {
    tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero,
                                tf2::Duration(std::chrono::seconds(1)));
  } catch (tf2::TransformException &ex) {
    // RCLCPP_ERROR(get_logger(), "Failed to obtain robot pose: %s", ex.what());
    // exit(EXIT_FAILURE);
    return false;
  }
  // Get the current pose
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame,
                                                    tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    // RCLCPP_ERROR(get_logger(), "Failed to obtain robot pose: %s", ex.what());
    // exit(EXIT_FAILURE);
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

bool MultiRobotPlanner::updateRobotPoses() {
  bool ret = true;
  //   for (auto &robot_namespace : robot_namespaces)
  //   {
  //     if (robot_curr_poses.find(robot_namespace) == robot_curr_poses.end())
  //     {
  //       robot_curr_poses[robot_namespace] =
  //       geometry_msgs::msg::PoseStamped();
  //     }
  //     ret =
  //         ret && getRobotPose(robot_namespace,
  //         robot_curr_poses[robot_namespace]);
  //   }
  return ret;
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
    std::vector<std::string> &robot_namespaces,
    std::vector<geometry_msgs::msg::Pose> &robot_start_poses,
    std::vector<geometry_msgs::msg::Pose> &robot_goal_poses,
    std::vector<nav_msgs::msg::Path> &planned_paths) {
  updateMap();
  // std::vector<nav_msgs::msg::Path> planned_paths;
  // publishPlannedPaths(planned_paths);

  return true;
}

// Convert the planned path represented as a list of grid positions to
// PoseStamped messages

// Example:
// std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_path;
// ... convert the path to PoseStamped messages ...
// return pose_stamped_path;
void MultiRobotPlanner::convertPathToPosePath(StatePath &state_path,
                                              PosePath &pose_path) {
  pose_path.clear();
  for (auto entry : state_path) {
    geometry_msgs::msg::Pose tmp_robot_pose = coordToGazebo(entry);
    pose_path.push_back(tmp_robot_pose);
  }
}

void MultiRobotPlanner::handleGetMultiPlanServiceRequest(
    const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request>
        request,
    std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response>
        response) {
  try {
    response->success = planPaths(request->robot_namespaces, request->start,
                                  request->goal, response->plan);
    response->robot_namespaces = request->robot_namespaces;
  } catch (const std::exception &ex) {
    // Handle the exception
    response->success = false;
    // response->error_message = ex.what();
    RCLCPP_ERROR(this->get_logger(), "Failed to plan paths: %s", ex.what());
  }
  return;
}

}  // namespace multi_robot_planner

int main(int argc, char *argv[]) {
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_robot_planner::MultiRobotPlanner>());

  // crete a service call to get paths for n robots -> service call.
  /** Service call accepts the following arguments:
  0. get the map                      [x]
  1. start position of the robot      [x]
  2. goal postion of the robot        [x]
  3. robot namespaces                 [x]

  Service call returns the following:
  1. A hashmap of posestamped messages with pose for each robot.
  */

  // TODO call this service call from multi-navigator node.

  /**
   * Todo @ Jingtian
   * 1. Write and read file with start and goal location for agents        [x]
   * 2. Initialize the instance                                            [x]
   * 3. Subscribe to the robot pose and pushlish next goal                 [x]
   * 4. Call CBS planner and the path                                      [x]
   * 5. Map allignment, from coordinate of Gazebo to gridmap               [x]
   * 6. Transform the state path, and pub the next state                   [x]
   * 7. Figure out the orientation problem                                 [ ]
   **/

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}