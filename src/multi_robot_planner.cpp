#include "ddg_multi_robot_planner/multi_robot_planner.hpp"

namespace multi_robot_planner
{
  MultiRobotPlanner::MultiRobotPlanner() : Node("multi_robot_planner_node")
  {
    // Initialize your variables and subscribers here
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    get_map_srv_client = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Not implemented properly
    map_update_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MultiRobotPlanner::mapCallback, this, std::placeholders::_1));

    // create a ros2 service call to call planPaths
    get_multi_plan_service_ = create_service<ddg_multi_robot_srvs::srv::GetMultiPlan>(
        "/multi_robot_planner/get_plan",
        [this](const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request> request,
              std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response> response) {
            handleGetMultiPlanServiceRequest(request, response);
        }
    );

    // create a ros2 service call to call updateNamespaces
    // integrate ros2 params from launch files to get the robotnamespaces

    // CBS object initialization
  }

  MultiRobotPlanner::~MultiRobotPlanner()
  {
    // Perform any cleanup or resource deallocation here
  }

  void MultiRobotPlanner::updateNamespaces(std::vector<std::string> &robot_namespaces)
  {
    // Update the robot namespaces
    this->robot_namespaces = robot_namespaces;
  }

  void MultiRobotPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    // TODO (@VineetTambe: change this to an actual mpa update this function)
    // Print out the width and height of the occupancy grid
    std::cout << "Received map with width=" << msg->info.width << " and height=" << msg->info.height << std::endl;
  }

  void MultiRobotPlanner::writeMapToFile(std::vector<std::vector<char>> &occupancy_map, const std::string &file_path)
  {
    std::ofstream file(file_path);
    if (!file.is_open())
    {
      // RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Failed to open file for writing");
      return;
    }
    file << "type octile" << std::endl;
    file << "height " << (int)occupancy_map.size() << std::endl;
    file << "width " << (int)occupancy_map[0].size() << std::endl;
    file << "map" << std::endl;

    // Write map data to file
    for (int i = 0; i < (int)occupancy_map.size(); ++i)
    {
      for (int j = 0; j < (int)occupancy_map[0].size(); ++j)
      {
        // int index = i * map.info.width + j;
        file << static_cast<char>(occupancy_map[i][j]);
      }
      file << std::endl;
    }

    file.close();

    RCLCPP_INFO(rclcpp::get_logger("service_client"), "Map written to file: %s", file_path.c_str());
  }

  void MultiRobotPlanner::parseMap(std::vector<std::vector<char>> &occupancy_map, const nav_msgs::msg::OccupancyGrid &map)
  {
    for (int i = 0; i < (int)map.info.height; i++)
    {
      for (int j = 0; j < (int)map.info.width; j++)
      {
        if (map.data[i * (int)map.info.width + j] > 50)
        {
          // RCLCPP_INFO(rclcpp::get_logger("service_client"), "HERE");
          occupancy_map[i][j] = '@';
        }
        else
        {
          occupancy_map[i][j] = '.';
        }
      }
    }
    // TODO change file naming method 
    writeMapToFile(occupancy_map, _map_file_name);
  }

  void MultiRobotPlanner::printMap(std::vector<std::vector<char>> &map)
  {
    for (int i = 0; i < (int)map.size(); i++)
    {
      for (int j = 0; j < (int)map[0].size(); j++)
      {
        std::cout << map[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }

  void MultiRobotPlanner::handle_response(nav_msgs::srv::GetMap::Response::SharedPtr response)
  {
    std::vector<std::vector<char>> occupancy_map;
    RCLCPP_INFO(this->get_logger(), "inside handle map update response ...");

    try
    {
      occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
      _map_height = response->map.info.height;
      _map_width = response->map.info.width;
      // occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
      parseMap(occupancy_map, response->map);

      RCLCPP_INFO(this->get_logger(), "Successfully received map");

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
      
      bool res = createAgentScenarioFile(robot_start_poses, robot_goal_poses, _map_file_name, _map_height, _map_width, _agent_scen_file_name);
      if (!res)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to create scenario file");
        // return false;
      }
    
      // printMap(occupancy_map);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Error in parsing occupancy grid: %s", ex.what());
      // RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Service call failed: %s", e.what());
    }
  }

  bool MultiRobotPlanner::updateMap()
  {
    // Wait for the service to be available
    while (!get_map_srv_client->wait_for_service(std::chrono::seconds(1)))
    {
      // TODO add a counter to prevent this loop from blocking the thread.
      if (!rclcpp::ok())
      {
        // RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "making a service request to /map_server/map !");

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    using ServiceResponseFuture =
      rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
        try
        {
          auto response = future.get();
          RCLCPP_INFO(this->get_logger(), "inside the handle response callback !");
          handle_response(response);
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
          return false;
        }
      };
    auto future_result = get_map_srv_client->async_send_request(request, response_received_callback);
    return true;
    // Update the map by calling the GetMap service or any other mechanism
    // You can use the get_map_srv_client to send a request and receive a response
  }

  bool MultiRobotPlanner::getRobotPose(std::string &robot_namespace, geometry_msgs::msg::PoseStamped &robot_pose)
  {
    // Wait for the transformation to become available
    std::string target_frame = "map";                               // The frame you want the pose in
    std::string source_frame = robot_namespace + "/" + "base_link"; // The robotX's base frame
    try
    {
      tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero,
                                  tf2::Duration(std::chrono::seconds(1)));
    }
    catch (tf2::TransformException &ex)
    {
      // RCLCPP_ERROR(get_logger(), "Failed to obtain robot pose: %s", ex.what());
      // exit(EXIT_FAILURE);
      return false;
    }
    // Get the current pose
    geometry_msgs::msg::TransformStamped transform_stamped;
    try
    {
      transform_stamped = tf_buffer_->lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
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

  bool MultiRobotPlanner::updateRobotPoses()
  {
    bool ret = true;
    for (auto &robot_namespace : robot_namespaces)
    {
      if (robot_curr_poses.find(robot_namespace) == robot_curr_poses.end())
      {
        robot_curr_poses[robot_namespace] = geometry_msgs::msg::PoseStamped();
      }
      ret = ret && getRobotPose(robot_namespace, robot_curr_poses[robot_namespace]);
    }
    return ret;
  }

  bool MultiRobotPlanner::createAgentScenarioFile(std::vector<geometry_msgs::msg::Pose>& robot_start_poses,
    std::vector<geometry_msgs::msg::Pose>& robot_goal_poses, 
    std::string &map_file_name,
    int &map_height,
    int &map_width,
    std::string &file_path){
    // Create a scenario file for the CBS algorithm
    std::ofstream file(file_path);
    if (!file.is_open())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", file_path.c_str());
      return false;
    }

    file << "version 1" << std::endl;
  
    int bucket = 0;
    int optimal_length = 1;

    for(int i = 0 ; i < robot_start_poses.size() ; i++){
      file << bucket << "\t" << map_file_name << "\t" << map_height << "\t" << map_width << "\t" << robot_start_poses[i].position.x << "\t" << robot_start_poses[i].position.y << "\t" << robot_goal_poses[i].position.x << "\t" << robot_goal_poses[i].position.y << "\t" << optimal_length << std::endl;
    }
    
    file.close();

    RCLCPP_INFO(this->get_logger(), "Agent Scen file written to: %s", file_path.c_str());
    return true;
  }

  bool MultiRobotPlanner:: planPaths(
            std::vector<std::string>& robot_namespaces,
            std::vector<geometry_msgs::msg::Pose>& robot_start_poses,
            std::vector<geometry_msgs::msg::Pose>& robot_goal_poses,
            std::vector<nav_msgs::msg::Path>& planned_paths
        )
  {
    updateMap();
  
    return true;
  }

  std::vector<geometry_msgs::msg::PoseStamped> MultiRobotPlanner::convertPathToPoseStamped(std::vector<std::pair<int, int>> path)
  {
    // Convert the planned path represented as a list of grid positions to PoseStamped messages

    // Example:
    // std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_path;
    // ... convert the path to PoseStamped messages ...
    // return pose_stamped_path;
  }

  void MultiRobotPlanner::handleGetMultiPlanServiceRequest(
    const std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Request> request,
    std::shared_ptr<ddg_multi_robot_srvs::srv::GetMultiPlan::Response> response) {
    try {
      response->success = planPaths(request->robot_namespaces, request->start, request->goal, response->plan);
      response->robot_namespaces = request->robot_namespaces;
    } catch (const std::exception& ex) {
      // Handle the exception
      response->success = false;
      // response->error_message = ex.what();
      RCLCPP_ERROR(this->get_logger(), "Failed to plan paths: %s", ex.what());
    }
    return;
}

} // namespace multi_robot_planner

int main(int argc, char *argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_robot_planner::MultiRobotPlanner>());

  // crete a service call to get paths for n robots -> service call.
  /** Service call accepts the following arguments:
  0. get the map                      [x]
  1. start position of the robot      [ ]
  2. goal postion of the robot        [ ]
  3. robot namespaces                 [ ]

  Service call returns the following:
  1. A hashmap of posestamped messages with pose for each robot.
  */

  // TODO call this service call from multi-navigator node.

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}