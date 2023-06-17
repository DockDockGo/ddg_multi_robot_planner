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

    // Write map data to file
    for (int i = 0; i < (int)occupancy_map.size(); ++i)
    {
      for (int j = 0; j < (int)occupancy_map[0].size(); ++j)
      {
        // int index = i * map.info.width + j;
        file << static_cast<int>(occupancy_map[i][j]) << " ";
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
          occupancy_map[i][j] = '@';
        }
        else
        {
          occupancy_map[i][j] = '.';
        }
      }
    }
    // writeMapToFile(occupancy_map, "/home/admin/ddg_mfi/mp_400_ws/svd_demo-parsed-map.txt");
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

    try
    {
      // auto response = future.get();
      if (occupancy_map.size() != response->map.info.height && occupancy_map[0].size() != response->map.info.width)
      {
        occupancy_map.clear();
        occupancy_map.reserve(response->map.info.height);
        for (int i = 0; i < (int)response->map.info.height; i++)
        {
          std::vector<char> innervec;
          innervec.reserve(response->map.info.width);
          occupancy_map.push_back(innervec);
        }
        occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
      }
      // occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
      parseMap(occupancy_map, response->map);

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully received map");
      printMap(occupancy_map);
    }
    catch (const std::exception &e)
    {
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

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

    // Send the service request
    auto future = get_map_srv_client->async_send_request(request);

    // Create a callback to handle the response
    auto handle_response_callback = [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future)
    {
      try
      {
        auto response = future.get();
        handle_response(response);
      }
      catch (const std::exception &e)
      {
        // RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        return false;
      }
    };

    // TODO (@VineetTambe: replace the below while loop with a timer callback)
    // Spin using spin_once() in a while loop until the response is received
    // while (rclcpp::ok())
    // {
    //   rclcpp::spin_some(this);
    //   // map service subscriber
    //   if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    //   {
    //     handle_response_callback(future);
    //   }
    // }

    rclcpp::TimerBase::SharedPtr timer = create_wall_timer(
        std::chrono::seconds(1), [&]()
        {
          if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
              handle_response_callback(future);
              timer->cancel();  // Stop the timer once the response is received
          } });

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

  bool MultiRobotPlanner:: planPaths(
            std::vector<std::string>& robot_namespaces,
            std::vector<geometry_msgs::msg::Pose>& robot_start_poses,
            std::vector<geometry_msgs::msg::Pose>& robot_goal_poses,
            std::vector<nav_msgs::msg::Path>& planned_paths
        )
  {
    // Plan paths for all robots based on their current poses
    // Return a mapping of robot namespace to their planned paths

    // Example:
    // std::vector<std::pair<int, int>> planned_paths;

    // for (const auto &robot_pose : robot_curr_poses)
    // {
    //     const std::string &robot_namespace = robot_pose.first;
    //

    // updateRobotPoses();

    //  TODO call CBS object

    // convertPathToPoseStamped(planned_paths);

    // std::unordered_map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> temp;

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
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to plan paths: %s", ex.what());
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