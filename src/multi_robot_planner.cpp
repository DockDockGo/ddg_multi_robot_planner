#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <fstream>

// TODO massive code cleanup required.

void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Print out the width and height of the occupancy grid
  std::cout << "Received map with width=" << msg->info.width << " and height=" << msg->info.height << std::endl;
}

// debug function
void writeMapToFile(std::vector<std::vector<char>> &occupancy_map, const std::string &file_path)
{
  std::ofstream file(file_path);
  if (!file.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Failed to open file for writing");
    return;
  }

  // Write map data to file
  for (int i = 0; i < occupancy_map.size(); ++i)
  {
    for (int j = 0; j < occupancy_map[0].size(); ++j)
    {
      // int index = i * map.info.width + j;
      file << static_cast<int>(occupancy_map[i][j]) << " ";
    }
    file << std::endl;
  }

  file.close();

  RCLCPP_INFO(rclcpp::get_logger("service_client"), "Map written to file: %s", file_path.c_str());
}

void parseMap(std::vector<std::vector<char>> &occupancy_map, const nav_msgs::msg::OccupancyGrid &map)
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

void printMap(std::vector<std::vector<char>> &map)
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

void handle_response(nav_msgs::srv::GetMap::Response::SharedPtr response)
{
  std::vector<std::vector<char>> occupancy_map;

  try
  {
    // auto response = future.get();
    occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
    parseMap(occupancy_map, response->map);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully received map");
    printMap(occupancy_map);
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Service call failed: %s", e.what());
  }
  rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("multi_robot_planner_node");

  // Create the subscriber to the "/map" topic
  // auto subscriber = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
  //                                                                           10,  // QoS history depth
  //                                                                           mapCallback);

  // service client to subscriber to the static map
  // TODO update this to consider dynamics obstacles as well.
  auto client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1)))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  // Send the service request
  auto future = client->async_send_request(request);

  // Create a callback to handle the response
  auto handle_response_callback = [&node](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future)
  {
    try
    {
      auto response = future.get();
      handle_response(response);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
      rclcpp::shutdown();
    }
  };

  // Spin using spin_once() in a while loop until the response is received
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    // map service subscriber
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      handle_response_callback(future);
    }
  }

  // crete a service call to get paths for n robots -> service call.
  /** Service call accepts the following arguments:
  1. start position of the robot
  2. goal postion of the robot
  3. robot namespaces

  Service call returns the following:
  1. A hashmap of posestamped messages with pose for each robot.
  */

  // TODO call this service call from multi-navigator node.

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
