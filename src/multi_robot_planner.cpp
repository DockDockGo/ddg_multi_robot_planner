#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>

void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Print out the width and height of the occupancy grid
  std::cout << "Received map with width=" << msg->info.width << " and height=" << msg->info.height << std::endl;
}

int main(int argc, char* argv[])
{
  // Initialize the ROS2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("multi_robot_planner_node");

  // Create the subscriber to the "/map" topic
  auto subscriber = node->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",
                                                                            10,  // QoS history depth
                                                                            mapCallback);

  // TODO create an instance of MultiSpaceTimeAstarPlanner class

  // TODO initalize the graph of the planner with the map

  // TODO create a service call exposing the MultiSpaceTimeAstarPlanner objects plan function

  // Spin the node so the subscriber can receive messages
  rclcpp::spin(node);

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
