#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>

// #include <vector>

void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Print out the width and height of the occupancy grid
  std::cout << "Received map with width=" << msg->info.width << " and height=" << msg->info.height << std::endl;
}

void parseMap(std::vector<std::vector<char>> & occupancy_map, const nav_msgs::msg::OccupancyGrid& map){
      // for i in range(row_num):
      //   tmp_str = ""
      //   for j in range(col_num):
      //       if mat[i, j] == 1:
      //           tmp_str += "@"
      //       else:
      //           tmp_str += "."
      //   tmp_str += "\n"
      //   f.write(tmp_str)

    for (int i = 0; i < (int)map.info.height; i++) {
      for (int j = 0; j < (int)map.info.width; j++) {
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
}

void printMap(std::vector<std::vector<char>>& map) 
{
  for(int i = 0 ; i < (int) map.size(); i++){
    for(int j = 0; j < (int) map[0].size(); j++){
      std::cout << map[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

void handle_response(nav_msgs::srv::GetMap::Response::SharedPtr response)
{
  std::vector<std::vector<char>> occupancy_map;

  try {
    // auto response = future.get();
    occupancy_map.resize(response->map.info.height, std::vector<char>(response->map.info.width, 0));
    parseMap(occupancy_map, response->map);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully received map");
    printMap(occupancy_map);

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("service_client"), "Service call failed: %s", e.what());
  }
  rclcpp::shutdown();
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
  

  // auto client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // // Wait for the service to be available
  // while (!client->wait_for_service(std::chrono::seconds(1))) {
  //   if (!rclcpp::ok()) {
  //     RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service");
  //     return 1;
  //   }
  //   RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  // }

  // RCLCPP_INFO(node->get_logger(), "Service found!");

  // auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  // // Send the service request
  // auto result = client->async_send_request(request);

  // try {
  //   // Check if the future has a valid response
  //   if (result.valid() && result.wait_for(std::chrono::seconds(15)) == std::future_status::ready) {
  //     // auto response = result.get();

  //     // // Access the map data
  //     // nav_msgs::msg::OccupancyGrid map = response->map;
  //     occupancy_map.resize(result.get()->map.info.height, std::vector<char>(result.get()->map.info.width, 0));
  //     parseMap(occupancy_map, result.get()->map);

  //     RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Successfully received map");
  //     printMap(occupancy_map);
  //     // Perform any cleanup or necessary actions
  //   } else {
  //     throw std::runtime_error("Service response not received");
  //   }
  // } catch (const std::exception& e) {
  //   RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
  // }


  auto client = node->create_client<nav_msgs::srv::GetMap>("/map_server/map");

  // Wait for the service to be available
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();

  // Send the service request
  auto future = client->async_send_request(request);

  // Wait for the service response and process it asynchronously
  // Wait for the service response and process it asynchronously
  // auto result_callback = std::bind(handle_response, std::placeholders::_1);
  // auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // rclcpp::spin_until_future_complete(node, future, callback_group, result_callback);

    // Create a callback to handle the response
  auto handle_response_callback = [&node](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future) {
    try {
      auto response = future.get();
      handle_response(response);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
      rclcpp::shutdown();
    }
  };

  // Spin using spin_once() in a while loop until the response is received
  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      handle_response_callback(future);
      break;
    }
  }



  // TODO create an instance of MultiSpaceTimeAstarPlanner class

  // TODO initalize the graph of the planner with the map

  // TODO create a service call exposing the MultiSpaceTimeAstarPlanner objects plan function

  // Spin the node so the subscriber can receive messages
  // rclcpp::spin(node);

  // rclcpp::WallRate loop_rate(30);
  // while (rclcpp::ok()) {
  //     rclcpp::spin_some(node);
  //     loop_rate.sleep();
  // }

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}
