# ddg_multi_robot_planner

This repo contains CBS based ROS2 planner

Instructions to run:

The planner can be launched byy running:
```
ros2 launch ddg_multi_robot_planner multi_robot_planner.launch.py
```

The planner listens on the ros2 topic:
```
/<agent_name>/cbs_path/goal_pose"
```
And expects the following a `geometry_msgs::msg::PoseStamped` message in the request.

Upon receiving data on this topic the planner will add the agent and goal to a vector that is maintained internally.
This datastructure is passed to the underlying CBS planner.

An implementation of the publisher which publishes target poses for the above planner to plan can be found here:
1. [multi_navigator](https://github.com/DockDockGo/multi_navigator/tree/cbs_navigator) ros2 package.



Other additional packages that are used in ddg_multi_robot_planner are:
1. [ddg_multi_robot_srvs](https://github.com/DockDockGo/ddg_multi_robot_srvs) - contains srv files for all the custom services used internally by ddg_multi_robot_planner 
