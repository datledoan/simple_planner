# simple_planner
Package that creates a simple global planner from the start point to the destination, capable of passing through predefined waypoints. The predefined waypoints are retrieved from the /waypoints topic. You can use the [waypoint_rviz_plugin](https://github.com/datledoan/waypoint_rviz_plugin) to set these waypoints in the RViz interface.

# Subscribe topic
geometry_msgs::PoseArray /waypoints

# Installation
* Clone code to your workspace and build
    ```sh
    cd your_ws/src
    git clone https://github.com/datledoan/simple_planner.git
    catkin build
    ```

# Demo


https://github.com/user-attachments/assets/c5a91521-c0db-453c-9680-d344c0823a62

