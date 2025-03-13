# INTRODUCTION
## PROJECT OVERVIEW
  This project involves the development of an autonomous restaurant robot designed to transport items between designated locations, such as the kitchen, tables, and a home position, within a mapped environment. The robot leverages the ROS Noetic framework, TurtleBot3, and MoveBase for autonomous navigation.

  The system is built upon SLAM-based navigation, utilizing an existing map for localization and path planning. The core functionality includes real-time goal assignment, dynamic point selection through RViz interactive markers, and obstacle avoidance using sensor-based perception.

## OBJECTIVE
The primary goal of this project is to implement a fully autonomous indoor navigation system for a restaurant setting, enabling the robot to:

- Dynamically set waypoints using RViz without modifying code manually.
- Efficiently navigate between predefined points (kitchen, tables, and home).
- Ensure smooth path planning with obstacle avoidance.
- Provide a reliable and adaptable system for autonomous service robots.

# Installation and Setup
## 1. Prerequisites
Before setting up the system, ensure that you have the necessary software and hardware:

### Hardware Requirements
- PC or Laptop running Ubuntu 20.04 with ROS Noetic installed.
- TurtleBot3 (Burger or Waffle Pi)
- LiDAR Sensor for obstacle detection (e.g., LDS-01).
- Raspberry Pi 4 (if using a real robot).

### Software Requirements
- Ubuntu 20.04
- ROS Noetic (with Navigation Stack and TurtleBot3 Packages)
- Gazebo 11 (for simulation)
- RViz (for visualization and interactive waypoint selection)
- Move Base (for autonomous navigation)
- GMapping/AMCL (for mapping and localization)
- TF Transformations (for coordinate frame conversions)

## 2. Dependencies and Installation Steps
### Step 1: Install ROS Noetic (if not installed)
If ROS Noetic is not installed, follow these steps:

> sudo apt update && sudo apt upgrade -y
> sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
> sudo apt install curl
> curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc' | sudo apt-key add -
> sudo apt update
> sudo apt install ros-noetic-desktop-full

Install necessary dependencies:

> sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
> sudo rosdep init
> rosdep update

### Step 2: Install TurtleBot3 Packages
> sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-turtlebot3-navigation

Set the TurtleBot3 model in the environment:
> echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
> source ~/.bashrc

### Step 3: Install Navigation Stack
> sudo apt install ros-noetic-navigation

### Step 4: Install Additional Dependencies
> sudo apt install ros-noetic-move-base ros-noetic-amcl ros-noetic-map-server ros-noetic-dwa-local-planner
> sudo apt install ros-noetic-tf ros-noetic-tf2-ros

### Step 5: Clone and Build the Custom Package
Navigate to your ROS workspace:
> mkdir -p ~/catkin_ws/src
> cd ~/catkin_ws/src
> git clone https://github.com/YOUR_GITHUB_REPO/restaurant_navigation.git
> cd ~/catkin_ws
> catkin_make
> source devel/setup.bash

## 3. How to Launch the Simulation
### 1. Launch the Gazebo Simulation
> roslaunch turtlebot3_gazebo turtlebot3_world.launch
This command spawns the TurtleBot3 in the simulated restaurant environment.

### 2. Run SLAM (Mapping the Environment)
If a map is not available, run GMapping to create one:
> roslaunch turtlebot3_slam turtlebot3_slam.launch use_sim_time:=true
Manually move the robot to build the map:
> roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
Once the mapping is done, save the map:
>rosrun map_server map_saver -f ~/catkin_ws/src/maps/restaurant_map

### 3. Start AMCL Localization
If a prebuilt map is available, launch AMCL for localization: 
> roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=~/catkin_ws/src/maps/restaurant_map.yaml

### 4. Start the Navigation System
Run the navigation script to set waypoints dynamically:
> rosrun restaurant_navigation navigation_node.py

### 5. Use RViz for Waypoint Selection
Launch RViz for real-time goal selection:
> roslaunch turtlebot3_navigation view_navigation.launch
Click on a location in RViz to set it as the goal.

# 5. Code Explanation
## 5.1. Main Script: restaurant_robot.py
The restaurant_robot.py script serves as the core node that orchestrates the restaurant robot’s tasks. Its primary responsibilities include:
- Task Coordination: Managing the workflow for both single and multiple orders.
- Navigation Control: Sending goals to the move_base action server to navigate between the home position, kitchen, and customer tables.
- User Interaction: Accepting and processing user inputs for order assignment, confirmations, and cancellations.

The script is organized into several modular functions:
  - move_to(location_name)
  - Retrieves the coordinates (x, y, and yaw) for a given location (e.g., "kitchen", "home", "table1") from a configuration (which can now be dynamically updated via RViz). It converts the yaw angle to a quaternion using a TF conversion function, constructs a MoveBaseGoal, and sends it to the move_base server. It then waits for the robot to reach the goal, logging the result.

> process_single_order(table_num)
- Manages the flow for a single order:
  From home to the kitchen for pickup.
  Waits for pickup confirmation (with a timeout).
  Navigates to the specified table and waits for delivery confirmation.
  Returns to the home position if delivery is confirmed, or follows the appropriate fallback if not.

> process_multiple_orders(order_list)
- Handles orders involving multiple tables by:
  Navigating once to the kitchen to collect orders.
  Sequentially visiting each table for delivery (with confirmation handling for each table).
  Returning to the kitchen and then home after completing all deliveries.

## 5.2. Updated RViz Point-Click Feature
To improve flexibility, the code now incorporates a dynamic waypoint selection system using RViz:
- Interactive Waypoint Selection:
  Instead of hardcoding coordinates, the system subscribes to the /clicked_point topic. When a user clicks a point in RViz, the node receives a geometry_msgs/PointStamped message.

- Dynamic Updates:
  A callback function processes the received point and updates the corresponding waypoint data (for example, assigning a clicked point as the kitchen, home, or one of the table positions). This allows for easy reconfiguration of the environment without altering the code.

## 5.3. MoveBase and Navigation Goals
The navigation system is built around ROS's move_base package:
- Goal Construction:
  The move_to function constructs a MoveBaseGoal using the selected coordinates and converts the yaw (angle in radians) to a quaternion. This conversion is crucial for correctly orienting the robot in the map frame.
- Action Client:
  The script uses an action client to communicate with the move_base action server. It sends the goal and waits for the result, ensuring that the robot has reached its intended destination.
- Path Planning:
  Behind the scenes, move_base leverages a global planner (such as A* or Dijkstra) to compute an optimal path and a local planner (like the Dynamic Window Approach) for smooth real-time navigation and obstacle avoidance.


## 5.4. Error Handling and Optimizations
Several measures have been implemented to ensure robust operation and improved maintainability:

- Timeouts for User Confirmation:
  Using Python’s signal module, the code enforces a timeout (10 seconds) for receiving user confirmations. This prevents the robot from waiting indefinitely and allows it to follow fallback behaviors if no response is received.

- Result Verification:
  After sending a goal to move_base, the code checks whether the goal was successfully reached. If the result indicates failure, it logs an appropriate warning message.

- Input Validation:
  The script validates user inputs for order types and table numbers, ensuring that only valid data is processed. This minimizes runtime errors due to unexpected or malformed input.

- Quaternion Conversion:
  To avoid errors like “Quaternion has length close to zero,” the script reliably converts Euler angles (yaw) to quaternions using the TF conversion functions. This guarantees that the robot’s orientation is always valid.

- Modular Function Design:
  The code is split into well-defined functions (move_to, process_single_order, process_multiple_orders), which not only enhances readability but also simplifies debugging and future enhancements.

- Dynamic Waypoint Updates:
  The integration of the RViz point-click feature allows for on-the-fly updates of the waypoint coordinates. This eliminates the need for hardcoding and makes the simulation adaptable to different environments without changing the underlying code.

# 6. Testing and Debugging
## 6.1. Common Issues and Fixes
- TF Transform Errors:
  - Issue: Warnings such as “Timed out waiting for transform from base_footprint to map” indicate missing or misconfigured transforms.
  - Fix: Verify that all required frames (map, odom, base_link, base_footprint) are published. Use rosrun tf view_frames to generate a PDF of the current TF tree. Ensure that AMCL or SLAM nodes are correctly initializing the transform publishers.

- Navigation Goal Failures:
  - Issue: The robot may not move if move_base returns a failure or if the goal’s quaternion is invalid.
  - Fix: Double-check the coordinate inputs and verify that the quaternion conversion from yaw is correct. Use rostopic echo /move_base/goal to inspect published goals.

- RViz Interactive Marker Not Updating:
  - Issue: Waypoints set via RViz might not update if the topic /clicked_point is not subscribed properly.
  - Fix: Ensure your node subscribes to /clicked_point and that RViz’s “Publish Point” tool is active. Confirm using rostopic echo /clicked_point that points are being published.

- User Input Issues:
  - Issue: Incorrect formatting of table numbers (e.g., using hyphens instead of commas) may lead to invalid inputs.
  - Fix: Follow the documented input formats and consider additional validation in the code to handle unexpected input gracefully.

## 6.2. Debugging Tools
- RViz:
  - Visualize robot pose, goal markers, costmaps, and sensor data.
  - Use interactive tools to set waypoints and inspect the environment.

- rostopic echo:
  - Monitor topics such as /move_base/goal, /clicked_point, and /tf to verify that data is being published as expected.

- rqt_graph:
  - Visualize the node and topic connections to ensure proper communication between your modules.

- roswtf:
  - Run roswtf to diagnose configuration issues in your ROS environment.

- TF Debugging Tools:
  - Use rosrun tf tf_echo <source_frame> <target_frame> to verify specific transforms.
The tf_monitor tool can provide real-time statistics on your TF tree.

# CHALLENGES and OVERCOMING THEM
## 1. TF Transform and Frame Issues
Challenge:
- The navigation stack was generating warnings such as “Timed out waiting for transform from base_footprint to map,” indicating that some required coordinate frames were either missing or misconfigured.

Solution:
- Verified the TF tree using tools like rosrun tf tf_echo and rosrun tf view_frames to ensure that the essential frames (map, odom, base_link, and base_footprint) were published correctly.
- Configured and launched AMCL/SLAM nodes appropriately so that the map frame was established, thereby satisfying the move_base requirements.
- Tuned the transform publishers to maintain a robust TF tree during simulation.
  
## 2. Navigation Goal Failures (Quaternion Errors)
Challenge:
- Warnings such as “Quaternion has length close to zero... discarding as navigation goal” were encountered because the orientation values were not being set properly. This was mainly due to hardcoded yaw values that did not translate correctly into valid quaternions.

Solution:
- Integrated the quaternion_from_euler function (or its alternative from tf_conversions) to convert yaw angles accurately into quaternions.
Ensured that the Euler to quaternion conversion was applied consistently across all navigation goals, leading to properly oriented move_base goals.
Tested with simple goals using rostopic pub to confirm that the quaternion conversion produced valid values before integrating it into the main script.

## 3. Handling User Input and Multiple Orders
Challenge:
- Parsing user input for multiple table numbers was initially error-prone, especially when users entered table numbers in different formats (e.g., "1,2,3" versus "1 2 3" or even "1-2").

Solution:
- Implemented robust input validation using regular expressions to split the input on commas or whitespace, ensuring only valid table numbers were accepted.
Added clear error messages and warnings for invalid inputs, making the system more user-friendly.
Developed comprehensive testing scenarios with different input formats to confirm that the system correctly parsed and processed the orders

## 4. Integrating RViz Point-Click Feature for Dynamic Waypoints
Challenge:
- Previously, the waypoint coordinates for the kitchen, home, and tables were hardcoded. Transitioning to an interactive approach using RViz required reliable subscription to the /clicked_point topic and proper handling of interactive markers.

Solution:
- Modified the code to subscribe to /clicked_point, capturing user-selected positions in real-time.
- Developed callback functions that update the internal configuration for waypoints dynamically, ensuring that these changes immediately reflected in navigation goals.
- Tested the RViz interactive marker tool extensively, confirming that clicking a point correctly updated the coordinates used by the move_base goal.

## 5. Debugging and System Integration
Challenge:
Coordinating multiple ROS nodes (navigation, localization, and custom scripts) can be challenging, leading to intermittent communication issues and unexpected behavior.

Solution:
Used debugging tools such as rostopic echo, rqt_graph, and roswtf to monitor inter-node communications and identify bottlenecks.
Modularized the code into clear, well-defined functions to isolate issues and facilitate targeted debugging.
Conducted iterative testing, starting with individual modules (e.g., testing navigation alone) before integrating the complete system. This ensured that each component performed reliably before full system integration.

# 7. FUTURE IMPROVEMENTS
## 7.1. Enhancements That Can Be Made
- Advanced Obstacle Avoidance:
  - Integrate sensor fusion (combining LiDAR, camera, and ultrasonic data) with machine learning algorithms to enhance obstacle detection and avoidance.
- Dynamic Path Planning:
  - Explore alternative planning algorithms (e.g., RRT* or CHOMP) that can provide more optimal or adaptive paths in highly dynamic environments.
- Improved User Interface:
  - Develop a graphical user interface (GUI) or mobile app for real-time control and monitoring, reducing reliance on command-line inputs.
- Voice Command Integration:
  - Incorporate speech recognition to allow voice commands for order delivery and control.

## 7.2. Additional Features to be Implemented
- Automated Order Management:
  - Interface the robot with a restaurant management system to automatically receive and process orders.

- Self-Diagnostics and Recovery:
  -Implement enhanced self-diagnostic routines and recovery behaviors for scenarios when the robot becomes stuck or loses localization.

- Data Logging and Analysis:
  - Develop a module to log navigation data, sensor readings, and system performance metrics for offline analysis and improvements.

- Multi-Robot Coordination:
  - Extend the system to support multiple robots working cooperatively in larger or more complex environments.
