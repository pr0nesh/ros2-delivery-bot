# ros2-delivery-bot
"Autonomous food delivery simulation using TurtleBot3 and ROS 2 Nav2 in a custom Gazebo café environment."


Project Overview
This project implements a food delivery robot system using TurtleBot3 in a custom Gazebo environment. The robot navigates autonomously using the Nav2 stack and handles real-world delivery tasks such as:
- Delivering to multiple tables
- Receiving confirmation at delivery points
- Canceling deliveries interactively
- Returning to predefined locations like the Kitchen or Home
System Architecture
1. Gazebo Simulation Setup
The simulation uses a custom Gazebo world (my_world.world).

Launch file: empty_world.launch.py  
Responsibilities:
- Launch gzserver and gzclient
- Spawn the TurtleBot3 robot
- Set initial position (x_pose, y_pose)
- Launch robot_state_publisher
2. Navigation Setup (Nav2)
Launch file: navigation2.launch.py  
Initializes the Nav2 stack using:
- Predefined map YAML file
- Parameter file based on the TurtleBot3 model
- RViz with default configuration

Enables:
- Global/local planning
- Recovery behaviors
- SLAM or localization
3. Delivery Task Handling – nav.py
Defines a ROS2 Node named restaurant_robot using BasicNavigator from nav2_simple_commander.

Features:
- Navigate to HOME, KITCHEN, TABLE1, TABLE2, TABLE3
- Multi-table support
- Confirmation with timeout
- Live feedback (distance/progress)
- Cancellations (cl, c1, c2, c3)
- Return logic
Default Locations

HOME:    {x: -1.0, y: 0.0, theta: 0.0}
KITCHEN: {x: 1.0,  y: -0.5, theta: 0.0}
TABLE1:  {x: 4.0,  y: 1.3, theta: 0.0}
TABLE2:  {x: 4.0,  y: 0.0, theta: 0.0}
TABLE3:  {x: 4.0,  y: -1.3, theta: 0.0}

Launch Instructions
1. Launch Gazebo World:
ros2 launch turtlebot3_gazebo empty_world.launch.py

2. Launch Nav2:
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true

3. Run Delivery Script:
ros2 run your_package_name nav.py
Robot Commands
Command
Description
t1
Deliver to Table 1
1,2,3 or t1,t2,t3
Multiple table delivery
c
Confirm delivery
cl
Cancel and go home
c1, c2, c3
Cancel a specific table
home
Force return to HOME
status
Check the current robot status
help
Display available commands
q
Quit the program
Flow Diagram (Simplified)

START
  └──> Spawn in Custom Gazebo World
        └──> Nav2 Initializes (Map + Params)
              └──> nav.py script starts
                    └──> Accept user command (t1, t2, ...)
                          ├──> Navigate to Kitchen
                          ├──> Confirm Pickup
                          ├──> Deliver to tables (with per-table checks)
                          └──> Return to Kitchen/Home

Key Concepts Used
- ROS 2 Nodes in Python for modular navigation and delivery logic
- Nav2 stack for path planning and obstacle avoidance
- PoseStamped goals for movement
- Threading for background initialization
- Interactive CLI for user control
- Custom world design in Gazebo




Conclusion
This assignment demonstrates a robust navigation and task-execution system using TurtleBot3 and Nav2. The robot simulates real-world café delivery behavior using ROS2 and Gazebo.
