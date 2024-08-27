# viam-rover.ros2cpp

A ROS 2 package for controlling the Viam Rover

Supported ROS 2 Version: Humble Hawksbill

## Instructions

### Building

* Create ros2 workspace
* Clone the repo under the src folder
* Build
    * `colcon build --executor sequential`
        * Sequential build helps prevent issues with running out system memory
    * OR `colcon build --packages-select viam_rover`
        * Building single package reduces the number of concurrent build processes

### Running

* Source the workspace
    * `source install/setup.bash`
* Run the node
    * `ros2 run viam_rover motor_control`
* Run the turtlebot3 teleop node
    * `ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=/cmd_vel`
