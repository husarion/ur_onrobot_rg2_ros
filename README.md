# ur_onrobot_rg2_ros
ROS packages for OnRobot RG2 gripper

# Examples

To run the ROS packages provided in this repository using a Docker configuration, please refer to the examples available in [ur-docker](https://github.com/husarion/ur-onrobot-rg2-docker) repo.

## ROS Nodes

### rg2_driver.py

Interim proposal for Onrobot RG2 gripper ROS controller

#### Publishes

- `/joint_states` [*sensor_msgs/JointState*]: the current state of the gripper joint.

#### Subscribes

- `/ur_hardware_interface/io_states` [*ur_msgs/IOStates*]: observes state of UR robot IO. Joint state is calculated based on `analog_out_1`. 

#### Services advertised

- `rg2/set_gripper_width` [*onrobot_rg2_driver/GripperState*]: Adjust the width of the RG2 gripper within a range of 0-102 mm by inputting a float value between 0.0-1.0.
