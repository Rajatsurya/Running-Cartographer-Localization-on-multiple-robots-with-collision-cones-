# Running-Cartographer-Localization-on-multiple-robots-with-collision-cones-Setup Instructions

## Connecting to Robots via SSH

### Robot 1 (tb3_0 - LDS-01)
```bash
ssh ubuntu@192.168.0.133 (IP will be different find it by typing ifconfig on robots terminal)
```

### Robot 2 (tb3_1 - LDS-02)
```bash
ssh ubuntu@192.168.0.156
```

> **Note**: IP addresses may change. Ensure the IP in the `.bashrc` file matches the laptop's IP that the robots are connected to.

---

## Robot Commands

### Robot 1: tb3_0
From the robot's terminal, run the following command:
```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_robot_carto.launch multi_robot_name="tb3_0" set_lidar_frame_id:="tb3_0/imu_link"
```

### Robot 2: No Namespace
From the robot's terminal, run the following command:
```bash
roslaunch turtlebot3_bringup turtlebot3_robot_carto.launch set_lidar_frame_id:="imu_link"
```

---

## Commands on the PC

### Terminal 1: Start ROS Core
Run the ROS core:
```bash
roscore
```

### Terminal 2: Launch Remote Configuration for `tb3_0`
```bash
ROS_NAMESPACE=tb3_0 roslaunch turtlebot3_bringup turtlebot3_remote_turtle_multi.launch
```

### Terminal 3: Launch Remote Configuration for Tortoise
```bash
roslaunch turtlebot3_bringup turtlebot3_remote_tortoise_config_pbstream.launch
```

### Terminal 4: Launch Navigation Stack
Run the navigation stack to control all robots on the same map:
```bash
roslaunch ms_jackal_demo turtle_real_two_carto.launch
```

### Terminal 5: Run Collision Cone Publisher (Real Carto)
```bash
rosrun collision_cone Collision_cone_pub_real_carto_only_poly.py
```

### Terminal 6: Run Collision Cone Publisher for `tb3_0`
```bash
rosrun collision_cone Collision_cone_pub_tb3_0_carto_only_poly.py
```

### Terminal 7: Run Velocity Angle Publisher
```bash
rosrun collision_cone velocity_angle_only_poly.py
```

---

## Notes
1. Verify all IP addresses and namespaces are correctly set before running the commands.
2. Ensure `.bashrc` on the robots and PC reflects the correct network configurations.
3. Use RViz to visualize and debug the robot's navigation and TF frames.
4. Ensure rosbridge is running for integration with HoloLens.
