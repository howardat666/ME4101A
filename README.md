# Motion Planning for Mobile Manipulator Placement

- **Name:** Luo Zhonghao
- **Email:** luo_zhonghao@u.nus.edu

---

## Table of Contents

- [Project Description](#project-description)
- [Package Dependencies](#package-dependencies)
- [Project Structure](#project-structure)
- [How to Run](#how-to-run)
- [Notes](#notes)

---

## Project Description

This project is mainly used to simulate the motion control of current mobile robots on ROS.

This repository contains the description file, configuration file, launch file, control code, and plug-ins for the
mobile robot.

---

## Package Dependencies

This project relies on the following ROS packages and Python libraries:

### ROS Packages

- `rospy`
- `std_msgs`
- `sensor_msgs`
- `nav_msgs`
- `geometry_msgs`
- `tf` (transformations)

### Python Libraries

- `numpy`
- `matplotlib`
- `tkinter`
- `scipy`
- `simple-pid`

### Installation Example

#### Python Libraries

```bash
pip install numpy matplotlib scipy simple-pid
```

#### ROS Packages

(Usually included in ROS installation. If missing, install manually)

```bash
sudo apt-get install ros-<ros_distro>-<package_name>
```

Replace `<ros_distro>` with your ROS version (e.g., `noetic`, `melodic`).

---

## Project Structure

```
mobile_base/
│
├─ basic_control.py                 # Use IK to control the robot
├─ joystick.py                      # Control the robot with joystick
├─ mu_gui.py                        # Change mu value with GUI
├─ pid_control.py                   # Control the robot with PID
├─ read_3v.py                       # Read the vx,vy,w of the robot from sensors
├─ read_wheel_speed.py              # Read the wheel speed from encoders
└─ README.md                        # Project documentation
```

---

## How to Run

### 0. Load Plugin (Optional: Adjust $\mu$ Value)

If you want to adjust the friction coefficient ($\mu$), you need to load the custom dynamic friction plugin.

- Copy `libdynamic_friction_plugin.so` to the following directory:

```bash
sudo cp libdynamic_friction_plugin.so /usr/lib/x86_64-linux-gnu/gazebo-11/plugins/
```

### 1. Build Workspace

Navigate to your catkin workspace and compile:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch ROS and Gazebo

Start the launch file to initialize Gazebo and necessary ROS topics:

```bash
roslaunch arc_mobile_base gazebo.launch
```

Start the launch file to enable visual odometer (Optional):

```bash
roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch 
```

### 3. Run Python Scripts

Navigate to your script directory and run the desired Python scripts. For example:

```bash
rosrun arc_mobile_base basic_control.py
```

---

## Notes

- If the joystick fails to work, please check whether the communication between the joystick and the system is normal.
- It is recommended to restart Gazebo after closing the control-related Python code to avoid communication issues caused
  by time rewind.
