# ROB 599 Project: Narrow Corridor Navigation

Michael Pfeiffer and Siddarth Viswanathan

## Install + Run:
Download gazebo using the following commands:
```bash
sudo apt-get install ros-jazzy-ros-gz
sudo apt install ros-jazzy-gz-tools-vendor
sudo apt install ros-jazzy-gz-sim-vendor
```
Confirm that it works by running `gz-sim`, and refer to https://gazebosim.org/docs/latest/ros_installation/ for troubleshooting. 

Clone the repo:
``` bash
git clone https://github.com/pfeiffmi/ROB599_NarrowCorridorNavigation.git
```
Cd into the directory
``` bash
cd ROB599_NarrowCorridorNavigation
```
Build the code
```bash
colcon build
source install/setup.bash
```
Create 2 terminals. 
Run the following in the first terminal and press play in the bottom left corner:
```bash
ros2 launch robot_pkg driving_sim.launch.py
```
In the other terminal, send the following command to move the car:
```bash
ros2 action send_goal /drive_until_wall robot_pkg_interfaces/action/DriveUntilWall "{stop_distance: 1.0, forward_speed: 2.0}"