# ROB 599 Project: Narrow Corridor Navigation

Michael Pfeiffer and Siddarth Viswanathan

## Install:
This project uses **ROS 2 Jazzy Jalisco** and **Python 3.12.3**

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
cd into the directory:
``` bash
cd ROB599_NarrowCorridorNavigation/src
```

Run the following commands to create the virtual environment for the interface:
```
python3 -m venv .py_env
source .py_env/bin/activate
pip install -r requirements.txt 
cd ..
```

Build the code in the same terminal:
```bash
colcon build
source install/setup.bash
```

## Run:

In the terminal with the active virtual env, display the interface with the command: 
```bash
ros2 launch interface_pkg _launch.py
```

Create another terminal without the venv and execute the following commands: 
```bash
source install/setup.bash
ros2 launch robot_pkg driving_sim.launch.py
```

This will display the Gazebo simulator. Press the orange play button in the bottom left corner of the simulator to start the sim.

Go to the interface and control the robot from there!