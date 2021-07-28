<a href="https://unige.it/en/">
<img src="images/unige.png" width="20%" height="20%" title="University of Genoa" alt="University of Genoa" >
</a>


# First Assignment of the Research Track 2 course (Robotics Engineering / JEMARO, Unige)

**Author's Name: Omotoye Shamsudeen Adekoya**

**Student ID: 5066348**

---

<div align="center">
<h1> Robot Controller </h1>
<img src="images/rviz-robotmodel.png" width="50%" height="50%" title="Two Wheeled non-holonomic robot" alt="Two Wheeled non-holonomic robot" >
</div>

>This package contains the nodes and the simulation environments for controlling a mobile robot in the Gazebo simulation environment and CoppeliaSim simulation environment.

**For Information about the general architecture of this package [click here](https://github.com/Omotoye/rt2_assignment1/blob/master/README.md)**


# How to Compile and Launch the Package (*ros2 branch*)
This package needs nodes from the **master** branch and nodes from this **ros2** branch.

## Compile 
### Creating a ROS Workspace 
First you create a folder for your catkin workspace

```bash
mkdir -p ~/catkin_ws/src
```

Clone the package repository for the master branch

```bash
cd ~/catkin_ws/src
git clone https://github.com/Omotoye/rt2_assignment1.git
```

Once the package has been successfully cloned, you then build the workspace

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws/
catkin_make
```

### Creating ROS2 Workspace       

First you create a folder for your ros2 workspace

```bash
mkdir -p ~/colcon_ws/src
```
build the workspace

```bash
source /opt/ros/foxy/setup.bash
cd ~/colcon_ws
colcon build
```

Clone the package repository 

```bash
cd ~/colcon_ws/src
git clone https://github.com/Omotoye/rt2_assignment1.git
```
Checkout to the ros2 branch 

```bash
cd ~/colcon_ws/src/rt2_assignment1
git checkout ros2
```


After doing this, you create three scripts in your home directory

### script for sourcing ROS 
```sh
#!/bin/bash

source /opt/ros/noetic/setup.bash
```
save this commands in a ros.sh file, then enter the command below to give it executable prevelages

```bash
chmod +x ros.sh
```

### script for sourcing ROS2 
```sh
#!/bin/bash

source /opt/ros/foxy/setup.bash 
```
save this commands in a ros2.sh file, then enter the command below to give it executable prevelages

```bash
chmod +x ros2.sh
```

### script for sourcing ROS and ROS2 together
```sh
#!/bin/bash

source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash 
```
save this commands in a ros12.sh file, then enter the command below to give it executable previleges

```bash
chmod +x ros12.sh
```
## Setting up ros1_bridge 
Step 1
```bash
# Open a Linux shell
source ros2.sh
cd ~/colcon_ws/src 
# Clone the ros1_bridge repository
git clone https://github.com/ros2/ros1_bridge.git
cd ~/colcon_ws
# Compile the rt2_assignment1 ros2 package that is already cloned and exclude the ros1_bridge package
colcon build --symlink-install --packages-skip ros1_bridge
```
Step 2: This is to compile the ros1_bridge to bride similar messages based on the configuration already setup by the mapping rule contained in the package. **_This is going to take some time, so you have to be patient_**
```bash
#Open a new Linux shell
source ros12.sh # ignore the warning
cd ~/colcon_ws
colcon build --packages-select ros1_bridge --cmake-force-configure
```

## Launch
After the compilation has finished, run the command below to install a package required to run a script i have prepared to launch to simulation. 
```bash 
sudo apt update  
sudo apt install gnome-shell  
```
Enter the command below to launch the simulation.   
```bash
cd ~/colcon_ws/src/rt2_assignment1/launch 
cp launch.sh ~
cd ~
chmod +x launch.sh # just to be sure it has executable permission
./launch.sh 
```
If for some reasons the script does not perform properly and you want to launch it manually, follow the steps below. 

**Launch three shells and follow the instructions corresponding to each of the shells** 
 
Shell 1
```bash 
source ros.sh 
roslaunch rt2_assignment1 ros2_sim.launch
```
Shell 2 
```bash 
source ros12.sh 
ros2 run ros1_bridge dynamic_bridge
```
Shell 3 
```bash
source ros2.sh
ros2 launch rt2_assignment1 sim_container.py
```


