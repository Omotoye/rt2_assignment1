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

# ROS Package Description 
There are three branches in this github repository; _**master, action**_ and _**ros2**_, each of this branch controls the mobile robot is their own unique way and it would all be decribed below. 

## General Package Architecture Description 
The architecture is contained of **four** nodes; 

*   _**user_interface**_
*   _**random_position**_
*   _**state_machine**_
*   _**go_to_point**_   

### User Interface (_user_interface.py_)
This node prompt the user to enter an integer that represent either a start of the robot motion simulation of a stop of the simulation. Based on the input gotten from the user, the node sends a request of either **start** or **stop** to the state_machine node 
