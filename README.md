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
This node prompts the user to enter an integer that represent either a start of the robot motion simulation or a stop of the simulation. Based on the input gotten from the user, the node sends a service request of either **start** or **stop** to the user_interface server in the state_machine node.

### Random Position (_random_position.cpp_)
This node takes in a request from the state machine with a minimum and maximum x and y coordinate message and from this request message it then generates a random position within that range and sends it as a response to the state machine. 

### State Machine (_state_machine.cpp_)
The state machine is in charge of transitioning the system state from one state to another (_i.e from start to stop and vice versa_). The state machine takes in a request of start or stop from the user_interface client and then based on the request message it then either sends a request to the random position server for a random position or it sends a request to the go to point server to stop the robot from going to some given point. If the request is to start the robot, after getting a response of a random target pose from the random position node, the state machine sends the random pose coordinates as a request message to the go to point server and waits until the service goal is completed. 

### Go To Point (_go_to_point.py_)
This node takes a service request of a target pose coordinate message from the state machine and then navigates the robot from it's current position coordinate to the required target position coordinate. 

This is the basic functions of the nodes contained in the **master** branch of this repository, for more information about the code contained in the nodes of the master branch [Click Here](https://omotoye.github.io/rt2_assignment1/ "Script code html Documentation") . The **action** and **ros2** branch makes minor changes to this nodes for some specific reason which would be described below. 

---

## Action Package Description

The action branch which can be found in this [link](https://github.com/Omotoye/rt2_assignment1/tree/action) makes some little changes to the **state_machine** and the **go_to_point** nodes. In this branch, the __service client server__ communication between the _state machine_ and the _go to point_ node is replace with an __action client and server__. This is because in the previous version contained in the __master__ branch, the service request to stop the robot's motion between random position is asynchronous _i.e the system has to wait for the robot to reach the target before the random target behaviour can be stopped_. The problem with the service implementation can be solve with an action server, this is because action servers allow their goal to be **preempted**. Therefore in this node the robot is stopped immediately after a request to stop has been given. A function was implemented in the go to point node to check if a preempt reqest has been activate, this function is checked on every loop of velocity command sent to the robot. Code snippet below. 
```python 
def check_preempt():
    """This function is used for checking if a preemption has be
    requested from the UI node. 

    Returns:
        [bool]: True if preempt is request and False otherwise. 
    """
    # check that preempt has not been requested by the client
    if _as.is_preempt_requested():
        print('The Goal has been Preempted')
        _as.set_preempted()
        done()
        return True
    return False
```

---

Also the structure of the **Position.srv** message sent between the node was change to the format required by the action by the action server, **Position.action**. 
```diff
-   float32 x
-   float32 y   #request
-   float32 theta
-   ---
-   bool ok   #response
+   float32 x
+   float32 y     #goal
+   float32 theta
+   ---
+   bool ok  #result
+   ---
+   nav_msgs/Odometry pose #feedback
```


## ROS2 Package Description

This package does exactly thesame thing that the package in the master branch does, the only difference is that it does part of it with **ros2**,  [click here](https://github.com/Omotoye/rt2_assignment1/tree/ros2) to go to the ros2 branch. The package in both the **master** and **action** branch is written specifically for **_ROS Noetic Ninjemys_**. The package in the **ros2** branch allow some parts of the simulation (**state_machine and random_position is controlled from ros2**) to be control from a *__ROS 2 Foxy Fitzroy__* package. This is made possible by a package called [ros1_bridge](https://github.com/ros2/ros1_bridge "ros1_brige"), this package helps to bridge the messages from a **ros2** package with messages in **ros** package which then enable communication between the nodes of the package. _Instructions on how to compile and launch the ros1_brige can be found in the readme contained in the ros2 branch; **all required configurations has already been made to the package**_.

