/**
* \file state_machine.cpp
* \brief This file implement a state machine that start and stops a go to point goal
* \author Omotoye Shamsudeen Adekoya
* \version 0.01
* \date 20/07/2021
*
* \param start boolean to know if to start of stop the go to point action
*
* \details
*
*
* Services : <BR>
* ° /user_interface
* ° /position_server
* 
* Action : <BR>
*   /go_to_point
*
* Description :
*

* This node acts as a state machine for requesting the goal action server to 
* a point received from the user_interface client or cancel target based on the 
* request sent from the user_interface  
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/Command.h"
#include "rt2_assignment1/Position.h"
#include "rt2_assignment1/RandomPosition.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <rt2_assignment1/PositionAction.h>

bool start = false; ///< For setting the value of the request from the user interface


/**
* \brief callback function for handling the request sent from the user interface
* \param req the request sent from the client
* \param res the response to be sent from the server to the client 
* \return always true as this function cannot fail.
*
* This function receives the request sent from the user interface client and set the value
* the start global variable. 
*
*/
bool user_interface(rt2_assignment1::Command::Request &req, rt2_assignment1::Command::Response &res)
{
    if (req.command == "start")
    {
        start = true;
    }
    else
    {
        start = false;
    }
    return true;
}

/**
 * \brief main function 
 * \param argc 
 * \param argv
 
 * \return always 0 
 * 
 * The main funtion initializes the node, service and action client object and waits to receive a request to 
 * the initialized service.  
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_machine");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/user_interface", user_interface);
    ros::ServiceClient client_rp = n.serviceClient<rt2_assignment1::RandomPosition>("/position_server");
    actionlib::SimpleActionClient<rt2_assignment1::PositionAction> ac("go_to_point", true);

    rt2_assignment1::RandomPosition rp;
    rt2_assignment1::PositionGoal goal;

    rp.request.x_max = 5.0;
    rp.request.x_min = -5.0;
    rp.request.y_max = 5.0;
    rp.request.y_min = -5.0;

    while (ros::ok())
    {
        ros::spinOnce();
        if (start)
        {
            client_rp.call(rp);
            goal.x = rp.response.x;
            goal.y = rp.response.y;
            goal.theta = rp.response.theta;

            ac.waitForServer(); //will wait for infinite time
            std::cout << "\nGoing to the position: x= " << goal.x << " y= " << goal.y << " theta = " << goal.theta << std::endl;
            ac.sendGoal(goal);
            while (true)
            {
                ros::spinOnce();
                if (start == false)
                {
                    ac.cancelGoal();
                    std::cout << "\nThe Goal has been Cancelled" << std::endl;
                    break;
                }
                else
                {
                    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        std::cout << "\nGoal Reached!" << std::endl;
                        break;
                    }
                }
            }
        }
    }
    return 0;
}
