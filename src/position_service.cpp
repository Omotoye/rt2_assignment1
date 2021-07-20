/**
* \file position_service.cpp
* \brief This file implements a position service
* \author Omotoye Shamsudeen Adekoya
* \version 0.01
* \date 20/07/2021
*
* \details
*
* Services : <BR>
*      \posiiton_server
*
* Description :
*
* This node advertises a position service. When the service is required, a request containing
* minimum and maximum values for the x and y position is used to generate a random position 
* between x (or y) min and x (or y) max.
*
*/

#include "ros/ros.h"
#include "rt2_assignment1/RandomPosition.h"

/**
 * \brief random number generator
 * \param M defines the minimum possible value of the random number 
 * \param N defines the maximum possible value of the random number
  
 * \return the random number
 * 
 * This function generates a random number between M and N 
 */

double randMToN(double M, double N)
{
    return M + (rand() / (RAND_MAX / (N - M)));
}

bool myrandom(rt2_assignment1::RandomPosition::Request &req, rt2_assignment1::RandomPosition::Response &res)
{
    res.x = randMToN(req.x_min, req.x_max);
    res.y = randMToN(req.y_min, req.y_max);
    res.theta = randMToN(-3.14, 3.14);
    return true;
}

/**
 * \brief main function 
 * \param argc 
 * \param argv
 
 * \return always 0 
 * 
 * The main funtion initializes the node and service object and waits to receive a request to 
 * the initialized service.  
 */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_position_server");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("/position_server", myrandom);
    ros::spin();

    return 0;
}