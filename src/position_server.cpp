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

#include <inttypes.h>
#include <memory>
#include "rt2_assignment1/srv/random_position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using RandomPosition = rt2_assignment1::srv::RandomPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{

    class RandomPositionServer : public rclcpp::Node
    {
    public:
        RandomPositionServer(const rclcpp::NodeOptions &options) : Node("random_position_server", options)
        {
            service_ = this->create_service<RandomPosition>(
                "/position_server", std::bind(&RandomPositionServer::myrandom, this, _1, _2, _3));
        }

    private:
        /**
        * \brief random number generator
        * \param M defines the minimum possible value of the random number 
        * \param N defines the maximum possible value of the random number
 *  
        * \return the random number
        * 
        * This function generates a random number between M and N 
        */

        double randMToN(double M, double N)
        {
            return M + (rand() / (RAND_MAX / (N - M)));
        }

        bool myrandom(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<RandomPosition::Request> req,
            const std::shared_ptr<RandomPosition::Response> res)
        {
            (void)request_header;
            res->x = randMToN(req->x_min, req->x_max);
            res->y = randMToN(req->y_min, req->y_max);
            res->theta = randMToN(-3.14, 3.14);
            return true;
        }
        rclcpp::Service<RandomPosition>::SharedPtr service_;
    };
}
// Registering the position server node   
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::RandomPositionServer)

//
//int main(int argc, char *argv[])
//{
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<RandomPositionServer>());
//    rclcpp::shutdown();
//    return 0;
//}
//