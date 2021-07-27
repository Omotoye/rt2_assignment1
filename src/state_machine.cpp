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
* Service : <BR>
*   /go_to_point
*
* Description :
*
* This node acts as a state machine for requesting the go to point server to go
* a point received from the user_interface client or cancel target based on the 
* request sent from the user_interface  
*
*/

#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

using namespace std::chrono_literals;
using Command = rt2_assignment1::srv::Command;
using RandomPosition = rt2_assignment1::srv::RandomPosition;
using Position = rt2_assignment1::srv::Position;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

namespace rt2_assignment1
{
    class StateMachine : public rclcpp::Node
    {
    public:
        bool pose_ready = false;
        StateMachine(const rclcpp::NodeOptions &options) : Node("state_machine", options)
        {
            service_ = this->create_service<Command>(
                "/user_interface", std::bind(&StateMachine::user_interface, this, _1, _2, _3));

            client1_ = this->create_client<RandomPosition>("/position_server");
            // while (!client1_->wait_for_service(std::chrono::seconds(1)))
            // {
            //     if (!rclcpp::ok())
            //     {
            //         RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            //         return;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            // }

            client2_ = this->create_client<Position>("/go_to_point");
            // while (!client2_->wait_for_service(std::chrono::seconds(1)))
            // {
            //     if (!rclcpp::ok())
            //     {
            //         RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
            //         return;
            //     }
            //     RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            // }
            timer1_ = this->create_wall_timer(
                2000ms, std::bind(&StateMachine::call_server1, this));
            timer2_ = this->create_wall_timer(
                2000ms, std::bind(&StateMachine::call_server2, this));
        }
        void call_server1()
        {
            this->request_1 = std::make_shared<RandomPosition::Request>();
            this->response_1 = std::make_shared<RandomPosition::Response>();

            using ServiceResponseFuture =
                rclcpp::Client<RandomPosition>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future)
            {
                this->response_1 = future.get();
                pose_ready = true;
            };

            this->request_1->x_max = 5.0;
            this->request_1->x_min = -5.0;
            this->request_1->y_max = 5.0;
            this->request_1->y_min = -5.0;
            if (start == true)
            {
                auto future_result = client1_->async_send_request(this->request_1, response_received_callback);
            }
        }
        void call_server2()
        {
            this->request_2 = std::make_shared<Position::Request>();
            this->response_2 = std::make_shared<Position::Response>();

            using ServiceResponseFuture =
                rclcpp::Client<Position>::SharedFuture;
            auto response_received_callback = [this](ServiceResponseFuture future)
            {
                this->response_2 = future.get();
            };

            this->request_2->x = this->response_1->x;
            this->request_2->y = this->response_1->y;
            this->request_2->theta = this->response_1->theta;
            if (start == true && pose_ready == true)
            {
                pose_ready = false;
                std::cout << "\nGoing to the position: x= " << this->request_2->x << " y= " << this->request_2->y << " theta = " << this->request_2->theta << std::endl;
                auto future_result = client2_->async_send_request(this->request_2, response_received_callback);
            }
        }

    private:
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
        bool user_interface(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<Command::Request> req,
            const std::shared_ptr<Command::Response> res)
        {
            (void)request_header;

            if (req->command == "start")
            {
                start = true;
            }
            else
            {
                start = false;
            }
            return true;
        }

        rclcpp::Client<RandomPosition>::SharedPtr client1_;
        rclcpp::Client<Position>::SharedPtr client2_;
        rclcpp::Service<Command>::SharedPtr service_;

        rclcpp::TimerBase::SharedPtr timer1_;
        rclcpp::TimerBase::SharedPtr timer2_;

        std::shared_ptr<RandomPosition::Request> request_1;
        std::shared_ptr<Position::Request> request_2;
        std::shared_ptr<RandomPosition::Response> response_1;
        std::shared_ptr<Position::Response> response_2;
    };
}
// Registering the State Machine Node 
RCLCPP_COMPONENTS_REGISTER_NODE(rt2_assignment1::StateMachine)

//int main(int argc, char *argv[])
//{
//    rclcpp::init(argc, argv);
//    auto node = std::make_shared<StateMachine>();
//    node->call_client1();
//    node->call_client2();
//    rclcpp::spin(node);
//    rclcpp::shutdown();
//    return 0;
//}
