#include <inttypes.h>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rt2_assignment1/srv/position.hpp"
#include "rt2_assignment1/srv/command.hpp"
#include "rt2_assignment1/srv/random_position.hpp"

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
        StateMachine(const rclcpp::NodeOptions &options) : Node("random_position_server", options)
        {
            service_ = this->create_service<Command>(
                "/position_server", std::bind(&StateMachine::user_interface, this, _1, _2, _3));

            client1_ = this->create_client<RandomPosition>("/position_server");
            while (!client1_->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            }

            client2_ = this->create_client<Position>("/go_to_point");
            while (!client2_->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
            }
            this->request_1 = std::make_shared<RandomPosition::Request>();
            this->request_2 = std::make_shared<Position::Request>();
            this->response_1 = std::make_shared<RandomPosition::Response>();
            this->response_2 = std::make_shared<Position::Response>();
        }
        void call_client1()
        {
            request_1->x_max = 5.0;
            request_1->x_min = -5.0;
            request_1->y_max = 5.0;
            request_1->y_min = -5.0;

            auto result_future = client1_->async_send_request(request_1);
            this->response_1 = result_future.get();
        }
        void call_client2()
        {
            request_2->x = this->response_1->x;
            request_2->y = this->response_1->y;
            request_2->theta = this->response_1->theta;
            std::cout << "\nGoing to the position: x= " << request_2->x << " y= " << request_2->y << " theta = " << request_2->theta << std::endl;
            auto result_future = client2_->async_send_request(request_2);
            this->response_2 = result_future.get();
        }

    private:
        bool start = false;

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

        std::shared_ptr<RandomPosition::Request> request_1;
        std::shared_ptr<Position::Request> request_2;
        std::shared_ptr<RandomPosition::Response> response_1;
        std::shared_ptr<Position::Response> response_2;

        rclcpp::Client<RandomPosition>::SharedPtr client1_;
        rclcpp::Client<Position>::SharedPtr client2_;
        rclcpp::Service<Command>::SharedPtr service_;
    };
}

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
