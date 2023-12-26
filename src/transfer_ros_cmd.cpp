
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <unistd.h>
#include <cmath>

#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

class CmdVelTransfer : public rclcpp::Node
{
public:
    CmdVelTransfer() : Node("transfer_cmd")
    {
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        cmdVelSub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&CmdVelTransfer::cmdVelCallback, this, std::placeholders::_1));
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // Process the received cmd_vel message
        // TODO: Add your code here
        RCLCPP_INFO(this->get_logger(), "Received cmd_vel message: linear.x = %f, linear.y = %f", msg->twist.linear.x, msg->twist.linear.y);
        sport_req.Move(req, msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
        // // Publish request messages
        req_puber->publish(req);
    }

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmdVelSub_;
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CmdVelTransfer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
