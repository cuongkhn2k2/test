#include "rclcpp/rclcpp.hpp"
#include "type_custom/srv/operator_two_ints.hpp"
#include "type_custom/msg/num.hpp"
#include "std_msgs/msg/int32.hpp"

#include <cstdlib>

using namespace std::chrono_literals;

class OperatorTwoIntsClient : public rclcpp::Node
{
public:
    OperatorTwoIntsClient(int num1, int num2);
    ~OperatorTwoIntsClient(){}
private:
    void callOperatorTwoInts();
    void responseCallback(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Client<type_custom::srv::OperatorTwoInts>::SharedPtr client_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr response_sub_;
    
    int num1_;
    int num2_;
};

OperatorTwoIntsClient::OperatorTwoIntsClient(int num1, int num2) : Node("client"), num1_(num1), num2_(num2)
{
    client_ = create_client<type_custom::srv::OperatorTwoInts>("operator_two_ints");
    response_sub_ = create_subscription<std_msgs::msg::Int32>("response", 10, std::bind(&OperatorTwoIntsClient::responseCallback, this, _1));
}

void OperatorTwoIntsClient::callOperatorTwoInts()
{
    while (!client_->wait_for_service(1s)) {
        RCLCPP_INFO(get_logger(), "Waiting for service...");
    }

    auto request = std::make_shared<type_custom::srv::OperatorTwoInts::Request>();
    request->a = num1_;
    request->b = num2_;

    auto future = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "Service call failed");
        return;
    }

    auto result = future.get();
    RCLCPP_INFO(get_logger(), "Response: %d", result->sum);
}

void OperatorTwoIntsClient::responseCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
    RCLCPP_INFO(get_logger(), "Response: %d", msg->data);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Usage: %s <num1> <num2>", argv[0]);
        return 1;
    }

    auto num1 = std::atoi(argv[1]);
    auto num2 = std::atoi(argv[2]);

    auto node = std::make_shared<OperatorTwoIntsClient>(num1, num2);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
