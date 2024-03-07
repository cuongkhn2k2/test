#include "rclcpp/rclcpp.hpp"
#include "type_custom/srv/operator_two_ints.hpp"
#include "type_custom/msg/num.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <thread>
#include <cstdlib>
using std::placeholders::_1;
using namespace std::chrono_literals;

class OperatorTwoIntsClient : public rclcpp::Node
{
public:
    OperatorTwoIntsClient(int num1, int num2);
    ~OperatorTwoIntsClient(){}
 
    void call_operator_two_ints(type_custom::msg::Num a, type_custom::msg::Num b);
    
private:
    std::thread _thread;
    rclcpp::TimerBase::SharedPtr _timer;
    
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _response_sub;
    void response_callback(const std_msgs::msg::Int32 &msg);
 
    rclcpp::Client<type_custom::srv::OperatorTwoInts>::SharedPtr _client;
    void operator_two_ints_request(type_custom::msg::Num a, type_custom::msg::Num b);
    void timer_callback();
    int num1_;
    int num2_;
    size_t count_;
};

OperatorTwoIntsClient::OperatorTwoIntsClient(int num1, int num2) : Node("client"), num1_(num1), num2_(num2)
{
    count_ = 0;
    _timer = this->create_wall_timer(1000ms, std::bind(&OperatorTwoIntsClient::timer_callback, this));
    _response_sub = this->create_subscription<std_msgs::msg::Int32>("response", 10, std::bind(&OperatorTwoIntsClient::response_callback, this, _1));
}

void OperatorTwoIntsClient::operator_two_ints_request(type_custom::msg::Num , type_custom::msg::Num )
{
    rclcpp::Client<type_custom::srv::OperatorTwoInts>::SharedPtr client = this->create_client<type_custom::srv::OperatorTwoInts>("operator_two_ints");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(this->get_logger(), "Waiting..!");
    }
    auto request = std::make_shared<type_custom::srv::OperatorTwoInts::Request>();
    request->a = num1_;
    request->b = num2_;
 
    auto f = client->async_send_request(request);
    try
    {
        auto response = f.get();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}
void OperatorTwoIntsClient::call_operator_two_ints(type_custom::msg::Num a, type_custom::msg::Num b)
{
    std::thread thread = std::thread(std::bind(&OperatorTwoIntsClient::operator_two_ints_request, this, a, b));
    thread.detach();
}

void OperatorTwoIntsClient::timer_callback()
{
    type_custom::msg::Num a, b;
    a.num = num1_;
    b.num = num2_;
    this->call_operator_two_ints(a, b);
}

void OperatorTwoIntsClient::response_callback(const std_msgs::msg::Int32 &msg)
{
    auto message = std_msgs::msg::String();
    message.data = "Result[" + std::to_string(count_++)+"]: ";
    RCLCPP_INFO(this->get_logger(), "%s %d", message.data.c_str(),msg.data);
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