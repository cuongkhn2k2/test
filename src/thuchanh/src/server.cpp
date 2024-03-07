#include "rclcpp/rclcpp.hpp"
#include "type_custom/srv/operator_two_ints.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
using std::placeholders::_2;

class OperatorTwoIntsServer : public rclcpp::Node
{
public:
    OperatorTwoIntsServer();
    //~OperatorTwoIntsServer();
 
    void set_operator(char ope);
 
private:
    char _rq_operator = '+';
    std::string _prm_operator = "operator";
 
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr _response_pub;
 
    rclcpp::Service<type_custom::srv::OperatorTwoInts>::SharedPtr _server;
    size_t count_;
    void operator_two_ints_callback(const type_custom::srv::OperatorTwoInts::Request::SharedPtr req,
                                    const type_custom::srv::OperatorTwoInts::Response::SharedPtr res);
};

OperatorTwoIntsServer::OperatorTwoIntsServer() : Node("server"), count_(0)
{
    _server = this->create_service<type_custom::srv::OperatorTwoInts>("operator_two_ints",
                                                                      std::bind(&OperatorTwoIntsServer::operator_two_ints_callback, this, _1, _2));
 
    _response_pub = this->create_publisher<std_msgs::msg::Int32>("response", 10);
    RCLCPP_INFO(this->get_logger(), "Server started!!");
}

void OperatorTwoIntsServer::operator_two_ints_callback(const type_custom::srv::OperatorTwoInts::Request::SharedPtr req,
                                                       const type_custom::srv::OperatorTwoInts::Response::SharedPtr res)
{
    _rq_operator = this->get_parameter(_prm_operator).as_int();
 
    switch (_rq_operator)
    {
    case '+':
        res->result = req->a + req->b;
        break;
    case '-':
        res->result = req->a - req->b;
        break;
    case '*':
        res->result = req->a * req->b;
        break;
    case '/':
        res->result = req->a / req->b;
        break;
    default:
        {
            res->result = req->a + req->b;
            _rq_operator = '+';
        }
        break;
    }
 
    std_msgs::msg::Int32 msg;
    msg.data = res->result;
    _response_pub->publish(msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                req->a, req->b);
    auto message = std_msgs::msg::String();
    message.data = "Sending back response[" + std::to_string(count_++)+"]: ";
    RCLCPP_INFO(this->get_logger(), "%s%ld %c %ld = %ld", message.data.c_str(), req->a, _rq_operator, req->b, res->result);
}

void OperatorTwoIntsServer::set_operator(char ope)
{
    this->declare_parameter(this->_prm_operator, ope);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
 
    auto node = std::make_shared<OperatorTwoIntsServer>();
    node->set_operator((char)*argv[1]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}