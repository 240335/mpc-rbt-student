//#include <chrono>
//#include <functional>
//#include <memory>
//#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class My_Node_Pub : public rclcpp::Node
{
  public:
    My_Node_Pub()
    : Node("My_Node"), count_(0)
    {
    	//subscriber - Potreba si pohlidat datove typy + prepocet, jinak spravne
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "battery_voltage", 10, std::bind(&My_Node_Pub::topic_callback, this, _1));
      
    	//publisher
      publisher_ = this->create_publisher<std_msgs::msg::String>("battery_percentage", 10);
    }

  private:
    	//subscriber
    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) const
    {
    	//publisher
      auto message = std_msgs::msg::Float32();
      message.data = "My_Node:"; //+ msg->data.c_str();
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<My_Node_Pub>());
  rclcpp::shutdown();
  return 0;
}
