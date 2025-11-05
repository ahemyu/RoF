
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/int64.hpp"

using std::placeholders::_1; //placeholder for arguments 

class Listener: public rclcpp::Node
// Write a subscriber node named listener that subscribes to the topic "first_test_topic" and that prints
// the received value on the command line using RCLCPP_INFO(). Additionally, display the history of the value of first_test_topic using rqt.
{
  public:
    Listener() :  Node("Listener") 
    {
    subscription_ = this->create_subscription<std_msgs::msg::Int64>("first_test_topic", 10, std::bind(&Listener::topic_callback, this, _1));
    }

  private: 
  void topic_callback(const std_msgs::msg::Int64 & msg) const //we get an int message now 
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%ld'", msg.data);
  }
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
