
#include <chrono> //time utilities 
#include <functional> // lambdas and stuff
#include <memory> // smart pointers
#include <string> 

#include "rclcpp/rclcpp.hpp" // core ros2 lib (Node class, publishing, subscribing etc)
#include "my_pubsub_package_interfaces/msg/num.hpp" // custom Num message

using namespace std::chrono_literals; // adds user-defined literals for time durations.

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node //inherits from Node class
{
public:
  MinimalPublisher(): Node("minimal_publisher"), count_(0) //definition of the constructor of MinimalPublisher class, this is the initilaizer list for init of simple member vars
  {
    // body of the constructor, here we can use this now as it is already constructed (and thus memory safe)
    publisher_ = this->create_publisher<my_pubsub_package_interfaces::msg::Num>("topic", 10); //underscore means it is a member variable (Attribute) of the class
    timer_ = this->create_wall_timer(
      2000ms, std::bind(&MinimalPublisher::timer_callback, this)); // create a timer that fires every 500 milliseconds, and when it fires, call the timer_callback() method on this object instance.
  }

private:
  void timer_callback()
  {
    auto message = my_pubsub_package_interfaces::msg::Num();
    message.num = count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%ld'", message.num);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_; //allocate the memory such that a value can be assigned to it in the constructor
  rclcpp::Publisher<my_pubsub_package_interfaces::msg::Num>::SharedPtr publisher_; //same here
  size_t count_; //same here
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //init ros
  rclcpp::spin(std::make_shared<MinimalPublisher>()); //spin calls call callback functions
  rclcpp::shutdown();
  return 0;
}
