#include <chrono> //time utilities 
#include <functional> // lambdas and stuff
#include <memory> // smart pointers
#include "rclcpp/rclcpp.hpp" // core ros2 lib (Node class, publishing, subscribing etc)
#include "std_msgs/msg/int64.hpp" // string message (ros message that is)

using namespace std::chrono_literals; // adds user-defined literals for time durations.

class Publisher : public rclcpp::Node {
//TODO: implement the constructor
public: 
  Publisher(): Node("MyPublisher"), counter_(0)
  {
    // Create a Publisher node named Talker in there,
    // which publishes an "int64" message (from the ROS package "std_msgs") under the topic "first_test_topic" with 20 Hz, counting up from 0 to 50 and
    // publishing the value series over and over again.
    publisher_ = this->create_publisher<std_msgs::msg::Int64>("first_test_topic", 10); //TODO: don't know what the 10 does   
      
    timer_ = this->create_wall_timer(
      50ms, std::bind(&Publisher::timer_callback, this)); // create a timer that fires every 500 milliseconds, and when it fires, call the timer_callback() method on this object instance.

  }
  private:
    void timer_callback(){
      // TODO: in this function we need to just send the message with the counter until it reaches 50, once it reaches 50 we just reset it to 0 and start again
      auto message = std_msgs::msg::Int64();
      message.data = counter_++;
      publisher_->publish(message);

      if(counter_ == 51){
        counter_ = 0;
      }
    }
    
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_; 
    size_t counter_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv); //init ros
  rclcpp::spin(std::make_shared<Publisher>()); //spin calls call callback functions
  rclcpp::shutdown();
  return 0;
}
