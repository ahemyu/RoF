#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
using namespace std::chrono_literals;
class SimpleTfBroadcaster : public rclcpp::Node
{
public:
  SimpleTfBroadcaster()
      : Node("my_tf_broadcaster"), count_(0), angle_z_deg_(0), angle_z_rad_(0)
  {
    tf_broadcaster1_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_broadcaster2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(100ms, std::bind(&SimpleTfBroadcaster::timer_callback, this));
  }

private:
  void timer_callback()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now(); //valid as of now
    t.header.frame_id = "world"; //world frame is standard
    t.child_frame_id = "frame_1"; // the frame we want to describe 
    t.transform.translation.x = 1.0; //frame_1 is offset by 1 in X direction 
    t.transform.translation.y = 0; //same
    t.transform.translation.z = 0; //same 
    angle_z_deg_ = 45;
    angle_z_rad_ = angle_z_deg_ / 180.0 * M_PI;//radians
    tf2::Quaternion q;
    q.setRPY(0, 0, angle_z_rad_); //arguments: roll(rotation x axis), pitch(rotation y axis), yaw (rotation z axis)
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster1_->sendTransform(t);
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "frame_1";
    t.child_frame_id = "frame_2";
    t.transform.translation.x = 0;
    t.transform.translation.y = 0;
    t.transform.translation.z = 1.0;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster2_->sendTransform(t);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster1_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2_;
  int16_t count_;
  double_t angle_z_deg_;
  double_t angle_z_rad_;
};
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}