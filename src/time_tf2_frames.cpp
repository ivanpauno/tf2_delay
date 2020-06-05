#include <memory>
#include <string>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#include <tf2_ros/transform_listener.h>

/** This node will just keep making TF requests at the current time and log
    how long of a delay there is before messages actually arrive.
*/


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf_monitor");

  const int TF2_BUFFER_TIME_SECONDS = 60;
  tf2::Duration cache_time = tf2::durationFromSec(TF2_BUFFER_TIME_SECONDS);
  tf2_ros::Buffer tfListenerBuffer(nh->get_clock(), cache_time);

  const bool spinThread = false;
  tf2_ros::TransformListener transformListener(tfListenerBuffer,
                                               nh,
                                               spinThread);

  geometry_msgs::msg::TransformStamped xform;

  rclcpp::Rate rate(10.0);
  bool waiting = false;
  tf2::TimePoint time;
  while(rclcpp::ok())
  {
    rclcpp::spin_some(nh);

    if (!waiting) // Try the current time, otherwise keep trying the old time that failed
      time = tf2_ros::fromRclcpp(nh->get_clock()->now());

    try {
      xform = tfListenerBuffer.lookupTransform("right_wheel", "chassis", time);

      if (waiting) // Record how long the delay was
      {
        tf2::TimePoint arriveTime = tf2_ros::fromRclcpp(nh->get_clock()->now());
        tf2::Duration  timeDiff   = arriveTime - time;
        std::string s = tf2::displayTimePoint(arriveTime);
        RCLCPP_INFO(nh->get_logger(),
                    "T=%s TF Arrival delay = %lf!", s.c_str(), tf2::durationToSec(timeDiff));
        waiting = false;
      }
    }
    catch (const tf2::ExtrapolationException& ex) {
      std::string s = tf2::displayTimePoint(tf2_ros::fromRclcpp(nh->get_clock()->now()));
      std::string errorMsg = ex.what();
      if (errorMsg.find("earliest data is at time") != std::string::npos)
      {
        RCLCPP_ERROR(nh->get_logger(),
                   "T=%s %s  Data too early error!!", s.c_str(), ex.what());
      }
      else // Should be the "interpolate into the future" error
      {
        RCLCPP_ERROR(nh->get_logger(),
                   "T=%s %s  Will wait and see how long the data takes to arrive!", s.c_str(), ex.what());
        waiting = true;
      }
    }
    catch (const tf2::TransformException& ex) {
      std::string s = tf2::displayTimePoint(tf2_ros::fromRclcpp(nh->get_clock()->now()));
      RCLCPP_ERROR(nh->get_logger(), "T=%s %s  Unknown error!", s.c_str(), ex.what());
    }
    catch (...) {
      RCLCPP_ERROR(nh->get_logger(), "another error");
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
