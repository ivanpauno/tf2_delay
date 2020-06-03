#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>


/** Simple node for testing with a published TF message
*/


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tf_publisher");

  tf2_ros::TransformBroadcaster broadcaster(nh);

  geometry_msgs::msg::TransformStamped tfStamp;

  tfStamp.transform.translation.x = 0;
  tfStamp.transform.translation.y = 0;
  tfStamp.transform.translation.z = 0;

  tfStamp.transform.rotation.x = 0;
  tfStamp.transform.rotation.y = 0;
  tfStamp.transform.rotation.z = 0;
  tfStamp.transform.rotation.w = 1.0;

  tfStamp.header.frame_id = "foo"; // This is the parent in the node tree
  tfStamp.child_frame_id  = "bar";

  rclcpp::Rate rate(10.0);
  while(rclcpp::ok())
  {
    rclcpp::spin_some(nh);

    broadcaster.sendTransform(tfStamp);

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
