#ifndef RR_JOYSTICK__RR_JOYSTICK_HPP_
#define RR_JOYSTICK__RR_JOYSTICK_HPP_

#include "rr_joystick/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rrobot {
namespace rr_joystick
{

class RrJoystickNode : rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RrJoystickNode(const std::string & node_name = "rr_joystick_node", bool intra_process_comms = true): 
    rclcpp_lifecycle::LifecycleNode(node_name,
    rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)) {}

  virtual ~RrJoystickNode() = default;
};

}  // namespace rr_joystick
}
#endif  // RR_JOYSTICK__RR_JOYSTICK_HPP_
