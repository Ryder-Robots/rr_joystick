// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef RR_JOYSTICK__RR_JOYSTICK_HPP_
#define RR_JOYSTICK__RR_JOYSTICK_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rr_joystick/visibility_control.h"
#include "sensor_msgs/msg/joy.hpp"
#include "rr_common_base/rr_node_joy_plugin_iface.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <pluginlib/class_loader.hpp>

namespace rrobot
{
    namespace rr_joystick
    {

        using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        /**
         * @class RrJoystickNode
         * @brief using transport driver plugin validate inbound messages, and write valid messages to [namespace]/joy topic
         */
        class RRJoystickNode : public rclcpp_lifecycle::LifecycleNode
        {
          public:
            explicit RRJoystickNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
                : rclcpp_lifecycle::LifecycleNode("rr_joystick_node", options)
            {}

            virtual ~RRJoystickNode() = default;

            /**
             * @fn on_configure
             * @brief performs configuration.
             * 
             * Sets up callback and transport plugin.
             */
            CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;

            /**
             * @fn on_activate
             * @brief activates transport, and publisher
             */
            CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;

            /**
             * @fn on_deactivate
             * @brief resets callback, and calls on_deactivate on transport plugin
             */
            CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;

            /**
             * @fn on_cleanup
             * @brief  resets callback, and calls on_cleanup on transport plugin
             */
            CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;

            /**
             * @fn publish_callback
             * @brief called by transport layer when new Joy event arrives
             * 
             * When new joy event arrives, perform data sanitisation check. If there is any
             * issues with the data then call trigger_error_transition(), the lifecycle manager
             * will decide how this transition should be handled.
             */
            void publish_callback(const sensor_msgs::msg::Joy& joy);

          private:
            rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Joy>::SharedPtr publisher_ = nullptr;
            std::shared_ptr<rrobots::interfaces::RrNodeJoyPluginIface> transport_ = nullptr;

        };

    } // namespace rr_joystick
} // namespace rrobot
#endif // RR_JOYSTICK__RR_JOYSTICK_HPP_
