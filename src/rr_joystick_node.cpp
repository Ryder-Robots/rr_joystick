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

#include "rr_joystick/rr_joystick_node.hpp"

using namespace std::placeholders;

namespace rrobot::rr_joystick
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    /**
     * Default to UDP plugin, for joysticks this will probally always be true.
     */
    CallbackReturn RRJoystickNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "configuring %s", this->get_name());

        // configure plugin first. default to UDP plugin
        declare_parameter("transport_plugin", "rr_joy_udp_plugin");
        std::string plugin_param = this->get_parameter("transport_plugin").as_string();
        RCLCPP_DEBUG(this->get_logger(), "transport plugin is '%s'", plugin_param.c_str());
        pluginlib::ClassLoader<rrobots::interfaces::RrNodeJoyPluginIface> poly_loader("rr_common_base", "rrobots::interfaces::RrNodeJoyPluginIface");
        try {
            transport_ = poly_loader.createSharedInstance(plugin_param);
        }
        catch (pluginlib::PluginlibException &ex) {
            RCLCPP_FATAL(this->get_logger(), "could not load transport plugin: %s", this->get_parameter("transport_plugin").as_string().c_str());
            return CallbackReturn::ERROR;
        }

        // create callback
        auto cb = std::bind(&RRJoystickNode::publish_callback, this, _1);

        // create publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/joy", rclcpp::QoS(10).best_effort());

        return transport_->configure(state, cb, this->shared_from_this());
    }

    CallbackReturn RRJoystickNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        publisher_->on_activate();
        return transport_->on_activate(state);
    }


    CallbackReturn RRJoystickNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        if (publisher_ != nullptr) {
            publisher_->on_deactivate();
        }
        return transport_->on_deactivate(state);
    }

    CallbackReturn RRJoystickNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        if (publisher_ != nullptr) {
            publisher_.reset();
        }
        return transport_->on_cleanup(state);
    }

    bool RRJoystickNode::validate(const sensor_msgs::msg::Joy &joy)
    {
        if (joy.axes.size() > AXES_SZ) {
            RCLCPP_ERROR(this->get_logger(),
                "axes size is greater than max AXES_SZ, ignoring values excess values");
            return false;
        }
        if (joy.buttons.size() > BUTTONS_SZ) {
            RCLCPP_ERROR(this->get_logger(),
                "buttons size is greater than max AXES_SZ, ignoring values excess "
                "values");

            return false;
        }
        // validate max and min values
        for (float i : joy.axes) {
            if (i > MAX_AXES || i < MIN_AXES) {
                RCLCPP_ERROR(this->get_logger(), "axes is not within threshold range");
                return false;
            }
        }

        // validate max and min values
        for (int i : joy.buttons) {
            if (i > MAX_BUTTON || i < MIN_BUTTON) {
                RCLCPP_ERROR(this->get_logger(), "button is not within threshold range");
                return false;
            }
        }
        return true;
    }

    // when invalid is recieved callback will call trigger_error_transition(), this will
    // a state change, and it will be update to lifecycle node to determine what to do with
    // the event.
    void RRJoystickNode::publish_callback(const sensor_msgs::msg::Joy &joy)
    {
        // perform data sanitation here.

        if (!validate(joy)) {
            RCLCPP_ERROR(this->get_logger(), "joy event failed sanitisation");
            this->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ON_ACTIVATE_ERROR);
            return;
        }

        publisher_->publish(joy);
    }
} // namespace rrobot::rr_joystick

RCLCPP_COMPONENTS_REGISTER_NODE(rrobot::rr_joystick::RRJoystickNode);