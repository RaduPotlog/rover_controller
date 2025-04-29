// Copyright (c) 2025, @radupotlog
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_C1_SYSTEM_HARDWARE_IF_HPP_
#define ROVER_C1_SYSTEM_HARDWARE_IF_HPP_

#include <memory>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/node.hpp>
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "ardupilot_msgs/msg/encoder.hpp"

#include <control_toolbox/low_pass_filter.hpp>

namespace rover_c1_hardware
{


class HardwareCommandPub : public rclcpp::Node  //the node definition for the publisher to talk to micro-ROS agent
{
public:

    HardwareCommandPub();

    void publish_data(const double left_wheel, const double right_wheel);

private:

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr publisher_;
};

class RoverC1SystemHardwareInterface : public hardware_interface::SystemInterface
{

public:

    RCLCPP_SHARED_PTR_DEFINITIONS(RoverC1SystemHardwareInterface);

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    rclcpp::Logger get_logger() const { return *logger_; }

    rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
    
    double get_delta_angle(uint8_t instance) const;
    double get_rate_in_rad(uint8_t instance) const;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<ardupilot_msgs::msg::Encoder>::SharedPtr encoder_subscription_;
    
    std::shared_ptr<HardwareCommandPub> hw_cmd_pub_;    
    
    // Objects for logging
    std::shared_ptr<rclcpp::Logger> logger_;
    
    rclcpp::Clock::SharedPtr clock_;

    // Store the command for the robot
    std::vector<double> hw_commands_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    
    int distance_count_[2] = {0};
    int dist_count_change_[2] = {0};
    uint32_t dt_ms_[2] = {0};
	
	double simulink_velocity_ = 0.0;
};

}  // namespace rover_c1_hardware

#endif  // ROVER_C1_SYSTEM_HARDWARE_IF_HPP_
