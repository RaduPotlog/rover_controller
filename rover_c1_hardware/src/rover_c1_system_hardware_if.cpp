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


#include "rover_c1_hardware/rover_c1_system_hardware_if.hpp"

#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/executors.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace rover_c1_hardware
{


using std::placeholders::_1;
using namespace std::chrono_literals;


/////////////////////////////// Publisher ///////////////////////////////////////////////////////
HardwareCommandPub::HardwareCommandPub() : Node("hardware_command_publisher")
{
    publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("/ap/joy", 5);
}

void HardwareCommandPub::publish_data(const double left_wheel, const double right_wheel)
{
    auto message = sensor_msgs::msg::Joy();

    // RC1
    message.axes.push_back(0);
    // RC2 - right wheel: -1 reverse, 1 forward
    message.axes.push_back((float)right_wheel);      
    message.axes.push_back((float)left_wheel);
    // RC4
    message.axes.push_back(0);

    // Publish data
    publisher_->publish(message);
}


/////////////////////////////// Hardware Interface ///////////////////////////////////////////////////////
hardware_interface::CallbackReturn RoverC1SystemHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("controller_manager.resource_manager.hardware_component.system.RoverC1"));
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

    hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        // DiffBotSystem has exactly two states and one command interface on each joint
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                hardware_interface::HW_IF_VELOCITY);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    
    rclcpp::NodeOptions options;
    options.arguments({ "--ros-args", "-r", "__node:=rover_c1_ros2_control_" });
    
    // Create node
    node_ = rclcpp::Node::make_shared("_", options);

    // Fireup publisher
    hw_cmd_pub_ = std::make_shared<HardwareCommandPub>();

    auto callback = [this](const ardupilot_msgs::msg::Encoder &msg) -> void {
        
        if (msg.instance < 2) {
            distance_count_[msg.instance] = msg.distance_count;
            dist_count_change_[msg.instance] = msg.dist_count_change;
            dt_ms_[msg.instance] = msg.dt_ms;
        }
	};
    
	auto callback_simuli = [this](const std_msgs::msg::Float32 &msg) -> void {
        
        simulink_velocity_ = msg.data;
		
    };
        
    // Fireup subscriber
    encoder_subscription_ = node_->create_subscription<ardupilot_msgs::msg::Encoder>("/ap/encoder", rclcpp::SensorDataQoS(), callback);
    encoder_subscription_simuli_ = node_->create_subscription<std_msgs::msg::Float32>("/simulink/speed_rad", rclcpp::SensorDataQoS(), callback_simuli);
   
	
	(void)encoder_subscription_;
	
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoverC1SystemHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    
    for (auto i = 0u; i < info_.joints.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RoverC1SystemHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto i = 0u; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }

    return command_interfaces;
}

hardware_interface::CallbackReturn RoverC1SystemHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Activating ...please wait...");

    for (auto i = 0u; i < hw_positions_.size(); i++) {
        if (std::isnan(hw_positions_[i])) {
            hw_positions_[i] = 0;
            hw_velocities_[i] = 0;
            hw_commands_[i] = 0;
        }
    }

    RCLCPP_INFO(get_logger(), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverC1SystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");
    RCLCPP_INFO(get_logger(), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
}

double RoverC1SystemHardwareInterface::get_delta_angle(uint8_t instance) const
{
    if (instance >= 2) {
        return 0.0f;
    }
    
    return (2.0 * 3.14) * (double)distance_count_[instance] / 60.0;
}

double RoverC1SystemHardwareInterface::get_rate_in_rad(uint8_t instance) const
{
    if (instance >= 2) {
        return 0.0;
    }

    if (dt_ms_ == 0) {
        return 0.0;
    }
	
	double rate_in_rad = (2.0 * 3.14) * ((double)dist_count_change_[instance] / (double)110.0) / ((double)dt_ms_[instance] * (double)1e-3f);
	
	if (std::isnan(rate_in_rad)) {
		return 0.0;
	}
	
    // Calculate delta_angle (in radians) per second
    return rate_in_rad;
}
  
hardware_interface::return_type RoverC1SystemHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
    (void)period;

    if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }
    
    uint8_t index = 0U;
    
    for (std::size_t i = hw_positions_.size(); i > 0; i--) {        
        hw_positions_[index] = get_delta_angle((uint8_t)(i - 1UL));
        index++;
    }
    
    index = 0U;
    
    for (std::size_t i = hw_velocities_.size(); i > 0; i--) {        
        hw_velocities_[index] = /*simulink_velocity_;*/ get_rate_in_rad((uint8_t)(i - 1UL));
        index++;
    }
    
	
	
    std::stringstream ss;
    ss << "Update velocity and position commands:";

    for (auto i = 0U; i < hw_positions_.size(); i++) {
        ss << std::fixed << std::setprecision(2) << std::endl
           << "\t" << "position " << hw_positions_[i] << " for '" << info_.joints[i].name.c_str() << "'!";
    }
    
    for (auto i = 0U; i < hw_velocities_.size(); i++) {
        ss << std::fixed << std::setprecision(2) << std::endl
           << "\t" << "velocity " << hw_velocities_[i] << " for '" << info_.joints[i].name.c_str() << "'!";
    }
    
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
    
    return hardware_interface::return_type::OK;
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

double convert_rad_s_to_m_s(double angular_speed, double radius)
{
	return angular_speed * radius;
}
  
hardware_interface::return_type RoverC1SystemHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
	const double max_velocity = 14.28;
    
	double left_wheel = 0.0;
	double right_wheel = 0.0;
	
	double left_wheel_cmd = 0.0;
	double right_wheel_cmd = 0.0;
	
	bool is_nan = false;
	
	for (auto i = 0U; i < hw_commands_.size(); i++) {
    	if (std::isnan((float)hw_commands_[i])) {
			is_nan = true;
			break;
		}
	}
	
	if (is_nan == true) {
		left_wheel_cmd = 0.0;
		right_wheel_cmd = 0.0;
	}
	else {
		
		if ((double)hw_commands_[0] > max_velocity) {
			left_wheel_cmd = max_velocity;
		}
		else if ((double)hw_commands_[0] < (max_velocity * (-1))) {
			left_wheel_cmd = (max_velocity * (-1));		
		}
		else {
			left_wheel_cmd = (double)hw_commands_[0];
		}
		
		if ((double)hw_commands_[1] > max_velocity) {
			right_wheel_cmd = max_velocity;
		}
		else if ((double)hw_commands_[1] < (max_velocity * (-1))) {
			right_wheel_cmd = (max_velocity * (-1));		
		}
		else {
			right_wheel_cmd = (double)hw_commands_[1];
		}
	}
	
	if (max_velocity > 0.0) {
		left_wheel = map(left_wheel_cmd, max_velocity * (-1), max_velocity, -1, 1);
		right_wheel = map(right_wheel_cmd, max_velocity * (-1), max_velocity, -1, 1);
    }
	
    // left wheel, right wheel
    hw_cmd_pub_->publish_data(left_wheel, right_wheel);
	

	
    std::stringstream ss;
    ss << "Update joints commands:";
    
	ss << std::fixed << std::setprecision(2) << std::endl << "\t" << "Max velocity " << max_velocity << "'!";
	
    for (auto i = 0U; i < hw_commands_.size(); i++) {
        ss << std::fixed << std::setprecision(2) << std::endl
           << "\t" << "command velocity from pid in rad " << hw_commands_[i] << " for '"  << info_.joints[i].name.c_str() << "'!";
    }
    
	ss << std::fixed << std::setprecision(2) << std::endl
	   << "\t" << "command velocity mapded " << left_wheel << " for '"  << info_.joints[0].name.c_str() << "'!";

	ss << std::fixed << std::setprecision(2) << std::endl
	   << "\t" << "command velocity mapded " << right_wheel << " for '"  << info_.joints[1].name.c_str() << "'!";

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());

    return hardware_interface::return_type::OK;
}

}  // namespace rover_c1_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    rover_c1_hardware::RoverC1SystemHardwareInterface, hardware_interface::SystemInterface)
