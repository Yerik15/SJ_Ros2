/*** 
 * @Author: _yerik
 * @Date: 2025-07-22 14:21:01
 * @LastEditTime: 2025-07-23 00:01:23
 * @LastEditors: _yerik
 * @Description: 
 * @FilePath: /Simple_Joint/SJ_ws/SJ_Ros2/SJ_hw/src/dm_hw.cpp
 * @Code. Run. No errors.
 */

#include "SJ_hw/dm_hw.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <string>

namespace damiao
{

DM_Motor_Type stringToMotorType(const std::string& type_str) {
    if (type_str == "DM4310") return DM_Motor_Type::DM4310;
    if (type_str == "DM4340") return DM_Motor_Type::DM4340;
    if (type_str == "DM8006") return DM_Motor_Type::DM8006;
    else {
        return DM_Motor_Type::DM4310; 
    }
}

hardware_interface::CallbackReturn DmHW::on_init(const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Initializing Damiao Hardware Interface...");

    // Initialize runtime data storage for joints
    hw_actuator_data_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // --- STAGE 1: Group joints by serial port from URDF info ---
    std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_to_motors_config;
    std::unordered_map<std::string, int> port_to_baud_rate;

    for (uint i = 0; i < info_.joints.size(); ++i)
    {
        const auto& joint = info_.joints[i];
        
        // --- Extract parameters for each joint ---
        std::string serial_port = joint.parameters.at("serial_port");
        int baud_rate = std::stoi(joint.parameters.at("baud_rate"));
        int can_id = std::stoi(joint.parameters.at("can_id"));
        int mst_id = std::stoi(joint.parameters.at("mst_id"));
        std::string motor_type_str = joint.parameters.at("motor_type");

        // --- Populate the data structures ---
        DmActData data;
        data.name = joint.name;
        data.can_id = can_id;
        data.mst_id = mst_id;
        data.motorType = stringToMotorType(motor_type_str);
        
        // Add to the temporary grouping map
        port_to_motors_config[serial_port][can_id] = data;
        if (port_to_baud_rate.find(serial_port) == port_to_baud_rate.end()) {
            port_to_baud_rate[serial_port] = baud_rate;
        }

        // Also populate our main runtime data vector at the correct index
        hw_actuator_data_[i] = data;
    }

    // --- STAGE 2: Create Motor_Control instances for each unique serial port ---
    for (const auto& pair : port_to_motors_config)
    {
        const std::string& port_name = pair.first;
        auto motor_config_map_for_port = pair.second; // Make a copy
        int baud_rate_for_port = port_to_baud_rate.at(port_name);

        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Creating Motor_Control for port '%s' at baud rate %d", port_name.c_str(), baud_rate_for_port);

        try {
            // Note: Motor_Control constructor needs a pointer to the map.
            motor_controls_[port_name] = std::make_unique<damiao::Motor_Control>(port_name, baud_rate_for_port, &motor_config_map_for_port);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(rclcpp::get_logger("DmHW"), "Failed to create Motor_Control for port '%s': %s", port_name.c_str(), e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    // --- STAGE 3: Build the joint_to_driver_map_ for fast runtime access ---
    for (uint i = 0; i < info_.joints.size(); ++i) {
        const std::string& port_for_this_joint = info_.joints[i].parameters.at("serial_port");
        joint_to_driver_map_[i] = motor_controls_.at(port_for_this_joint).get();
    }

    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware Interface initialized successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DmHW::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            hw_actuator_data_[i].name, hardware_interface::HW_IF_POSITION, &hw_actuator_data_[i].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            hw_actuator_data_[i].name, hardware_interface::HW_IF_VELOCITY, &hw_actuator_data_[i].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            hw_actuator_data_[i].name, hardware_interface::HW_IF_EFFORT, &hw_actuator_data_[i].effort));
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DmHW::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (uint i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hw_actuator_data_[i].name, "position_des", &hw_actuator_data_[i].cmd_pos));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hw_actuator_data_[i].name, "velocity_des", &hw_actuator_data_[i].cmd_vel));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hw_actuator_data_[i].name, "kp", &hw_actuator_data_[i].kp));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hw_actuator_data_[i].name, "kd", &hw_actuator_data_[i].kd));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            hw_actuator_data_[i].name, "feedforward", &hw_actuator_data_[i].cmd_effort));
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn DmHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Activating hardware...");
    for (auto const& [port_name, driver] : motor_controls_)
    {
        driver->enable();
    }
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware activated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Deactivating hardware...");
    for (auto const& [port_name, driver] : motor_controls_)
    {
        driver->disable();
    }
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware deactivated successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DmHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (auto const& [port_name, driver] : motor_controls_)
    {
        driver->read(); 
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DmHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    for (auto const& [port_name, driver] : motor_controls_)
    {
        driver->write();
    }
    return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn DmHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Configuring hardware...");
    
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware configured successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Cleaning up hardware...");
    
    motor_controls_.clear();
    RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware cleaned up successfully.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DmHW::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_FATAL(rclcpp::get_logger("DmHW"), "Hardware has encountered an error. Deactivating...");
    
    on_deactivate(rclcpp_lifecycle::State());
    return hardware_interface::CallbackReturn::SUCCESS;
}

} // namespace damiao


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(damiao::DmHW, hardware_interface::SystemInterface)