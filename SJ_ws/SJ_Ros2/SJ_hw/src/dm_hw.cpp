/***
 * @Author: _yerik
 * @Date: 2025-07-22 14:21:01
 * @LastEditTime: 2025-07-23 01:36:15
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

    // 辅助1.1 字符串转电机类型
    DM_Motor_Type stringToMotorType(const std::string &type_str)
    {
        if (type_str == "DM4310")
            return DM_Motor_Type::DM4310;
        if (type_str == "DM4340")
            return DM_Motor_Type::DM4340;
        if (type_str == "DM8006")
            return DM_Motor_Type::DM8006;
        else
        {
            return DM_Motor_Type::DM4310;
        }
    }

    // 生命周期1. 初始化硬件接口
    hardware_interface::CallbackReturn DmHW::on_init(const hardware_interface::HardwareInfo &info)
    {
        // 生命周期1.1 调用父类初始化
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Initializing Damiao Hardware Interface...");

        // 生命周期1.2 初始化关节数据存储
        hw_actuator_data_.resize(info_.joints.size());

        // 生命周期1.3 按串口分组关节（从URDF参数）
        std::unordered_map<std::string, int> port_to_baud_rate;

        for (uint i = 0; i < info_.joints.size(); ++i)
        {
            const auto &joint = info_.joints[i];

            // 生命周期1.3.1 提取每个关节的参数
            std::string serial_port = joint.parameters.at("serial_port");
            int baud_rate = std::stoi(joint.parameters.at("baud_rate"));
            int can_id = std::stoi(joint.parameters.at("can_id"));
            int mst_id = std::stoi(joint.parameters.at("mst_id"));
            std::string motor_type_str = joint.parameters.at("motor_type");

            // 生命周期1.3.2 填充关节数据结构
            DmActData data;
            data.name = joint.name;
            data.can_id = can_id;
            data.mst_id = mst_id;
            data.motorType = stringToMotorType(motor_type_str);

            // 生命周期1.3.3 添加到串口-关节映射
            port_to_motors_config_[serial_port][can_id] = data;
            if (port_to_baud_rate.find(serial_port) == port_to_baud_rate.end())
            {
                port_to_baud_rate[serial_port] = baud_rate;
            }

            // 生命周期1.3.4 填充主关节数据向量
            hw_actuator_data_[i] = data;
        }

        // 生命周期1.4 为每个串口创建电机控制对象
        for (auto &pair : port_to_motors_config_)
        {
            const std::string &port_name = pair.first;
            auto *motor_config_map_ptr = &pair.second;
            int baud_rate_for_port = port_to_baud_rate.at(port_name);

            RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Creating Motor_Control for port '%s' at baud rate %d", port_name.c_str(), baud_rate_for_port);

            try
            {
                // Motor_Control 构造函数需要指向关节配置的指针
                motor_controls_[port_name] = std::make_unique<damiao::Motor_Control>(port_name, baud_rate_for_port, motor_config_map_ptr);
            }
            catch (const std::exception &e)
            {
                RCLCPP_FATAL(rclcpp::get_logger("DmHW"), "Failed to create Motor_Control for port '%s': %s", port_name.c_str(), e.what());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // 生命周期1.5 构建关节编号到驱动对象的映射
        for (uint i = 0; i < info_.joints.size(); ++i)
        {
            const std::string &port_for_this_joint = info_.joints[i].parameters.at("serial_port");
            joint_to_driver_map_[i] = motor_controls_.at(port_for_this_joint).get();
        }

        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware Interface initialized successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 生命周期2. 导出关节状态接口（供控制器读取关节状态）
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

    // 生命周期3. 导出关节命令接口（供控制器下发命令）
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

    // 生命周期4. 激活硬件接口
    hardware_interface::CallbackReturn DmHW::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Activating hardware...");
        for (auto const &[port_name, driver] : motor_controls_)
        {
            driver->enable();
        }
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware activated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 生命周期5. 失活硬件接口
    hardware_interface::CallbackReturn DmHW::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Deactivating hardware...");
        for (auto const &[port_name, driver] : motor_controls_)
        {
            driver->disable();
        }
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware deactivated successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 生命周期6. 读取硬件数据（如关节位置、速度、力矩等）
    hardware_interface::return_type DmHW::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // 生命周期6.1 读取所有串口数据
        for (auto const &[port_name, driver] : motor_controls_)
        {
            driver->read();
        }

        // 生命周期6.2 同步数据到主关节数据向量
        for (uint i = 0; i < hw_actuator_data_.size(); ++i)
        {
            const auto &joint_info = info_.joints[i];
            const std::string &port = joint_info.parameters.at("serial_port");
            int can_id = std::stoi(joint_info.parameters.at("can_id"));

            // 生命周期6.2.1 从 map 中找到最新的状态
            const DmActData &updated_data_from_map = port_to_motors_config_.at(port).at(can_id);

            // 生命周期6.2.2 更新 vector 中的状态值，供控制器读取
            hw_actuator_data_[i].pos = updated_data_from_map.pos;
            hw_actuator_data_[i].vel = updated_data_from_map.vel;
            hw_actuator_data_[i].effort = updated_data_from_map.effort;
        }

        return hardware_interface::return_type::OK;
    }

    // 生命周期7. 写入硬件命令（如目标位置、速度、力矩等）
    hardware_interface::return_type DmHW::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // 生命周期7.1 同步命令数据到 map
        for (uint i = 0; i < hw_actuator_data_.size(); ++i)
        {
            const auto &joint_info = info_.joints[i];
            const std::string &port = joint_info.parameters.at("serial_port");
            int can_id = std::stoi(joint_info.parameters.at("can_id"));

            // 生命周期7.1.1 从 vector 中获取最新的命令
            const DmActData &command_data_from_vector = hw_actuator_data_[i];

            // 生命周期7.1.2 更新 map 中的命令值，供驱动写入
            auto &data_in_map = port_to_motors_config_.at(port).at(can_id);
            data_in_map.cmd_pos = command_data_from_vector.cmd_pos;
            data_in_map.cmd_vel = command_data_from_vector.cmd_vel;
            data_in_map.kp = command_data_from_vector.kp;
            data_in_map.kd = command_data_from_vector.kd;
            data_in_map.cmd_effort = command_data_from_vector.cmd_effort;
        }

        // 生命周期7.2 发送命令到所有串口
        for (auto const &[port_name, driver] : motor_controls_)
        {
            driver->write();
        }

        return hardware_interface::return_type::OK;
    }

    // 生命周期8. 配置硬件接口
    hardware_interface::CallbackReturn DmHW::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Configuring hardware...");

        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware configured successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 生命周期9. 清理硬件接口
    hardware_interface::CallbackReturn DmHW::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Cleaning up hardware...");

        motor_controls_.clear();
        RCLCPP_INFO(rclcpp::get_logger("DmHW"), "Hardware cleaned up successfully.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // 生命周期10. 错误处理
    hardware_interface::CallbackReturn DmHW::on_error(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_FATAL(rclcpp::get_logger("DmHW"), "Hardware has encountered an error. Deactivating...");

        on_deactivate(rclcpp_lifecycle::State());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

} // namespace damiao

// ROS2插件导出宏（官方代码）
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(damiao::DmHW, hardware_interface::SystemInterface)