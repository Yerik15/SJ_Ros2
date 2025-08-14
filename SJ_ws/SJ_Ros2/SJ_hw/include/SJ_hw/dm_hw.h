/***
 * @Author: _yerik
 * @Date: 2025-07-22 14:21:17
 * @LastEditTime: 2025-07-23 00:38:40
 * @LastEditors: _yerik
 * @Description:
 * @FilePath: /Simple_Joint/SJ_ws/SJ_Ros2/SJ_hw/include/SJ_hw/dm_hw.h
 * @Code. Run. No errors.
 */
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "SJ_hw/damiao.h"

namespace damiao
{

  // 机械臂硬件接口类，继承自ROS2官方SystemInterface
  class DmHW : public hardware_interface::SystemInterface
  {
  public:
    // 生命周期1. 初始化硬件接口
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    // 生命周期2. 导出关节状态接口（供控制器读取关节状态）
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    // 生命周期3. 导出关节命令接口（供控制器下发命令）
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    // 生命周期4. 激活硬件接口
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    // 生命周期5. 失活硬件接口
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    // 生命周期6. 读取硬件数据（如关节位置、速度、力矩等）
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    // 生命周期7. 写入硬件命令（如目标位置、速度、力矩等）
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    // 生命周期8. 配置硬件接口
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    // 生命周期9. 清理硬件接口
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    // 生命周期10. 错误处理
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

  private:
    // 1. 关节数据相关
    // 1.1 从urdf获取关节信息，并存储所有关节的实时数据
    std::vector<damiao::DmActData> hw_actuator_data_;

    // 2. 电机控制相关
    // 2.1 串口与电机控制对象的映射关系（串口号->电机控制器）
    std::unordered_map<std::string, std::unique_ptr<damiao::Motor_Control>> motor_controls_;

    // 2.2 关节与电机驱动对象的映射关系（关节编号->电机控制器指针）
    std::unordered_map<int, damiao::Motor_Control *> joint_to_driver_map_;

    // 2.3 串口与多个电机配置的映射关系（串口号->(关节编号->关节数据)）
    std::unordered_map<std::string, std::unordered_map<int, DmActData>> port_to_motors_config_;
  };

  // 智能指针定义（ROS2官方宏）
  RCLCPP_SHARED_PTR_DEFINITIONS(DmHW)

} // namespace damiao