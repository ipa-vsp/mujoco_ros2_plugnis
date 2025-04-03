#ifndef MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_
#define MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

namespace mujoco_ros2_control
{
class MujocoSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MujocoSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  bool init_sim(mjModel *model, mjData *data, const hardware_interface::HardwareInfo &hardware_info);
private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum integration_level_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    ACCELERATION = 3
  };

  // Active control mode for each actuator
  std::vector<integration_level_t> control_level_;

  mjModel *mj_model_;
  mjData *mj_data_;
};
} // namespace mujoco_ros2_control

#endif  // MUJOCO_ROS2_CONTROL__MUJOCO_SYSTEM_HPP_