#include "mujoco_ros2_control/mujoco_system.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control
{

bool MujocoSystem::init_sim(/*mjModel * model, mjData * data, */ const hardware_interface::HardwareInfo &hardware_info)
{
  RCLCPP_INFO(get_logger(), "Initializing MujocoSystem...");
  // mj_model_ = model;
  // mj_data_ = data;
  // mj_joint_info_ = std::make_shared<std::unordered_map<std::string, MujocoJointInfo>>();

  // for(size_t joint_index = 0; joint_index < hardware_info.joints.size(); ++joint_index) 
  // {
  //   auto joint = hardware_info.joints[joint_index];
  //   int mj_joint_id = mj_name2id(mj_model_, mjtObj::mjOBJ_JOINT, joint.name.c_str());
  //   if (mj_joint_id == -1) {
  //     RCLCPP_ERROR(get_logger(), "Joint '%s' not found in Mujoco model.", joint.name.c_str());
  //     return false;
  //   }
  //   MujocoJointInfo joint_info;
  //   joint_info.mj_joint_type = mj_model_->jnt_type[mj_joint_id];
  //   joint_info.mj_pos_adr = mj_model_->jnt_qposadr[mj_joint_id];
  //   joint_info.mj_vel_adr = mj_model_->jnt_dofadr[mj_joint_id];
  //   mj_joint_info_->insert({joint.name, joint_info});
  //   RCLCPP_INFO(get_logger(), "Joint '%s' found in Mujoco model with type %d", joint.name.c_str(), joint_info.mj_joint_type);
  // }
  return true;
}

hardware_interface::CallbackReturn MujocoSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  control_level_.resize(info_.joints.size(), integration_level_t::POSITION);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemMultiInterface has exactly 3 state interfaces
    // and 3 command interfaces on each joint
    if (joint.command_interfaces.size() <= 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces. At least 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION ||
          joint.command_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %s command interface. Expected %s, %s, %s or %s.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() <= 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s'has %zu state interfaces. At least 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_ACCELERATION ||
          joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %s state interface. Expected %s, %s, %s or %s.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
        hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // for(const auto & joint : info_.joints)
  // {
  //   if(mj_joint_info_->find(joint.name) == mj_joint_info_->end())
  //   {
  //     RCLCPP_ERROR(get_logger(), "Joint '%s' not found in Mujoco model.", joint.name.c_str());
  //     return hardware_interface::CallbackReturn::ERROR;
  //   }
  // }

  auto get_initial_value = [this](const hardware_interface::InterfaceInfo & info)
  {
    if(!info.initial_value.empty())
    {
      return std::stod(info.initial_value);
    }
    else
    {
      return 0.0; // std::numeric_limits<double>::quiet_NaN();
    }
  };

  // reset values always when configuring hardware
  for (const auto & [name, descr] : joint_state_interfaces_)
  { 
    set_state(name, 0.0); // get_initial_value(descr.interface_info));
  }

  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  RCLCPP_INFO(get_logger(), "Preparing command mode switch...");
  // Prepare for new command modes
  std::vector<integration_level_t> new_modes = {};
  for (std::string key : start_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
      {
        new_modes.push_back(integration_level_t::POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
      {
        new_modes.push_back(integration_level_t::VELOCITY);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION)
      {
        new_modes.push_back(integration_level_t::ACCELERATION);
      }
    }
  }
  // Example criteria: All joints must be given new command mode at the same time
  if (new_modes.size() != info_.joints.size())
  {
    return hardware_interface::return_type::ERROR;
  }
  // Example criteria: All joints must have the same command mode
  if (!std::all_of(
        new_modes.begin() + 1, new_modes.end(),
        [&](integration_level_t mode) { return mode == new_modes[0]; }))
  {
    return hardware_interface::return_type::ERROR;
  }

  // Stop motion on all relevant joints that are stopping
  for (std::string key : stop_interfaces)
  {
    for (std::size_t i = 0; i < info_.joints.size(); i++)
    {
      if (key.find(info_.joints[i].name) != std::string::npos)
      {
        set_command(
          info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION,
          get_state(info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION));
        set_command(info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY, 0.0);
        set_command(info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION, 0.0);
        control_level_[i] = integration_level_t::UNDEFINED;  // Revert to undefined
      }
    }
  }
  // Set the new command modes
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    if (control_level_[i] != integration_level_t::UNDEFINED)
    {
      // Something else is using the joint! Abort!
      return hardware_interface::return_type::ERROR;
    }
    control_level_[i] = new_modes[i];
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MujocoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating... please wait...");

  for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_start_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // Set some default values
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    control_level_[i] = integration_level_t::UNDEFINED;
  }

  RCLCPP_INFO(get_logger(), "System successfully activated! %u", control_level_[0]);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Deactivating... please wait...");

  for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(get_logger(), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  // RCLCPP_INFO(get_logger(), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto name_acc = info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    switch (control_level_[i])
    {
      case integration_level_t::UNDEFINED:
        // RCLCPP_INFO(get_logger(), "Nothing is using the hardware interface!");
        return hardware_interface::return_type::OK;
        break;
      case integration_level_t::POSITION:
        set_state(name_acc, 0.);
        set_state(name_vel, 0.);
        set_state(
          name_pos,
          get_state(name_pos) + (get_command(name_pos) - get_state(name_pos)) / hw_slowdown_);
        break;
      case integration_level_t::VELOCITY:
        set_state(name_acc, 0.);
        set_state(name_vel, get_command(name_vel));
        set_state(
          name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds() / hw_slowdown_);
        break;
      case integration_level_t::ACCELERATION:
        set_state(name_acc, get_command(name_acc));
        set_state(
          name_vel, get_state(name_vel) + get_state(name_acc) * period.seconds() / hw_slowdown_);
        set_state(
          name_pos, get_state(name_pos) + get_state(name_vel) * period.seconds() / hw_slowdown_);
        break;
    }
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "pos: " << get_state(name_pos) << ", vel: " << get_state(name_vel)
       << ", acc: " << get_state(name_acc) << " for joint " << i;
  }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";
  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    // Simulate sending commands to the hardware
    const auto name_acc = info_.joints[i].name + "/" + hardware_interface::HW_IF_ACCELERATION;
    const auto name_vel = info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY;
    const auto name_pos = info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t"
       << "command pos: " << get_command(name_pos) << ", vel: " << get_command(name_vel)
       << ", acc: " << get_command(name_acc) << " for joint " << i
       << ", control lvl: " << static_cast<int>(control_level_[i]);
  }
  // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace mujoco_ros2_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control::MujocoSystem, mujoco_ros2_control::MujocoSystemInterface)