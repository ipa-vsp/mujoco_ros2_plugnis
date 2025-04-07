#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "hardware_interface/system_interface.hpp"

#include "mujoco_ros2_control/mujoco_ros2_control.hpp"

namespace mujoco_ros2_control
{

class MJResourceManager : public hardware_interface::ResourceManager
{
public:
    MJResourceManager(rclcpp::Node::SharedPtr & node /*, mjModel *mujoco_model, mjData *mujoco_data*/)
    : hardware_interface::ResourceManager(
            node->get_node_clock_interface(), node->get_node_logging_interface()),
        mj_system_loader_("mujoco_ros2_control", "mujoco_ros2_control::MujocoSystemInterface"),
        logger_(node->get_logger().get_child("MJResourceManager"))
        // mj_model_(mujoco_model),
        // mj_data_(mujoco_data)
    {
        node_ = node;
    }

    MJResourceManager(const MJResourceManager &) = delete;

    // Called from Controller Manager when robot description is initialized from callback
    bool load_and_initialize_components(
    const std::string & urdf,
    unsigned int update_rate) override
    {
    components_are_loaded_and_initialized_ = true;

    const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

    for (const auto & individual_hardware_info : hardware_info) {
        std::string robot_hw_sim_type_str_ = individual_hardware_info.hardware_plugin_name;
        RCLCPP_INFO(
        logger_, "Load hardware interface %s ...",
        robot_hw_sim_type_str_.c_str());

        // Load hardware
        std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface> mjSimSystem;
        std::scoped_lock guard(resource_interfaces_lock_, claimed_command_interfaces_lock_);
        try {
            mjSimSystem = std::unique_ptr<mujoco_ros2_control::MujocoSystemInterface>(
                mj_system_loader_.createUnmanagedInstance(robot_hw_sim_type_str_));
        } catch (pluginlib::PluginlibException & ex) {
            RCLCPP_ERROR(
                logger_,
                "The plugin failed to load for some reason. Error: %s\n",
                ex.what());
            continue;
        }

        // initialize simulation requirements
        if (!mjSimSystem->init_sim(/*mj_model_, mj_data_,*/ individual_hardware_info))
        {
            RCLCPP_FATAL(
                logger_, "Could not initialize robot simulation interface");
            components_are_loaded_and_initialized_ = false;
            break;
        }
        RCLCPP_DEBUG(
            logger_, "Initialized robot simulation interface %s!",
            robot_hw_sim_type_str_.c_str());

        // initialize hardware
        import_component(std::move(mjSimSystem), individual_hardware_info);
    }

    return components_are_loaded_and_initialized_;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;

    // mjModel *mj_model_;
    // mjData *mj_data_;

    /// \brief Interface loader
    pluginlib::ClassLoader<mujoco_ros2_control::MujocoSystemInterface> mj_system_loader_;

    rclcpp::Logger logger_;
};    
MujocoRos2Control::MujocoRos2Control(
  rclcpp::Node::SharedPtr &node, const rclcpp::NodeOptions & options /*, mjModel *mujoco_model, mjData *mujoco_data*/)
    : node_(node),
        options_(options),
    //   mj_model_(mujoco_model),
    //   mj_data_(mujoco_data),
      logger_(rclcpp::get_logger(node_->get_name() + std::string(".mujoco_ros2_control"))),
      control_period_(rclcpp::Duration(1, 0)),
      last_update_sim_time_ros_(0, 0, RCL_ROS_TIME)
{
}

MujocoRos2Control::~MujocoRos2Control()
{
  stop_cm_thread_ = true;
  cm_executor_->remove_node(controller_manager_);
  cm_executor_->cancel();
  cm_thread_.join();
}

void MujocoRos2Control::init()
{
  clock_publisher_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  // Read urdf from ros parameter server then
  // setup actuators and mechanism control node.
  std::string urdf_string;
  std::vector<hardware_interface::HardwareInfo> control_hardware_info;
  try
  {
    urdf_string = node_->get_parameter("robot_description").as_string();
    control_hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf_string);
  }
  catch (const std::runtime_error &ex)
  {
    RCLCPP_ERROR_STREAM(logger_, "Error parsing URDF : " << ex.what());
    return;
  }

  std::unique_ptr<hardware_interface::ResourceManager> resource_manager =
    std::make_unique<MJResourceManager>(node_ /*, mj_model_, mj_data_*/);

  // Create the controller manager
  RCLCPP_INFO(logger_, "Loading controller_manager");
  cm_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  controller_manager_ = std::make_shared<controller_manager::ControllerManager>(
    std::move(resource_manager), cm_executor_, "controller_manager", node_->get_namespace(), options_);

  cm_executor_->add_node(controller_manager_);

  if (!controller_manager_->has_parameter("update_rate"))
  {
    RCLCPP_ERROR_STREAM(logger_, "controller manager doesn't have an update_rate parameter");
    return;
  }

  auto update_rate = controller_manager_->get_parameter("update_rate").as_int();
  RCLCPP_INFO(logger_, "controller manager update rate: %d Hz", update_rate);
  control_period_ = rclcpp::Duration(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / static_cast<double>(update_rate))));

  // Force setting of use_sime_time parameter
  controller_manager_->set_parameter(
    rclcpp::Parameter("use_sim_time", rclcpp::ParameterValue(true)));

  stop_cm_thread_ = false;
  auto spin = [this]()
  {
    while (rclcpp::ok() && !stop_cm_thread_)
    {
      cm_executor_->spin_once();
    }
  };
  cm_thread_ = std::thread(spin);
}

void MujocoRos2Control::update()
{
  // Get the simulation time and period
  // auto sim_time = mj_data_->time;
  int sim_time_sec = static_cast<int>(10);
  int sim_time_nanosec = static_cast<int>((10 - sim_time_sec) * 1000000000);

  rclcpp::Time sim_time_ros(sim_time_sec, sim_time_nanosec, RCL_ROS_TIME);
  rclcpp::Duration sim_period = sim_time_ros - last_update_sim_time_ros_;

  publish_sim_time(sim_time_ros);

  // mj_step1(mj_model_, mj_data_);

  if (sim_period >= control_period_)
  {
    controller_manager_->read(sim_time_ros, sim_period);
    controller_manager_->update(sim_time_ros, sim_period);
    last_update_sim_time_ros_ = sim_time_ros;
  }

  // use same time as for read and update call - this is how it is done in ros2_control_node
  controller_manager_->write(sim_time_ros, sim_period);

  // mj_step2(mj_model_, mj_data_);
}

void MujocoRos2Control::publish_sim_time(rclcpp::Time sim_time)
{
  // TODO(sangteak601)
  rosgraph_msgs::msg::Clock sim_time_msg;
  sim_time_msg.clock = sim_time;
  clock_publisher_->publish(sim_time_msg);
}

}  // namespace mujoco_ros2_control