#ifndef MUJOCO_PLUGIN_LOADER_HPP_
#define MUJOCO_PLUGIN_LOADER_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "mujoco/mujoco.h"

namespace mujoco_sim_ros
{
    class MujocoPluginLoader
    {
        public:
            virtual ~MujocoPluginLoader() = default;
            virtual void on_init(rclcpp::Node::SharedPtr &node, rclcpp::NodeOptions &options, mjModel *model, mjData * data) = 0;
            virtual void on_start(mjModel *model, mjData *data) = 0;
            virtual void on_configure(mjModel *model, mjData *data) = 0;
            virtual void on_activate(mjModel *model, mjData *data) = 0;
            virtual void update(mjModel *model, mjData *data) = 0;
            virtual void on_cleanup(mjModel *model, mjData *data) = 0;
            virtual void on_deactivate(mjModel *model, mjData *data) = 0;
        protected:
            MujocoPluginLoader() = default;
    };
}

#endif