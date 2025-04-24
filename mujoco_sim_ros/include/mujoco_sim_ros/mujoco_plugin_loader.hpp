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
            virtual void init(rclcpp::Node::SharedPtr &node, rclcpp::NodeOptions &options, mjModel *model, mjData * data) = 0;
            virtual void reset(mjModel *model, mjData *data) = 0;
            virtual void on_configure(mjModel *model, mjData *data) = 0;
            virtual void pre_step(mjModel *model, mjData *data) = 0;
            virtual void step(mjModel *model, mjData *data) = 0;
            virtual void on_cleanup(mjModel *model, mjData *data) = 0;
            virtual void on_deactivate(mjModel *model, mjData *data) = 0;
        protected:
            MujocoPluginLoader() = default;
    };
}

#endif