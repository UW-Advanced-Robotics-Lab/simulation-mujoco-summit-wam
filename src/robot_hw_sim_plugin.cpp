/*
* Copyright 2018 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation version 2 of the License.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @file   robot_hw_sim_plugin.cpp
* @author Giuseppe Barbieri <giuseppe@shadowrobot.com>
* @brief  Node to allow ros_control hardware interfaces to be plugged into mujoco
* 
* Current version specifically for UWARL
* Last edit: Nov 28, 2023 (Tim van Meijel)
*
**/

#include <mujoco_ros_control/robot_hw_sim_plugin.h>
#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

#include <string>
#include <vector>
#include <map>
#include <std_msgs/String.h>
#include <list>


namespace mujoco_ros_control
{

bool RobotHWSimPlugin::init_sim(
      const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    return false;
  }

void RobotHWSimPlugin::pass_mj_data(std::map<std::string, std::vector <double> > *list_joints)
  {
    return;
  }

std::map<std::string, double>* RobotHWSimPlugin::get_mj_data(void)
  {
    std::map<std::string, double > * test;
    return test;
  }

}  // namespace mujoco_ros_control


