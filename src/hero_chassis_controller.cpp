//
// 麦克纳姆轮底盘控制器 —— ros_control 插件实现。
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <ros/ros.h>

namespace hero_chassis_controller {

bool HeroChassisController::initWheel(Wheel &wheel, const std::string &wheel_name,
                                      hardware_interface::EffortJointInterface *effort_joint_interface,
                                      ros::NodeHandle &controller_nh) {
  // 参数结构（config/controllers.yaml）：
  //   wheels:
  //     left_front:
  //       joint: left_front_wheel_joint
  //       target_velocity: 5.0
  //       pid: { p: 0.5, i: 0.0, d: 0.0, i_clamp: 2.0, antiwindup: true }
  ros::NodeHandle nh(controller_nh, "wheels/" + wheel_name);

  std::string joint_name;
  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("Missing param 'wheels/%s/joint', check config/controllers.yaml", wheel_name.c_str());
    return false;
  }
  wheel.joint = effort_joint_interface->getHandle(joint_name);

  // Pid::init 从 <wheels>/<name>/pid 读 p / i / d / i_clamp / antiwindup，
  if (!wheel.pid.init(ros::NodeHandle(nh, "pid"))) {
    ROS_ERROR("Failed to load PID gains from 'wheels/%s/pid'", wheel_name.c_str());
    return false;
  }

  nh.param("target_velocity", wheel.target_velocity, 0.0);
  ROS_INFO("%-12s joint=%-24s target=%6.3f rad/s", wheel_name.c_str(), joint_name.c_str(),
           wheel.target_velocity);
  return true;
}

void HeroChassisController::resetWheel(Wheel &wheel) {
  wheel.pid.reset(0.0, 0.0);
}

void HeroChassisController::updateWheel(Wheel &wheel, const ros::Duration &period) {
  const double error = wheel.target_velocity - wheel.joint.getVelocity();
  wheel.joint.setCommand(wheel.pid.computeCommand(error, period));
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &controller_nh) {
  return initWheel(left_front_, "left_front", effort_joint_interface, controller_nh) &&
         initWheel(right_front_, "right_front", effort_joint_interface, controller_nh) &&
         initWheel(left_back_, "left_back", effort_joint_interface, controller_nh) &&
         initWheel(right_back_, "right_back", effort_joint_interface, controller_nh);
}

void HeroChassisController::starting(const ros::Time & /*time*/) {
  resetWheel(left_front_);
  resetWheel(right_front_);
  resetWheel(left_back_);
  resetWheel(right_back_);
}

void HeroChassisController::update(const ros::Time & /*time*/, const ros::Duration &period) {
  updateWheel(left_front_, period);
  updateWheel(right_front_, period);
  updateWheel(left_back_, period);
  updateWheel(right_back_, period);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController, controller_interface::ControllerBase)
