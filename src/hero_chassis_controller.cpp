//
// 麦克纳姆轮底盘控制器 —— ros_control 插件实现。
//

#include "hero_chassis_controller/hero_chassis_controller.h"

#include <angles/angles.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <pluginlib/class_list_macros.hpp>
namespace hero_chassis_controller {

bool HeroChassisController::initWheel(
    Wheel& wheel, const std::string& wheel_name,
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& controller_nh) {
  ros::NodeHandle nh(controller_nh, "wheels/" + wheel_name);

  std::string joint_name;
  if (!nh.getParam("joint", joint_name)) {
    ROS_ERROR("Missing param 'wheels/%s/joint', check config/controllers.yaml", wheel_name.c_str());
    return false;
  }

  // getHandle 在关节不存在时抛 HardwareInterfaceException，
  // controller_manager 会捕获并让 spawner 加载失败 —— 关节名写错立刻暴露。
  wheel.joint = effort_joint_interface->getHandle(joint_name);

  // Pid::init 从 <wheels>/<name>/pid 读 p / i / d / i_clamp / antiwindup。
  if (!wheel.pid.init(ros::NodeHandle(nh, "pid"))) {
    ROS_ERROR("Failed to load PID gains from 'wheels/%s/pid'", wheel_name.c_str());
    return false;
  }

  ROS_INFO("%-12s joint=%-24s init_target=%6.3f rad/s", wheel_name.c_str(), joint_name.c_str(),
           wheel.target_velocity);
  return true;
}

void HeroChassisController::resetWheel(Wheel& wheel) { wheel.pid.reset(0.0, 0.0); }

void HeroChassisController::updateWheel(Wheel& wheel, const ros::Duration& period) {
  const double error = wheel.target_velocity - wheel.joint.getVelocity();
  wheel.joint.setCommand(wheel.pid.computeCommand(error, period));
}

void HeroChassisController::stopAllWheels() {
  left_front_.target_velocity = 0.0;
  right_front_.target_velocity = 0.0;
  left_back_.target_velocity = 0.0;
  right_back_.target_velocity = 0.0;
}

void HeroChassisController::setWheelTargets(double vx, double vy, double wz) {
  // K = 半轴距 + 半轮距，是轮心到旋转中心距离在轮子驱动方向上的投影（力臂）。
  const double k = 0.5 * (wheel_base_ + wheel_track_);
  const double r = wheel_radius_;

  // 符号来源：simple_chassis_controller 的 cmd_ 表（前进、左移、顺时针）。
  // 若实测发现前进变后退，整体取反即可。
  left_front_.target_velocity = (vx - vy - k * wz) / r;
  right_front_.target_velocity = (vx + vy + k * wz) / r;
  left_back_.target_velocity = (vx + vy - k * wz) / r;
  right_back_.target_velocity = (vx - vy + k * wz) / r;
}

void HeroChassisController::computeChassisVelocity(double& vx, double& vy, double& wz) const {
  const double k = 0.5 * (wheel_base_ + wheel_track_);
  const double r = wheel_radius_;

  // 必须是实际转速，不能是 target_velocity：PID 有稳态误差，
  const double w_lf = left_front_.joint.getVelocity();
  const double w_rf = right_front_.joint.getVelocity();
  const double w_lb = left_back_.joint.getVelocity();
  const double w_rb = right_back_.joint.getVelocity();

  //正运动学公式
  vx = r * (w_lf + w_rf + w_lb + w_rb) / 4.0;
  vy = r * (-w_lf + w_rf + w_lb - w_rb) / 4.0;
  wz = r * (-w_lf + w_rf - w_lb + w_rb) / (4.0 * k);
}

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& msg) {
  last_cmd_vel_time_ = ros::Time::now();
  cmd_vx_ = msg.linear.x;
  cmd_vy_ = msg.linear.y;
  cmd_wz_ = msg.angular.z;

  // 全局模式朝向锁：发角速度=解锁自由转；只发线速度=锁存当前朝向。
  if (cmd_vel_in_world_) {
    if (std::abs(cmd_wz_) > 1e-4) {
      yaw_locked_ = false;
    } else if (!yaw_locked_) {
      yaw_target_ = odometry_.getYaw();
      yaw_locked_ = true;
    }
  }
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                 ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  if (!(initWheel(left_front_, "left_front", effort_joint_interface, controller_nh) &&
        initWheel(right_front_, "right_front", effort_joint_interface, controller_nh) &&
        initWheel(left_back_, "left_back", effort_joint_interface, controller_nh) &&
        initWheel(right_back_, "right_back", effort_joint_interface, controller_nh))) {
    return false;
  }

  controller_nh.param("geometry/wheel_radius", wheel_radius_, wheel_radius_);
  controller_nh.param("geometry/wheel_base", wheel_base_, wheel_base_);
  controller_nh.param("geometry/wheel_track", wheel_track_, wheel_track_);
  controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
  controller_nh.param("cmd_vel_in_world", cmd_vel_in_world_, cmd_vel_in_world_);

  // 必须用 root_nh：controller_nh 的命名空间是 /controller/hero_chassis_controller，
  // 用它订阅会变成 /controller/hero_chassis_controller/cmd_vel，收不到 rostopic pub /cmd_vel。
  cmd_vel_sub_ = root_nh.subscribe("cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

  if (!odometry_.init(root_nh, controller_nh)) {
    return false;
  }

  // 朝向纠偏 PID，参数在 yaml 的 pid_follow 段。
  if (!pid_yaw_.init(ros::NodeHandle(controller_nh, "pid_follow"))) {
    ROS_ERROR("Failed to load pid_follow gains");
    return false;
  }

  ROS_INFO("cmd_vel ready: r=%.5f base=%.3f track=%.3f K=%.3f timeout=%.2f", wheel_radius_,
           wheel_base_, wheel_track_, 0.5 * (wheel_base_ + wheel_track_), cmd_vel_timeout_);
  return true;
}

void HeroChassisController::starting(const ros::Time& /*time*/) {
  resetWheel(left_front_);
  resetWheel(right_front_);
  resetWheel(left_back_);
  resetWheel(right_back_);
  last_cmd_vel_time_ = ros::Time(0);  // 视为从未收到指令
  stopAllWheels();
  cmd_vx_ = 0.0;
  cmd_vy_ = 0.0;
  cmd_wz_ = 0.0;
  yaw_locked_ = false;
  pid_yaw_.reset();
  odometry_.reset();  // 位姿回原点，否则 restart 后接着上次漂
}

void HeroChassisController::update(const ros::Time& time, const ros::Duration& period) {
  // 超时保护：超过 cmd_vel_timeout 没收到新指令就停车。
  if (!last_cmd_vel_time_.isZero() && (time - last_cmd_vel_time_).toSec() > cmd_vel_timeout_) {
    cmd_vx_ = 0.0;
    cmd_vy_ = 0.0;
    cmd_wz_ = 0.0;
    yaw_locked_ = false;  // 解锁
    pid_yaw_.reset();     // 清朝向 PID 状态
  }

  double vx_b = cmd_vx_;
  double vy_b = cmd_vy_;
  double wz_b = cmd_wz_;

  if (cmd_vel_in_world_) {
    // 用里程计积分的朝向角和旋转矩阵将全局系速度变换为底盘系速度
    const double yaw = odometry_.getYaw();
    vx_b = std::cos(yaw) * cmd_vx_ + std::sin(yaw) * cmd_vy_;
    vy_b = -std::sin(yaw) * cmd_vx_ + std::cos(yaw) * cmd_vy_;

    // 朝向纠偏：锁定 yaw_target_，PID 输出 wz 修正（error = target - state）。
    if (yaw_locked_) {
      const double yaw_err = angles::shortest_angular_distance(yaw, yaw_target_);
      wz_b = pid_yaw_.computeCommand(yaw_err, period);
    }
  }
  setWheelTargets(vx_b, vy_b, wz_b);
  updateWheel(left_front_, period);
  updateWheel(right_front_, period);
  updateWheel(left_back_, period);
  updateWheel(right_back_, period);

  // 正运动学 + 里程计：用实际轮速积分位姿并发布 /odom 与 odom->base_link。
  double vx = 0.0;
  double vy = 0.0;
  double wz = 0.0;
  computeChassisVelocity(vx, vy, wz);
  odometry_.update(vx, vy, wz, time, period);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)
