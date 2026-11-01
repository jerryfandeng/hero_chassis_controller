//
// 麦克纳姆轮底盘控制器 —— ros_control 插件实现。
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>

namespace hero_chassis_controller {

/*!
 * \brief RoboMaster 英雄机器人麦克纳姆轮底盘控制器。
 *
 * 每个轮子一个独立的 control_toolbox::Pid（各自持有积分与微分状态，不能共用）。
 * 目标转速来自 /cmd_vel 经逆运动学换算；超时未收到指令时四轮目标归零。
 */
class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  // 三参数版本：需要 root_nh 才能订阅全局 /cmd_vel。
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;

  void starting(const ros::Time &time) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  //! 一个轮子的全部状态：句柄、PID、目标转速。
  struct Wheel {
    hardware_interface::JointHandle joint;
    control_toolbox::Pid pid;
    double target_velocity{0.0};
  };

  bool initWheel(Wheel &wheel, const std::string &wheel_name,
                 hardware_interface::EffortJointInterface *effort_joint_interface,
                 ros::NodeHandle &controller_nh);
  void updateWheel(Wheel &wheel, const ros::Duration &period);
  void resetWheel(Wheel &wheel);

  //!cmd_vel 回调：只做逆运动学，不碰 PID。
  void cmdVelCallback(const geometry_msgs::Twist &msg);
  //! 逆运动学：底盘速度 -> 四个轮子的目标转速。
  void setWheelTargets(double vx, double vy, double wz);
  void stopAllWheels();

  Wheel left_front_;
  Wheel right_front_;
  Wheel left_back_;
  Wheel right_back_;

  ros::Subscriber cmd_vel_sub_;
  ros::Time last_cmd_vel_time_;

  // 几何与超时参数，来自 config/controllers.yaml，默认值取自 URDF。
  double wheel_radius_{0.07625};
  double wheel_base_{0.4};    // 前后轮间距
  double wheel_track_{0.4};   // 左右轮间距
  double cmd_vel_timeout_{0.5};
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
