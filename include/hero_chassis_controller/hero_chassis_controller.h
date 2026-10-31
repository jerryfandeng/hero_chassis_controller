//
// 麦克纳姆轮底盘控制器 —— ros_control 插件实现。
//

#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <string>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &controller_nh) override;

  void starting(const ros::Time &time) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

 private:
  // 一个轮子的全部状态：句柄、PID、目标转速。
  struct Wheel {
    hardware_interface::JointHandle joint;
    control_toolbox::Pid pid;
    double target_velocity{0.0};
  };

  // 普通成员函数
  bool initWheel(Wheel &wheel, const std::string &wheel_name,
                 hardware_interface::EffortJointInterface *effort_joint_interface,
                 ros::NodeHandle &controller_nh);
  void updateWheel(Wheel &wheel, const ros::Duration &period);
  void resetWheel(Wheel &wheel);
  Wheel left_front_;
  Wheel right_front_;
  Wheel left_back_;
  Wheel right_back_;
};
}  // namespace hero_chassis_controller
#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
