//
// 里程计发布器：由底盘速度积分出位姿，发布 nav_msgs/Odometry 与 TF odom->base_link。
// 不持有任何 JointHandle —— 速度由调用方计算后传入，便于脱离 ros_control 单测。
//

#ifndef HERO_CHASSIS_CONTROLLER_ODOMETRY_PUBLISHER_H
#define HERO_CHASSIS_CONTROLLER_ODOMETRY_PUBLISHER_H

#include <string>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace hero_chassis_controller {

class OdometryPublisher {
 public:
  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

  //! 位姿清零。控制器 restart 时必须调用，否则位姿接着上次漂。
  void reset();

  //! 每周期调用：先积分，再按 publish_rate_ 降频发布。
  void update(double vx, double vy, double wz, const ros::Time &time, const ros::Duration &period);

  //! 当前朝向角（yaw），供全局坐标系速度模式做世界->底盘旋转。
  double getYaw() const { return yaw_; }

 private:
  ros::Publisher odom_pub_;
  tf::TransformBroadcaster odom_broadcaster_;

  double x_{0.0};
  double y_{0.0};
  double yaw_{0.0};
  ros::Time last_pub_time_;

  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  double publish_rate_{50.0};
  double max_odom_vel_{5.0};  // 超过该速度不积分，防一帧尖峰污染位姿（正常行驶 0.1~2 m/s）
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_ODOMETRY_PUBLISHER_H
