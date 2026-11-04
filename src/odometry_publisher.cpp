//
// 里程计发布器实现。TODO 标记处由 Jerry 补完。
//

#include "hero_chassis_controller/odometry_publisher.h"

#include <cmath>

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

namespace hero_chassis_controller {

bool OdometryPublisher::init(ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  controller_nh.param("odom/publish_rate", publish_rate_, publish_rate_);
  controller_nh.param("odom/frame", odom_frame_, odom_frame_);
  controller_nh.param("odom/base_frame", base_frame_, base_frame_);
  controller_nh.param("odom/max_vel", max_odom_vel_, max_odom_vel_);

  if (publish_rate_ <= 0.0) {
    ROS_WARN("odom/publish_rate=%.2f 非法，回退到 50Hz", publish_rate_);
    publish_rate_ = 50.0;
  }

  // 用 root_nh
  // controller_nh 会发布到 /controller/hero_chassis_controller/odom。
  odom_pub_ = root_nh.advertise<nav_msgs::Odometry>("odom", 10);

  ROS_INFO("odom ready: frame=%s->%s rate=%.1fHz max_vel=%.2f", odom_frame_.c_str(), base_frame_.c_str(),
           publish_rate_, max_odom_vel_);
  return true;
}

void OdometryPublisher::reset() {
  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
  last_pub_time_ = ros::Time(0);
}
//接受底盘x,y线速度 绕z轴的角速度
void OdometryPublisher::update(double vx, double vy, double wz, const ros::Time &time,
                               const ros::Duration &period) {
  const double dt = period.toSec();

  // 世界系积分：机体系速度先旋转到 odom 系再累加，不能直接加。
  // 尖峰保护：仿真里关节速度偶尔跳变，一帧坏数据会永久污染位姿。
  if (std::sqrt(vx * vx + vy * vy) < max_odom_vel_) {
    x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
    y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
  }
  yaw_ += wz * dt;

  // 积分每拍都做（1000Hz），发布按 publish_rate_ 限流；反过来会导致位移丢失。
  if ((time - last_pub_time_).toSec() < 1.0 / publish_rate_) {
    return;
  }
  last_pub_time_ = time;

  const geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw_);

  // TF：parent=odom，child=base_link
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;
  odom_trans.header.frame_id = odom_frame_;
  odom_trans.child_frame_id = base_frame_;
  odom_trans.transform.translation.x = x_;
  odom_trans.transform.translation.y = y_;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  odom_broadcaster_.sendTransform(odom_trans);

  // pose 在 odom 系（积分结果），twist 在 base_link 系（机体系速度原值，不再旋转）。
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  odom_pub_.publish(odom);
}

}  // namespace hero_chassis_controller
