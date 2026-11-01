//
// /cmd_vel 演示发布者：按固定序列发 Twist，覆盖六种运动模式。
// 用 rostopic echo /cmd_vel 可以同时观察数据变化。
//

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <vector>

namespace {

struct Phase {
  const char *name;
  double duration;  // s
  double vx, vy, wz;
};

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "cmd_vel_demo");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // 等待订阅者（控制器）连上，否则前几条消息会丢。
  while (pub.getNumSubscribers() == 0 && ros::ok()) {
    ROS_INFO_THROTTLE(1.0, "waiting for /cmd_vel subscriber (controller) ...");
    ros::Duration(0.2).sleep();
  }

  const std::vector<Phase> phases{
      {"stop", 2.0, 0.0, 0.0, 0.0},
      {"forward", 3.0, 0.15, 0.0, 0.0},
      {"stop", 1.5, 0.0, 0.0, 0.0},
      {"strafe_left", 3.0, 0.0, 0.15, 0.0},
      {"stop", 1.5, 0.0, 0.0, 0.0},
      {"strafe_right", 3.0, 0.0, -0.15, 0.0},
      {"stop", 1.5, 0.0, 0.0, 0.0},
      {"spin_ccw", 3.0, 0.0, 0.0, 0.5},
      {"stop", 1.5, 0.0, 0.0, 0.0},
      {"spin_cw", 3.0, 0.0, 0.0, -0.5},
      {"stop", 2.0, 0.0, 0.0, 0.0},
  };

  ros::Rate rate(10.0);  // 10 Hz，与常见 teleop 频率一致

  for (const auto &p : phases) {
    if (!ros::ok()) {
      break;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = p.vx;
    msg.linear.y = p.vy;
    msg.angular.z = p.wz;

    ROS_INFO("[%-13s] linear.x=%+.2f linear.y=%+.2f angular.z=%+.2f  (%.1f s)", p.name, p.vx, p.vy, p.wz,
             p.duration);

    const ros::Time end = ros::Time::now() + ros::Duration(p.duration);
    while (ros::ok() && ros::Time::now() < end) {
      pub.publish(msg);
      ros::spinOnce();
      rate.sleep();
    }
  }

  pub.publish(geometry_msgs::Twist());  // 收尾归零
  ROS_INFO("demo finished, published zero twist");
  return 0;
}
