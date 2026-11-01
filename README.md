# hero_chassis_controller

RoboMaster 英雄机器人麦克纳姆轮底盘的控制器，按 `ros_control` 插件实现，通过
`pluginlib` 注册为 `controller_interface::ControllerBase`，只依赖 `EffortJointInterface`。
本包由 `gdut-dynamic-x/rm_template` 改造而来。

## 实现

每个轮子一个独立的 `control_toolbox::Pid` 做速度环：误差 = 目标转速 − 实测转速，
输出力矩写入 `EffortJointInterface`。四个轮子的句柄、PID 与目标速度封装在
`struct Wheel` 中，共四个实例，互不共享状态。

### 速度控制

`update()` 每个周期读 `/cmd_vel`（`geometry_msgs/Twist`）最近一次的目标速度， 经麦克纳姆轮逆运动学换算成四个轮子的目标角速度，再由各自的 `control_toolbox::Pid`追到实测转速并输出力矩。

#### 逆运动学公式

ω_LF = (vx − vy − K·ω_z) / r
ω_RF = (vx + vy + K·ω_z) / r
ω_LB = (vx + vy − K·ω_z) / r
ω_RB = (vx − vy + K·ω_z) / r


### 测试

`src/cmd_vel_demo_node.cpp` 是 `/cmd_vel` 话题的发布者，用于发布底盘运动消息
（`geometry_msgs/Twist` 下的 x、y 方向线速度和绕 z 轴的角速度），测试底盘运动。
它以 10 Hz 依次发送前进、左右平移、正反转自转六种运动，每种 3 秒。

## 依赖

- ROS Noetic（Ubuntu 20.04）
- `control_toolbox`、`controller_interface`、`hardware_interface`、`pluginlib`
- `rm_description_for_task` —— 机器人描述与 `hero.launch`
- `rm_gazebo` —— 仿真侧 RobotHW（`librm_robot_hw_sim.so`）

## 构建
```bash
cd ~/caktin_ws/src
git clone https://github.com/jerryfandeng/hero_chassis_controller.git
source /opt/ros/noetic/setup.bash
cd .. && catkin build hero_chassis_controller
```