# hero_chassis_controller

RoboMaster 英雄机器人麦克纳姆轮底盘的控制器，按 `ros_control` 插件实现，通过
`pluginlib` 注册为 `controller_interface::ControllerBase`，只依赖 `EffortJointInterface`。
本包由 `gdut-dynamic-x/rm_template` 改造而来。

## 实现

每个轮子一个独立的 `control_toolbox::Pid` 做速度环：误差 = 目标转速 − 实测转速，
输出力矩写入 `EffortJointInterface`。四个轮子的句柄、PID 与目标速度封装在
`struct Wheel` 中，共四个实例，互不共享状态。

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