ROS Navigation Stack
====================

A 2D navigation stack that takes in information from odometry, sensor
streams, and a goal pose and outputs safe velocity commands that are sent
to a mobile base.

 * AMD64 Debian Job Status: [![Build Status](http://build.ros.org/buildStatus/icon?job=Kbin_uX64__navigation__ubuntu_xenial_amd64__binary)](http://build.ros.org/view/Kbin_uX64/job/Kbin_uX64__navigation__ubuntu_xenial_amd64__binary/)

Related stacks:

 * http://github.com/ros-planning/navigation_msgs (new in Jade+)
 * http://github.com/ros-planning/navigation_tutorials
 * http://github.com/ros-planning/navigation_experimental

For discussion, please check out the
https://groups.google.com/group/ros-sig-navigation mailing list.

1.base_local_planner(或者TrajectoryPlannerROS)  

base_local_planner::TrajectoryPlannerROS对象是base_local_planner::TrajectoryPlanner对象的ROS封装，在初始化时指定的ROS命名空间使用，继承了nav_core::BaseLocalPlanner接口。它是move_base默认的局部规划包。该软件包提供了对平面上本地机器人导航的轨迹展开和动态窗口方法的实现。根据计划遵循和成本图，控制器生成速度命令以发送到移动基站。该软件包支持完整和非完整机器人，可以表示为凸多边形或圆形的任何机器人足迹，并将其配置公开为可在启动文件中设置的ROS参数。此包的ROS包装器遵循nav_core包中指定的BaseLocalPlanner接口。与dwa_local_planner思路接近。  

唯一区别是DWA与“TrajectoryPlanner”的不同之处在于如何对机器人的控制空间进行采样。在给定机器人的加速度极限的情况下，TrajectoryPlanner在整个前向模拟周期内从可实现的速度集合中进行采样，而DWA在给定机器人的加速度极限的情况下仅针对一个模拟步骤从可实现的速度集合中进行采样。在实践中，我们发现DWA和轨迹展示在我们的所有测试中都具有相同的性能，并建议使用DWA来提高效率，因为其样本空间更少。  
