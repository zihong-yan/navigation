base_local_planner
局部规划根据传感器的数据为机器人选择适当的线速度、角速度，来完成全局路径当前局部片段的执行。局部规划从全局和局部costmap中选择一条路径执行，因此局部规划可以重新计算机器人的路径，以防止撞击物体，但仍然需要它到达目的地。为了有效地评分轨迹，使用了栅格地图。对于每个控制周期，在机器人周围创建一个局部网格（局部地图的大小），并将全局路径映射到该区域。这意味着某些栅格单元(全局路径经过的栅格)将被标记为距离0。然后有效地将所有其他单元的距离标记为到最近的被标记为0的栅格的曼哈顿距离。

在TrajectoryCostFunction定义了接口scoreTrajectory()用来评估每一条轨迹的代价。
TrajectorySampleGenerator产生一系列轨迹，然后TrajectoryCostFunction遍历轨迹打分，TrajectorySearch找到最好的轨迹拿来给小车导航；由于小车不是一个质点，worldModel会检查小车有没有碰到障碍物。
同前面所讲，在机器人周围创建一个局部网格（局部地图的大小），并将全局路径映射到该区域。这意味着某些栅格单元(全局路径经过的栅格)将被标记为距离0。然后有效地将所有其他单元的距离标记为到最近的被标记为0的栅格的曼哈顿距离。MapGridCostFunction就是局部规划的路径离全局规划的路径的距离，也能够来评估到目标的距离，它维护了一个base_local_planner::MapGrid map_，MapGrid是一系列MapCell，而MapCell包含了一个target_dist，也就是说，
MapGridCostFunction建立后随时知道地图上一个点到全局规划轨迹的距离，或者是到目标的距离。具体是在computeTargetDistance中实现的。
ObstacleCostFunction就是计算机器人在局部的costmap上路径的代价，检测是否撞到障碍。其中ObstacleCostFunction用到了worldModel(如下图)接口来检测底座是不是撞到障碍物，从初始化函数可以看出来，ObstacleCostFunction选用了CostmapModel作为具体实现，想改为其他具体实现随时可以改这一行。
OscillationCostFunction:（震荡成本函数）振荡发生在x、y或维数中，正负值连续选择。为了防止振荡，当机器人向任何方向移动时，在下一个周期中，相反的方向将被标记为无效，直到机器人移动到距离设置标记的位置超过一定距离为止。
PreferForwardCostFunction:这个成本函数类在设计时考虑到机器人只在机器人(倾斜激光)前面具有良好的传感器覆盖。成本函数喜欢在前面方向的运动，惩罚向后和扫射运动。在其他机器人或其他领域，这可能是非常不受欢迎的行为。
类的参数与动态调整 Class parameters and dynamic adjustment
在这些类中出现的参数可以被分类为:
The parameters that appear in these classes can be classified as:
-robot configuration
-goal tolerance
-forward simulation
-trajectory scoring
-oscillation prevention
-global plan
在参数调整中会很麻烦，使用rqt_reconfigure toll来动态的调整参数:
rosrun rqt_reconfigure rqt_reconfigure
主体计算 Principal Computation
主体的思想在TrajectoryPlannerROS::computerVelocityCommands():
  bool TrajectoryPlannerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
    if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> local_plan;
    tf::Stamped<tf::Pose> global_pose;
    if (!costmap_ros_->getRobotPose(global_pose)) {
      return false;
    }
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    //get the global plan in our frame
    if (!transformGlobalPlan(*tf_, global_plan_, global_pose, *costmap_, global_frame_, transformed_plan)) {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }
    //now we'll prune the plan based on the position of the robot
    if(prune_plan_)
      prunePlan(global_pose, transformed_plan, global_plan_);
    tf::Stamped<tf::Pose> drive_cmds;
    drive_cmds.frame_id_ = robot_base_frame_;
    tf::Stamped<tf::Pose> robot_vel;
    odom_helper_.getRobotVel(robot_vel);
    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
      return false;
    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    //we assume the global goal is the last point in the global plan
    double goal_x = goal_point.getOrigin().getX();
    double goal_y = goal_point.getOrigin().getY();
    double yaw = tf::getYaw(goal_point.getRotation());
    double goal_th = yaw;
    //check to see if we've reached the goal position
    if (xy_tolerance_latch_ || (getGoalPositionDistance(global_pose, goal_x, goal_y) <= xy_goal_tolerance_)) {
      //if the user wants to latch goal tolerance, if we ever reach the goal location, we'll
      //just rotate in place
      if (latch_xy_goal_tolerance_) {
        xy_tolerance_latch_ = true;
      }
      double angle = getGoalOrientationAngleDifference(global_pose, goal_th);
      //check to see if the goal orientation has been reached
      if (fabs(angle) <= yaw_goal_tolerance_) {
        //set the velocity command to zero
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        rotating_to_goal_ = false;
        xy_tolerance_latch_ = false;
        reached_goal_ = true;
      } else {
        //we need to call the next two lines to make sure that the trajectory
        //planner updates its path distance and goal distance grids
        tc_->updatePlan(transformed_plan);
        Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
        map_viz_.publishCostCloud(costmap_);
        //copy over the odometry information
        nav_msgs::Odometry base_odom;
        odom_helper_.getOdom(base_odom);
        //if we're not stopped yet... we want to stop... taking into account the acceleration limits of the robot
        if ( ! rotating_to_goal_ && !base_local_planner::stopped(base_odom, rot_stopped_velocity_, trans_stopped_velocity_)) {
          if ( ! stopWithAccLimits(global_pose, robot_vel, cmd_vel)) {
            return false;
          }
        }
        //if we're stopped... then we want to rotate to goal
        else{
          //set this so that we know its OK to be moving
          rotating_to_goal_ = true;
          if(!rotateToGoal(global_pose, robot_vel, goal_th, cmd_vel)) {
            return false;
          }
        }
      }
      //publish an empty plan because we've reached our goal position
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      //we don't actually want to run the controller when we're just rotating to goal
      return true;
    }
    tc_->updatePlan(transformed_plan);
    //compute what trajectory to drive along
    Trajectory path = tc_->findBestPath(global_pose, robot_vel, drive_cmds);
    map_viz_.publishCostCloud(costmap_);
    //pass along drive commands
    cmd_vel.linear.x = drive_cmds.getOrigin().getX();
    cmd_vel.linear.y = drive_cmds.getOrigin().getY();
    cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());
    //if we cannot move... tell someone
    if (path.cost_ < 0) {
      ROS_DEBUG_NAMED("trajectory_planner_ros",
          "The rollout planner failed to find a valid plan. This means that the footprint of the robot was in collision for all simulated trajectories.");
      local_plan.clear();
      publishPlan(transformed_plan, g_plan_pub_);
      publishPlan(local_plan, l_plan_pub_);
      return false;
    }
    ROS_DEBUG_NAMED("trajectory_planner_ros", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.",
        cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    // Fill out the local plan
    for (unsigned int i = 0; i < path.getPointsSize(); ++i) {
      double p_x, p_y, p_th;
      path.getPoint(i, p_x, p_y, p_th);
      tf::Stamped<tf::Pose> p =
          tf::Stamped<tf::Pose>(tf::Pose(
              tf::createQuaternionFromYaw(p_th),
              tf::Point(p_x, p_y, 0.0)),
              ros::Time::now(),
              global_frame_);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(p, pose);
      local_plan.push_back(pose);
    }
    //publish information to the visualizer
    publishPlan(transformed_plan, g_plan_pub_);
    publishPlan(local_plan, l_plan_pub_);
    return true;
  }
————————————————
版权声明：本文为CSDN博主「Nksjc」的原创文章，遵循 CC 4.0 BY-SA 版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/Nksjc/article/details/78832665

