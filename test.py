#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose

colors = dict()
def setColor(self, name, r, g, b, a=0.9):
    # 初始化moveit颜色对象
    color = ObjectColor()

    # 设置颜色值
    color.id = name
    color.color.r = r
    color.color.g = g
    color.color.b = b
    color.color.a = a

    # 更新颜色字典
    colors[name] = color

# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)

# 初始化ROS节点
rospy.init_node('moveit_obstacles_demo')

# 初始化场景对象
scene = PlanningSceneInterface()

# 创建一个发布场景变化信息的发布者
scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)

# 等待场景准备就绪
rospy.sleep(1)

# 初始化需要使用move group控制的机械臂中的arm group
arm = MoveGroupCommander('manipulator')

# 获取终端link的名称
end_effector_link = arm.get_end_effector_link()

# 设置位置(单位：米)和姿态（单位：弧度）的允许误差
arm.set_goal_position_tolerance(0.01)
arm.set_goal_orientation_tolerance(0.05)

# 当运动规划失败后，允许重新规划
arm.allow_replanning(True)

# 设置目标位置所使用的参考坐标系
reference_frame = 'base_link'
arm.set_pose_reference_frame(reference_frame)

# 设置每次运动规划的时间限制：5s
arm.set_planning_time(5)

box1_id = 'box1'

# 移除场景中之前运行残留的物体
# scene.remove_world_object(table_id)
scene.remove_world_object(box1_id)
# scene.remove_world_object(box2_id)
rospy.sleep(1)

# 控制机械臂先回到初始化位置
arm.set_named_target('up')
arm.go()
rospy.sleep(2)
box1_size = [0.1, 0.05, 0.05]

box1_pose = PoseStamped()
box1_pose.header.frame_id = reference_frame
box1_pose.pose.position.x = 0.71
box1_pose.pose.position.y = -0.1
box1_pose.pose.position.z = 0.5
box1_pose.pose.orientation.w = 1.0
scene.add_box(box1_id, box1_pose, box1_size)

setColor(box1_id, 0.8, 0.4, 0, 1.0)


arm.set_pose_target(box1_pose, end_effector_link)
arm.go()
rospy.sleep(2)

arm.set_named_target('up')
arm.go()
rospy.sleep(2)