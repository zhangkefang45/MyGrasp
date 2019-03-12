#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import numpy as np
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose


class Robot():
    dt = 0.1  # 转动的速度和 dt 有关
    action_bound = [-1.57, 1.57]  # 转动的角度范围
    state_dim = 6  # 六个观测值
    action_dim = 6  # 六个动作
    def __init__(self):
        rospy.init_node('trajectory_demo')

        # 是否需要回到初始化的位置
        self.reset = rospy.get_param('~reset', False)

        # 机械臂中joint的命名
        self.arm_joints = ['shoulder_pan_joint',
                      'shoulder_lift_joint',
                      'elbow_joint',
                      'wrist_1_joint',
                      'wrist_2_joint',
                      'wrist_3_joint']



        self.arm_goal = []
        if self.reset:
            # 如果需要回到初始化位置，需要将目标位置设置为初始化位置的六轴角度
            self.arm_goal = [0, 0, 0, 0, 0, 0]

        else:
            # 如果不需要回初始化位置，则设置目标位置的六轴角度
            self.arm_goal = [0, 0, 0, -1.57, -1.57, -1.57]

    # 连接机械臂轨迹规划的trajectory action server
    @staticmethod
    def connect():
        rospy.loginfo('Waiting for arm trajectory controller...')
        arm_client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        arm_client.wait_for_server()
        rospy.loginfo('...connected.')
        return arm_client

    # 使用设置的目标位置创建一条轨迹数据
    def plan(self, arm_client):
        arm_trajectory = JointTrajectory()
        arm_trajectory.joint_names = self.arm_joints
        arm_trajectory.points.append(JointTrajectoryPoint())
        arm_trajectory.points[0].positions = self.arm_goal
        arm_trajectory.points[0].velocities = [0.0 for i in self.arm_joints]
        arm_trajectory.points[0].accelerations = [0.0 for i in self.arm_joints]
        arm_trajectory.points[0].time_from_start = rospy.Duration(3.0)
        rospy.loginfo('Moving the arm to goal position...')

        # 创建一个轨迹目标的空对象
        arm_goal = FollowJointTrajectoryGoal()

        # 将之前创建好的轨迹数据加入轨迹目标对象中
        arm_goal.trajectory = arm_trajectory

        # 设置执行时间的允许误差值
        arm_goal.goal_time_tolerance = rospy.Duration(0.0)

        # 将轨迹目标发送到action server进行处理，实现机械臂的运动控制
        arm_client.send_goal(arm_goal)

        # 等待机械臂运动结束
        arm_client.wait_for_result(rospy.Duration(5.0))

        rospy.loginfo('...done')

    # 渲染
    def render(self):
        pass

    #
    def step(self, action):
        done = False
        reawrd = 0

        action = np.clip(action, self.action_bound)

        


    # 初始化
    def reset(self):
        self.arm_goal = [-1.57, 0, 0, 0, 0, 0]
        arm_client = self.connect()
        self.plan(arm_client)


    # 设置场景物体的颜色
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
        self.colors[name] = color

    # 将颜色设置发送并应用到moveit场景当中
    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异
        p.is_diff = True

        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)

        # 发布场景物体颜色设置
        self.scene_pub.publish(p)


if __name__ == '__main__':
    try:
        robot = Robot()
        robot.render()
    except rospy.ROSInterruptException:
        pass
