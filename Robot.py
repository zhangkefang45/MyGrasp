#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import numpy as np
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor



class Robot():
    dt = 0.1  # 转动的速度和 dt 有关
    action_bound = [-1.57, 1.57]  # 转动的角度范围
    state_dim = 6  # 六个观测值
    action_dim = 6  # 六个动作

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
        # 初始化move_group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('arm')
        self.gripper = moveit_commander.MoveGroupCommander('gripper')
        # 机械臂的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)
        self.gripper.set_goal_joint_tolerance(0.001)
        self.arm_goal = [0, 0, 0, 0, 0, 0]
        # 控制机械臂竖起
        self.arm.set_named_target('up')
        self.arm.go()
        rospy.sleep(2)
        # 控制夹爪运动
        self.gripper.set_joint_value_target([0.6])
        self.gripper.go()
        rospy.sleep(2)
        #self.reset()

    # 渲染
    def render(self):
        pass

    #
    def step(self, action):
        done = False
        reawrd = 0

        action = np.clip(action, self.action_bound)
        # 转动一定的角度
        self.arm_goal + action*self.dt
        self.arm_goal %= 2*np.pi

        s = self.arm_goal

        # 如果和物体的距离小于一定值，则回合结束
        position = self.arm.get_current_pose('wrist_3_link').pose.position
        end_x, end_y, end_z = position.x, position.y, position.z
        # print(end_x, end_y, end_z)


    # 初始化
    def reset(self):
        self.arm_goal = [0, 0, 0, 0, 0, 0]
        self.arm.set_joint_value_target(self.arm_goal)
        self.arm.go()
        rospy.sleep(1)

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

    def sample(self):
        return np.random.rand(6)-0.5


if __name__ == '__main__':
    robot = Robot()

