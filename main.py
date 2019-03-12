#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Robot import Robot
# 全局变量
MAX_EPISODES = 500
MAX_EP_STEPS = 200

# 环境设置
robot = Robot()
# 学习方法
method = DDPG()

# 训练
for i in range(MAX_EPISODES):
    # s = robot.render()
    for j in range(MAX_EP_STEPS):
        s = robot.reset()
