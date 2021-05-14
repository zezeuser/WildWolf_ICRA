# -*- coding: utf-8 -*-
from rmaics import rmaics
from kernal import record_player
from kernal import kernal
import numpy as np
from copy import deepcopy 
import torch
import torch.nn as nn
from elegantrl import agent
from elegantrl.run import Arguments, train_and_evaluate, train_and_evaluate_mp
from elegantrl.env import PreprocessEnv


# print(game.game.car_num)   car_num = 1
game = rmaics(agent_num=2, render=True)
state = game.reset()
#######state  = [1,30] , action = [4,7]##########

# actions = np.array( [ [1, 1, 1, 1, 1, 1, 1], [1, 1, 1, 1, 1, 1, 1] ] )
# state, rewards, done, _ = game.step(actions)
# print(actions.shape)


# 创建并初始化Agent
# Agent= agent.AgentD3QN()
Agent= agent.AgentDDPG()
#初始化训练参数
arguments = Arguments(agent = Agent, env = game , gpu_id= None)
arguments.if_per = True
# arguments.env = PreprocessEnv(env = game)
arguments.init_before_training()

#开始训练
train_and_evaluate(arguments)


