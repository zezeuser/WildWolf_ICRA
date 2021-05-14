# -*- coding: utf-8 -*-
# RoboMaster AI Challenge Simulator (RMAICS)

from kernal import kernal
import numpy as np
import torch
from copy import deepcopy
class rmaics(object):

    def __init__(self, agent_num, render=True):
        self.game = kernal(car_num=agent_num, render=render)
        self.g_map = self.game.get_map()
        self.memory = []
        self.env_name = "RMUA"
        self.action_dim= 1*7
        self.state_dim = 1*30
        self.max_step = 3600  #180s
        # self.if_discrete = True
        self.if_discrete = False
        self.target_return = 500
        self.save_last = 0

    def reset(self):
        self.state = self.game.reset()
        # state, object
        self.obs = self.get_observation(self.state)
        return self.obs

    def step(self, actions):
        if self.save_last % 5 == 0 :
            self.last_state = deepcopy(self.state)
        
        state = self.game.step(actions)

        state = self.game.step(actions)
        obs = self.get_observation(state)
        rewards = self.get_reward(state)

        self.memory.append([self.obs, actions, rewards])
        self.state = state

        return obs, rewards, state.done, None
    
    def get_observation(self, state):
        # personalize your observation here
        obs = np.zeros((1,30))
        obs[0,0:16] = state.agents[0,:]
        obs[0,17:23] = state.buff[:,0]
        obs[0,24:30] = state.buff[:,1]
        return obs
    
    def get_reward(self, state):
        rewards = 0 
        rewards_shoot = 0 
        # personalize your reward here
        hit_times = self.Hit_times(state)
        if self.shoot_car(state):
            rewards_shoot += 2
        rewards = hit_times * -0.2 + rewards_shoot 
        # print("rewards:",rewards)
        return rewards

    def play(self):
        self.game.play()

    def Hit_times(self, state):
        hit_times = 0
        #若此状态相比于上一状态撞墙次数增加则计数
        if state.agents[0][12] - self.last_state.agents[0][12]> 0 : 
            # print("HIT wall")
            hit_times += 1  
        elif state.agents[0][13] - self.last_state.agents[0][13] > 0 :
            # print("Hit wall")
            hit_times += 1  
        elif state.agents[0][14] - self.last_state.agents[0][14] > 0 :
            # print("Hit car")
            hit_times += 1  
        return hit_times
    
    def shoot_car(self , state):
        if  self.last_state.agents[1][6] - state.agents[1][6]  > 0 :
            # print("shoot")
            return True
        else:
            return False

    def save_record(self, file):
        self.game.save_record(file)
        
        
if __name__ == '__main__':
    import time
    # save_file_name = './records/record1.npy'
    save_file_name = './records/%d.npy'%int(time.time())
    game = rmaics(agent_num=4, render=True)
    game.reset()
    # only when render = True
    game.play()
    
#    from kernal import record_player
#    print('play saved file')
#    player = record_player()
#    player.play(save_file_name)