# -*- coding: utf-8 -*-
"""
Created on Thu Jul  6 10:44:12 2023

@author: Menzel_Asus2019
"""

import numpy as np
from sklearn import tree

def allowed_actions(self, s1):
    # Generate list of actions allowed depending on nao leg state
    actions_allowed = []
    if (s1 < self.Ns_leg - 2):  # No passing furthest left kick
        actions_allowed.append(self.action_dict["left"])
    if (s1 > 1):  # No passing furthest right kick
        actions_allowed.append(self.action_dict["right"])
    actions_allowed.append(self.action_dict["kick"]) # always able to kick
    actions_allowed = np.array(actions_allowed, dtype=int)
    return actions_allowed

# Initial constants
NS_LEG = 10
NS_GK = 20
N_ACTIONS = NS_LEG
NUM_EPISODES = 200
kicked = 0
cumR = 0
epsilon = 30


# Initialize Decision Trees
s1_tree = tree.DecisionTreeClassifier()
s2_tree = tree.DecisionTreeClassifier()
R_tree = tree.DecisionTreeClassifier()

# Initialize input (always same) and output vectors for trees
x_array = np.zeros(3)
deltaS1 = np.array((0))
deltaS2 = np.array((0))
deltaR = np.array((0))

# Define rewards
goal_reward = 20 # Reward for scoring goal
miss_penalty = -2 # Miss the goal
fall_penalty = -20 # Penalty for falling over
action_penalty = -1 # Penalty for each action execution

# Learning parameters
gamma = 0.001 # Discount factor
        

# State sets
Sm1 = np.zeros(NS_LEG)
Sm2 = np.zeros(NS_GK)

# Define quantized action space
RHipRoll_actions = np.linspace(-0.5, -0.8, N_ACTIONS) # Number hip roll actions
RHipPitch_actions = np.array((0.2, -1.4))

# Define actions
action_dict = {"left": 0, "right": 1, "kick": 2}
action_list = [0, 1, 2]
action_translations = [(-1, 0), (1, 0), (0, 1)] # action translations within quantized action space

# Visit count
visits = np.zeros((NS_LEG, NS_GK, len(action_list)))

# Prob transitions
Pm = np.zeros((NS_LEG, NS_GK, len(action_list)))
Rm = np.zeros((NS_LEG, NS_GK, len(action_list)))
Q = np.zeros((NS_LEG, NS_GK, len(action_list)))

while(1):

    while(kicked == 0):
        
        # Get leg state -- 10 possible states
        min_state = RHipRoll_actions[NS_LEG-1]
        max_state = RHipRoll_actions[0]
        s1 = round((joint_angles[15] - min_state)/(max_state - min_state) * (NS_LEG-1))
        
        # Get goalie position x-position -- 10 possible states
        s2 = round(BlobX/320 * (NS_GK-1))
        
        actions_allowed = allowed_actions(s1)
    
        if randi() 
        chosen_action = actions_allowed[np.argmax(Q[s1, s2, actions_allowed])]
        
        #estimate reward through decision tree
        ##
        #Do a decision tree where we add up all the expected rewards/costs of each action
        ##
        
        R = np.max(cumR)
        
        Q[s1,s2,chosen_action] = gamma * Pm[s1,s2,chosen_action] * Q[s1,s2,chosen_action] + R
        
        ##
        #do action
        ##
        
    #wait for reward
    #update decision tree with reward and states visited
    
        
    





