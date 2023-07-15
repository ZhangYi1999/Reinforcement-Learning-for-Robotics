#!/usr/bin/env python
#NAMES: Menzel Nicholas, Gacic Dajana, Zhang Yi, Causevic Azur
from sklearn import tree
import numpy as np
from enum import Enum
import copy

class Action(Enum):
    KICK = 0
    LEFT = 1
    RIGHT = 2

class Reinforce_Learning_Kick:

    def __init__(self, number_of_states, min_hip_angle, max_hip_angle, number_of_states_gk, centerX, hipjoint):
        # Total number of states
        self.number_of_states = number_of_states
        self.number_of_states_gk = number_of_states_gk

        self.min_hip_angle = min_hip_angle
        self.max_hip_angle = max_hip_angle
        
        # State sets
        self.Sm1 = np.zeros(number_of_states)
        self.Sm2 = np.zeros(number_of_states_gk)
        
        # Visits
        self.visits = np.zeros((self.number_of_states, self.number_of_states_gk, len(Action)))

        # Action space - 1 action only - HipRoll
        self.hip_actions = np.linspace(self.min_hip_angle,self.max_hip_angle,self.number_of_states)
        self.hip_interval = (self.max_hip_angle - self.min_hip_angle) / self.number_of_states
        
        # Current State
        self.s1 = int(round((number_of_states - 1)/2)) #int(round((hipjoint - min_hip_angle) / (max_hip_angle - min_hip_angle) * (number_of_states-1)))
        self.s2 = int(round(centerX/320*(self.number_of_states_gk-1)))

        # Previous states
        self.prev_state_s1 = 0
        self.prev_state_s2 = 0

        # Define rewards
        self.goal_reward = 20 # Reward for scoring goal
        self.miss_penalty = -2 # Miss the goal
        self.fall_penalty = -80 # Penalty for falling over
        self.action_penalty = -1 # Penalty for each action execution

        # Input and output for DTs
        self.x_array = np.zeros(3)
        self.diff_S1 = np.array((0))
        self.diff_S2 = np.array((0))
        self.diff_R = np.array((0))

        # Initialize Decision Trees
        self.hip_tree = tree.DecisionTreeClassifier()
        self.gk_tree = tree.DecisionTreeClassifier()
        self.reward_tree = tree.DecisionTreeClassifier()

        # Probability transitions
        self.Pm = np.zeros((self.number_of_states, self.number_of_states_gk, len(Action)))
        self.Rm = np.zeros((self.number_of_states, self.number_of_states_gk, len(Action)))

        # Learning parameters
        self.gamma = 0.001 # Discount factor

        # Q values for state and action
        self.Q = np.zeros((self.number_of_states, self.number_of_states_gk, len(Action)))

    def get_reward(self, result):
        if result == 1:
            reward = self.goal_reward
        elif result == 2:
            reward = self.miss_penalty
        elif result == 3:
            reward = self.fall_penalty
        elif result == 4:
            reward = self.action_penalty
        return reward

    def reset_state(self, centerX):
        self.s1 = 1
        self.s2 = int(round(centerX/320*(self.number_of_states_gk-1)))

    def take_action(self, centerX):

        print("state s1: {}".format(self.s1))
        print("state s2: {}".format(self.s2))
        
        actions_allowed = self.allowed_actions()

        Q_sa = self.Q[self.s1, self.s2, actions_allowed]
        print("Q: {}".format(self.Q))
        #print("Q_sa: {}".format(Q_sa))


        # Get argmax of Q value (for action selection)
        a_idx = np.argmax(Q_sa)
        action = actions_allowed[a_idx]
        #print("take action: {}".format(action))

        # Increment visits and update state set
        self.prev_state_s1 = copy.deepcopy(self.s1)
        self.prev_state_s2 = copy.deepcopy(self.s2)

        self.visits[self.s1, self.s2,action] += 1 
        print("Visit matrix after update: ", self.visits)  

        pause = False
        if Action(action) == Action.KICK:
            pause = True # pause after kicking ball
        elif Action(action) == Action.LEFT:
            self.s1 = self.s1 - 1
        elif Action(action) == Action.RIGHT:
            #print("take action right")
            self.s1 = self.s1 + 1

        self.s2 = int(round(centerX/320*(self.number_of_states_gk-1)))
        #self.s2 = 5

        return action, np.array([self.s1, self.s2])

    def allowed_actions(self):
        # Generate list of actions allowed depending on nao leg state
        actions_allowed = []
        
        if(self.s1 < self.number_of_states - 1): # No passing furthest right kick
            actions_allowed.append(2) #Action.RIGHT
        
        if (self.s1 > 0):  # No passing furthest left kick
            actions_allowed.append(1) #Action.LEFT
        
        actions_allowed.append(0) #Action.KICK
        actions_allowed = np.array(actions_allowed, dtype = int)
        print(actions_allowed)

        return actions_allowed



    def update_model(self, action, reward):
        n = 2 # 2 states
        Change = False
        
        # DT for hip input and output
        x = np.array([action, self.prev_state_s1, self.prev_state_s2])
        self.x_array = np.vstack((self.x_array, x))
        self.diff_S1 = np.append(self.diff_S1, self.prev_state_s1 - self.s1)
        self.hip_tree = self.hip_tree.fit(self.x_array, self.diff_S1)

        # DT for gk input and output
        self.diff_S2 = np.append(self.diff_S2, self.prev_state_s2 - self.s2)
        self.gk_tree = self.gk_tree.fit(self.x_array, self.diff_S2)

        self.diff_R = np.append(self.diff_R, reward)
        self.reward_tree = self.reward_tree.fit(self.x_array, self.diff_R)

        Change = True

        for sm1 in range(self.number_of_states):
            for sm2 in range(self.number_of_states_gk):
                for am in range(len(Action)):
                    self.Pm[sm1, sm2, am] = self.combine_results(sm1, sm2, am)
                    self.Rm[sm1, sm2, am] = self.get_predictions(sm1, sm2, am)
        return Change

    def compute_values(self, exp):
        
        minvisits = np.min(self.visits)

        for sm1 in range(self.number_of_states):
            for sm2 in range(self.number_of_states_gk):
                for am in range(len(Action)):
                    
                    if exp and self.visits[sm1, sm2, am] == minvisits:
                    
                        self.Q[sm1, sm2, am] = self.goal_reward
                    
                    else:
                        
                        self.Q[sm1, sm2, am] = self.Rm[sm1, sm2, am]
                        
                        for sm1_ in range(self.number_of_states):
                            for sm2_ in range(self.number_of_states_gk):
                                # if np.any(self.Sm1 == sm1_):
                                #     pass
                                # else:
                                #     self.Sm1.append(sm1_)
                                # if np.any(self.Sm2 == sm2_):
                                #     pass
                                # else:
                                #     self.Sm2.append(sm2_)
                                self.Q[sm1, sm2, am] += self.gamma * self.Pm[sm1_, sm2_, am] \
                                    * np.max(self.Q[sm1_, sm2_, :])


    def combine_results(self, sm1, sm2, am):
        # State change predictions
        deltaS1_pred = self.hip_tree.predict([[am, sm1, sm2]])
        deltaS2_pred = self.gk_tree.predict([[am, sm1, sm2]])
        state_change_pred = np.append(deltaS1_pred, deltaS2_pred)

        # Next state prediction
        state_pred = np.array((sm1, sm2)) + state_change_pred

        # Probabilities of state change
        deltaS1_prob = np.max(self.hip_tree.predict_proba([[am, sm1, sm2]]))
        deltaS2_prob = np.max(self.gk_tree.predict_proba([[am, sm1, sm2]]))
        P_deltaS = deltaS1_prob * deltaS2_prob

        # # Debug code
        # deltaS1_prob = self.s1_tree.predict_proba([[am, sm1, sm2]])
        # deltaS2_prob = self.s2_tree.predict_proba([[am, sm1, sm2]])
        # print("State pred: {}".format(state_pred))
        # print(deltaS1_prob)
        # print(deltaS2_prob)

        # What do we do with next state prediction?
        # Is the probability of the change in state the same as the prob of the next state?

        return P_deltaS

    def get_predictions(self, sm1, sm2, am):
        
        deltaR_pred = self.reward_tree.predict([[am, sm1, sm2]])
        # Should change to average of predictions
        
        return deltaR_pred.tolist()[0]

    def check_model(self):
        
        exp = np.all(self.Rm[self.prev_state_s1, self.prev_state_s2, :] < 0)
        #print("Rm: {}".format(self.Rm))
        return exp
