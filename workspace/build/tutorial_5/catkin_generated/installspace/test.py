#!/usr/bin/env python2
# July 9, 2020
# Nick Tacca

from __future__ import division
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from sklearn import tree
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

# Initial constants
NS_LEG = 10
NS_GK = 20
N_ACTIONS = NS_LEG
NUM_EPISODES = 200

class Central:

    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False  
        self.key = ""
        self.BlobX = 0
        self.BlobY = 0

        pass

    def key_cb(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.key = data.data

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Red detection -- 2 masks required
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_range1 = np.array([0,140,0])
        upper_range1 = np.array([15,255,255])
        mask1 = cv2.inRange(hsv, lower_range1, upper_range1)

        lower_range2 = np.array([170,140,0])
        upper_range2 = np.array([185,255,255])
        mask2 = cv2.inRange(hsv, lower_range2, upper_range2)

        mask = mask1 + mask2

        # Convert mask to binary image
        ret,thresh = cv2.threshold(mask,127,255,0)

        # Find contours in the binary image
        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        for c in contours:
            area = cv2.contourArea(c)
            areas.append(area)

        # Index of largest blob
        if len(areas) != 0:
            idx = np.argmax(areas)     

            # Get moment of largest blob   
            M = cv2.moments(contours[idx])

            # Calculate x,y coordinate of centroid
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            #print("Centroid of largest blob: {}, {}".format(cx, cy))

            # Show centroid in image
            cv2.circle(cv_image, (cx,cy), 5, (255,255,255), -1)

            self.BlobX = cx
            self.BlobY = cy
        
        # cv2.imshow("image window",cv_image)
        # cv2.imshow("mask window",mask)
        # cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def set_joint_angles(self, LHipRoll_angle, LAnkleRoll_angle, RKneePitch_angle, RHipRoll_angle, RHipPitch_angle, RAnklePitch_angle):

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LHipRoll")
        joint_angles_to_set.joint_angles.append(LHipRoll_angle)
        joint_angles_to_set.joint_names.append("LAnkleRoll")
        joint_angles_to_set.joint_angles.append(LAnkleRoll_angle)
        joint_angles_to_set.joint_names.append("RKneePitch")
        joint_angles_to_set.joint_angles.append(RKneePitch_angle)
        joint_angles_to_set.joint_names.append("RHipRoll")
        joint_angles_to_set.joint_angles.append(RHipRoll_angle)
        joint_angles_to_set.joint_names.append("RHipPitch")
        joint_angles_to_set.joint_angles.append(RHipPitch_angle)
        joint_angles_to_set.joint_names.append("RAnklePitch")
        joint_angles_to_set.joint_angles.append(RAnklePitch_angle)
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.8 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
    
    def determine_state(self, env):
        # Get leg state -- 10 possible states
        min_state = env.RHipRoll_actions[NS_LEG-1]
        max_state = env.RHipRoll_actions[0]
        s1 = round((self.joint_angles[15] - min_state)/(max_state - min_state) * (NS_LEG-1))

        # Get goalie position (only need to use x-position) -- 10 possible states
        s2 = round(self.BlobX/320 * (NS_GK-1))

        return np.array((s1, s2))

    def central_execute(self, nao, env):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        self.set_stiffness(True)
        rospy.sleep(1.0)

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        train = yes_or_no("Would you like to train a new agent?")

        if train:
            while True:

                # # Finding home position (balance on one leg)
                # while True:
                #     if self.key == "H1":
                #         break

                #     # Stand on one leg
                #     self.set_joint_angles(-0.379, 0.379, 0, -0.7, 0, 0)
                #     rospy.sleep(0.1)
                
                # while True:
                #     if self.key == "H2":
                #         break

                #     self.set_joint_angles(-0.379, 0.379, 0.3, -0.7, 0.2, -0.4)
                #     rospy.sleep(0.1)
		
		        # Need to adjust nao position slightly at this point (0.06m forward, 0.03m left)

                while True:
                    if self.key == "C":
                        break

                    self.set_joint_angles(-0.3, 0.3, 0.4, -0.6, 0.2, -0.4)
                    rospy.sleep(0.1)

                # Main RL-DT loop
                cumm_reward = []
                total_reward = 0
                for episode in range(NUM_EPISODES):

                    # Get action from optimal policy
                    action, pause = env.get_action(nao.state)

                    # Take action
                    RHipRoll, RHipPitch = nao.action_execution(env, action)

                    if pause:
                        self.set_joint_angles(-0.3, 0.3, 0.4, RHipRoll, RHipPitch, -0.3) # heel in way
                        while True:
                            self.set_stiffness(True)
                            if self.key == "G" or self.key == "M" or self.key == "F":
                                break
                    else:
                        self.set_joint_angles(-0.3, 0.3, 0.4, RHipRoll, RHipPitch, -0.4)
                    
                    rospy.sleep(0.1)

                    # Determine next state
                    state_ = self.determine_state(env)

                    # Determine reward from action taken
                    reward = env.get_reward(self.key)
                    total_reward += reward
                    cumm_reward.append(total_reward)

                    # Increment visits and update state set
                    env.visits[nao.state[0], nao.state[1], action] += 1
                    # if np.any(env.Sm1 == state_[0]):
                    #     pass
                    # else:
                    #     env.Sm1.append(state_[0])

                    # if np.any(env.Sm2 == state_[1]):
                    #     pass
                    # else:
                    #     env.Sm2.append(state_[1])                   
                    
                    # Update model
                    CH = env.update_model(nao.state, action, reward, state_)
                    exp = env.check_model(nao.state)

                    if CH:
                        env.compute_values(exp)

                    if episode+1 % 10 == 0:
                        print("Episode: {}/{}, Cummulative Reward: {}".format(episode+1, NUM_EPISODES, total_reward))

                    # Update state
                    nao.state = state_

                    if pause:
                        while True:
                            if self.key == "C":
                                break

                            self.set_joint_angles(-0.3, 0.3, 0.4, -0.6, 0.2, -0.4)
                            rospy.sleep(0.1)

                        # Reset
                        nao.state = np.zeros(2)
                        nao.a1 = 8
                        nao.a2 = 0
                        self.key = ""
                    elif self.key == "Q":
                        break
                cumm_reward = np.array(cumm_reward)
                show_plot(cumm_reward)
                save_environment(env)

        else:
            # Load environment (contains Q & decision trees)
            env = load_environment()

            while True:

                while True:
                    if self.key == "C":
                        break

                    self.set_joint_angles(-0.3, 0.3, 0.4, -0.6, 0.2, -0.4)
                    rospy.sleep(0.1)

                rospy.sleep(1.0)

                # Get current state
                nao.state = self.determine_state(env)

                # Get action based on learned model
                action, pause = env.get_action(nao.state)

                # Take action
                RHipRoll, RHipPitch = nao.action_execution(env, action)

                if pause:
                    self.set_joint_angles(-0.3, 0.3, 0.4, RHipRoll, RHipPitch, -0.25) # heel in way
                    while True:
                        if self.key == "G" or self.key == "M" or self.key == "F":
                            break
                    
                    while True:
                        if self.key == "C":
                            break

                        self.set_joint_angles(-0.3, 0.3, 0.4, -0.6, 0.2, -0.4)
                        rospy.sleep(0.1)
                    
                        # Reset
                    nao.state = np.zeros(2)
                    nao.a1 = 0
                    nao.a2 = 0
                    self.key = ""
                elif self.key == "Q":
                    break
                else:
                    self.set_joint_angles(-0.3, 0.3, 0.4, RHipRoll, RHipPitch, -0.4)

                rospy.sleep(0.1)

        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rospy.spin()

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            rate.sleep()

    # each Subscriber is handled in its own thread
    #rospy.spin()

class Agent:

    def __init__(self, state, a1, a2):
        self.state = state
        self.a1 = a1 # RHipRoll action idx
        self.a2 = a2 # RHipPitch action idx
    
    def action_execution(self, env, action):
        self.a1 += env.action_translations[action][0]
        self.a2 += env.action_translations[action][1]
        # print(env.action_translations[action][1])
        
        # Restrict boundaries
        if self.a1 < 0:
            self.a1 = 0
        elif self.a1 > N_ACTIONS - 1:
            self.a1 = N_ACTIONS - 1

        # # Should not matter because pause & reset
        # if self.a2 < 0:
        #     self.a2 = 0
        # elif self.a1 > 1:
        #     self.a2 = 1
        
        # Get action for NAO execution
        RHipRoll = env.RHipRoll_actions[self.a1]
        RHipPitch = env.RHipPitch_actions[self.a2]

        return RHipRoll, RHipPitch

class Environment:

    def __init__(self, Ns_leg, Ns_gk):
        # Total number of states
        self.Ns_leg = Ns_leg
        self.Ns_gk = Ns_gk

        # State sets
        # self.Sm1 = []
        # self.Sm2 = []
        self.Sm1 = np.zeros(NS_LEG)
        self.Sm2 = np.zeros(NS_GK)

        # Define quantized action space
        self.RHipRoll_actions = np.linspace(-0.5, -0.8, N_ACTIONS) # Number hip roll actions
        self.RHipPitch_actions = np.array((0.2, -1.4))

        # Define actions
        self.action_dict = {"left": 0, "right": 1, "kick": 2}
        self.action_list = [0, 1, 2]
        self.action_translations = [(-1, 0), (1, 0), (0, 1)] # action translations within quantized action space

        # Visit count
        self.visits = np.zeros((self.Ns_leg, 
                           self.Ns_gk,
                           len(self.action_list)))
        
        # Prob transitions
        self.Pm = np.zeros((self.Ns_leg, 
                           self.Ns_gk,
                           len(self.action_list)))
        self.Rm = np.zeros((self.Ns_leg, 
                           self.Ns_gk,
                           len(self.action_list)))
        
        # Initialize Decision Trees
        self.s1_tree = tree.DecisionTreeClassifier()
        self.s2_tree = tree.DecisionTreeClassifier()
        self.R_tree = tree.DecisionTreeClassifier()

        # Initialize input (always same) and output vectors for trees
        self.x_array = np.zeros(3)
        self.deltaS1 = np.array((0))
        self.deltaS2 = np.array((0))
        self.deltaR = np.array((0))

        # Define rewards
        self.goal_reward = 20 # Reward for scoring goal
        self.miss_penalty = -2 # Miss the goal
        self.fall_penalty = -20 # Penalty for falling over
        self.action_penalty = -1 # Penalty for each action execution
        
        # Learning parameters
        self.gamma = 0.001 # Discount factor
        
        # Q values for state and action
        self.Q = np.zeros((self.Ns_leg, 
                           self.Ns_gk,
                           len(self.action_list)))

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
    
    def get_reward(self, key):
        if key == "G":
            reward = self.goal_reward
        elif key == "M":
            reward = self.miss_penalty
        elif key == "F":
            reward = self.fall_penalty
        else:
            reward = self.action_penalty
        return reward
                 
    def get_action(self, state):
        actions_allowed = self.allowed_actions(state[0])
        Q_sa = self.Q[state[0], state[1], actions_allowed]

        # Get argmax of Q value (for action selection)
        a_idx = np.argmax(Q_sa)
        action = actions_allowed[a_idx]
        # print(state)
        print("Q: {}".format(Q_sa))
        print("Action: {}".format(action))

        pause = False
        if action == 2:
            pause = True # pause after kicking ball

        return action, pause
    
    def check_model(self, state):
        exp = np.all(self.Rm[state[0], state[1], :] < 0)
        return exp
    
    def add_experience(self, n, state, action, delta):
        if n == 0:
            # x (input)
            x = np.append(np.array(action), state)
            self.x_array = np.vstack((self.x_array, x))

            # y (output)
            self.deltaS1 = np.append(self.deltaS1, delta)
            # print("Input: {}".format(self.x_array))
            # print("True Output: {}".format(self.deltaS1))
            self.s1_tree = self.s1_tree.fit(self.x_array, self.deltaS1)
        elif n == 1:
            self.deltaS2 = np.append(self.deltaS2, delta)
            # print("Input: {}".format(self.x_array))
            # print("True Output: {}".format(self.deltaS2))
            self.s2_tree = self.s2_tree.fit(self.x_array, self.deltaS2)
        elif n == 3:
            self.deltaR = np.append(self.deltaR, delta)
            # print("Input: {}".format(self.x_array))
            # print("True Output: {}".format(self.deltaR))
            self.R_tree = self.R_tree.fit(self.x_array, self.deltaR)
        CH = True
        return CH
    
    def combine_results(self, sm1, sm2, am):
        # State change predictions
        deltaS1_pred = self.s1_tree.predict([[am, sm1, sm2]])
        deltaS2_pred = self.s2_tree.predict([[am, sm1, sm2]])
        state_change_pred = np.append(deltaS1_pred, deltaS2_pred)

        # Next state prediction
        state_pred = np.array((sm1, sm2)) + state_change_pred

        # Probabilities of state change
        deltaS1_prob = np.max(self.s1_tree.predict_proba([[am, sm1, sm2]]))
        deltaS2_prob = np.max(self.s2_tree.predict_proba([[am, sm1, sm2]]))
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
        deltaR_pred = self.R_tree.predict([[am, sm1, sm2]])
        # Should change to average of predictions
        return deltaR_pred.tolist()[0]
    
    def update_model(self, state, action, reward, state_):
        n = len(state)
        CH = False
        xi = np.zeros(n)
        for i in range(n):
            xi[i] = state[i] - state_[i]
            CH = self.add_experience(i, state, action, xi[i])

        CH = self.add_experience(n+1, state, action, reward)

        for sm1 in range(len(self.Sm1)):
            for sm2 in range(len(self.Sm2)):
                for am in range(len(self.action_list)):
                    self.Pm[sm1, sm2, am] = self.combine_results(sm1, sm2, am)
                    self.Rm[sm1, sm2, am] = self.get_predictions(sm1, sm2, am)
        return CH
    
    def compute_values(self, exp):
        minvisits = np.min(self.visits)
        for sm1 in range(len(self.Sm1)):
            for sm2 in range(len(self.Sm2)):
                for am in range(len(self.action_list)):
                    if exp and self.visits[sm1, sm2, am] == minvisits:
                        self.Q[sm1, sm2, am] = self.goal_reward
                    else:
                        self.Q[sm1, sm2, am] = self.Rm[sm1, sm2, am]
                        for sm1_ in range(self.Ns_leg):
                            for sm2_ in range(self.Ns_gk):
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
                                # print(self.Q[sm1, sm2, am])
                                # print(self.Pm[sm1_, sm2_, am])
                                # print(self.Q[sm1_, sm2_, :])
                                # print(np.max(self.Q[sm1_, sm2_, :]))
                                # print(sm1, sm1_)
                                # print(self.Pm[sm1_])

                                """ Issue here is that prob of next state for all the states = 1 -- messed up coming out of DT
                                 Should be mostly 0 or close to zero (eg. state[0] 1 going to state[0] 6 should be 0 for all actions and state[1])
                                 """

def yes_or_no(question):
    while "The answer is invalid":
        reply = str(raw_input(question+' (y/n): ')).lower().strip()
        if reply[:1] == 'y':
            return True
        if reply[:1] == 'n':
            return False

def show_plot(cumm_reward):
    plt.plot(cumm_reward)
    plt.xlabel("Episodes")
    plt.ylabel("Cummulative Reward")
    plt.grid()
    dirname = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(dirname, "Cummulative_Reward-{}Eps.png".format(NUM_EPISODES)), dpi=500)
    plt.show()

def save_environment(env):
    dirname = os.path.dirname(os.path.abspath(__file__))

    env_fn = os.path.join(dirname, "environment.obj")
    pickle.dump(env, open(env_fn, 'wb'))

def load_environment():
    dirname = os.path.dirname(os.path.abspath(__file__))

    env_fn = os.path.join(dirname, "environment.obj")
    env = pickle.load(open(env_fn, 'rb'))
    return env
    

def main():
    env = Environment(Ns_leg=NS_LEG, Ns_gk=NS_GK)
    nao = Agent(state=np.zeros(2), a1=8, a2=0)

    # Instantiate central class and start loop
    central_instance = Central()
    central_instance.central_execute(nao, env)

if __name__=='__main__':
    main()