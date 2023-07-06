#!/usr/bin/env python
#NAMES: Menzel Nicholas, Pocasangre Ernesto, Gacic Dajana, Zhang Yi, Causevic Azur
import rospy
from math import pi
from rospy import rostime
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys
from sklearn import tree


from naoqi import ALProxy

#RShoulderPitch_min = -0.6810
#RShoulderPitch_max = 0.0291
#according to the table:
RShoulderPitch_min = -2.085
RShoulderPitch_max = 2.085

RShoulderRoll_min = -0.314
RShoulderRoll_max = 1.326

#RShoulderRoll_min = -0.9176
#RShoulderRoll_max = 0.0250

Number_of_states_leg = 5
Number_of_states_gk = 5
Number_of_actions = Number_of_states_leg
Number_of_episodes = 100

class Central:


    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False 
        self.wavingFlag = False
        self.repeatFlag = False
        self.targetAngle = -pi/2

        self.centerX = 0
        self.centerY = 0
        self.RShoulderPitch = 0
        self.RShoulderRoll = 0
        self.training_data = []
        self.target = [0,0]
        self.weights = []
        self.output1 =[0]
        self.output2 = [0]
        self.set_stiffness(True)

        
        try:
            self.motion = ALProxy("ALMotion", "10.152.246.59", 9559)
        except Exception,e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            sys.exit(1)

        try:
            self.postureProxy = ALProxy("ALRobotPosture", "10.152.246.59", 9559)
        except Exception, e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            


        pass


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        self.RShoulderPitch = (self.joint_angles[20] - RShoulderPitch_min) / (RShoulderPitch_max - RShoulderPitch_min)
        self.RShoulderRoll = (self.joint_angles[21] - RShoulderRoll_min) / (RShoulderRoll_max - RShoulderRoll_min)
        
        pass

    def touch_cb(self,data):       
        if data.button == 1: # press the head tactile button 1
            pass

        if data.button == 2: # press the head tactile button 2
            pass

        if data.button == 3: # press the head tactile button 3
            pass
            # if data.state == 0: # turn off stiffness when release the button
            #     names = "Body"
            #     stiffnessLists = 0.0
            #     self.motion.setStiffnesses(names,stiffnessLists)


    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_red = np.array([-10,50,50])
        upper_red = np.array([10,255,255])

        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        M = cv2.moments(mask)
 
        # calculate x,y coordinate of center
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        # normalize the data
        height, width, _ = cv_image.shape
        self.centerX = 1.0*cX/width
        self.centerY = 1.0*cY/height

        res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
        
        # put text and highlight the center
        cv2.circle(res, (cX, cY), 5, (255, 0, 0), -1)

        
        cv2.imshow("image window",res)
        cv2.namedWindow("image window")        # Create a named window
        cv2.moveWindow("image window", 200, 200) 
        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException as e:
            rospy.logerr(e)

    def set_joint_angles(self,joint_name,head_angle):
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names=joint_name # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(head_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
        
    def set_home_position(self):
        # set the head and elbow in constant positions
        names = ["HeadYaw","HeadPitch","RElbowYaw","RElbowRoll","RWristYaw","RShoulderPitch","RShoulderRoll"]
        angles = [-0.8,0.0,0.0,0.0,pi,0.0,0.0]
        fractionMaxSpeed  = 0.2
        self.motion.setAngles(names,angles,fractionMaxSpeed)

        
    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)
        self.postureProxy.goToPosture("Stand", 0.5)
    
        #HipPitch (27- -88), KneePitch(121 - -5), AnklePitch(53 - -68) -> forward-backward movemenet
        #HipRoll (-21 - 45), AnkleRoll(-23 - 44) -> left-right

        # Getting ready to kick
        names = ["LHipRoll","LAnkleRoll","RKneePitch","RHipRoll","RHipPitch","RAnklePitch", 'RAnkleRoll']
        angles = [-0.3, 0.3, 0.4, -0.6, 0.2, -0.4, 0.6]
        fractionMaxSpeed  = 0.2
        self.motion.setAngles(names,angles,fractionMaxSpeed)

        rospy.sleep(3)

        # Kick!
        names = ['RHipPitch', 'RAnklePitch','RKneePitch']
        angles = [-0.2, 0.2, 0.0]
        fractionMaxSpeed = 0.3
        self.motion.setAngles(names, angles, fractionMaxSpeed)

        #State variables -> RHipRoll -> move left-right to precisely hit the ball
        # RHipPitch -> kicking the ball


        # lean the body a little bit left
        # names = ["LHipRoll","LAnkleRoll","RHipRoll","RAnkleRoll","LShoulderRoll"]
        # angles = [-0.3,0.3,-0.3,0.3,0.7]
        # fractionMaxSpeed  = 0.1
        # self.motion.setAngles(names,angles,fractionMaxSpeed)

        # move the right leg out and the left arm out
        # names = ["RHipRoll","RAnkleRoll","RElbowRoll","RElbowYaw"]
        # angles = [-0.6,0.6,1.0,1.0]
        # fractionMaxSpeed  = 0.1
        # self.motion.setAngles(names,angles,fractionMaxSpeed)

        # # bend the right leg
        # names = ["RKneePitch","RAnklePitch"]
        # angles = [0.5,-0.5]
        # fractionMaxSpeed  = 0.05
        # self.motion.setAngles(names,angles,fractionMaxSpeed)

        # # bend the right knee 
        # names = ["LHipRoll","LAnkleRoll","RKneePitch","RAnklePitch","RHipRoll","RAnkleRoll"]
        # angles = [-0.10,0.2,1.5,-0.8, -0.3, 0.3]
        # fractionMaxSpeed  = 0.1
        # self.motion.setAngles(names,angles,fractionMaxSpeed)   

        rospy.sleep(2)
        #kick 

        # names = ["RKneePitch"]
        # angles = [0]
        # fractionMaxSpeed  = 0.9
        # self.motion.setAngles(names,angles,fractionMaxSpeed)   

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            rate.sleep()

        rospy.spin()
        self.postureProxy.goToPosture("Crouch", 0.5)

        #self.set_stiffness(False)

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
        elif self.a1 > Number_of_actions - 1:
            self.a1 = Number_of_actions - 1

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
        self.Sm1 = np.zeros(Number_of_states_leg)
        self.Sm2 = np.zeros(Number_of_states_gk)

        # Define quantized action space
        self.RHipRoll_actions = np.linspace(-0.5, -0.8, Number_of_actions) # Number hip roll actions
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
    
    # Keyboard input needed!
    def get_reward(self, rew_key):
        
        if rew_key == 1:
            reward = self.goal_reward
        
        elif rew_key == 2:
            reward = self.miss_penalty
        
        elif rew_key == 3:
            reward = self.fall_penalty
        
        elif rew_key == 4:
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



if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()