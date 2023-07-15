#!/usr/bin/env python2
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
import sys
import numpy as np
from enum import Enum
from naoqi import ALProxy
import os
import pickle
import matplotlib.pyplot as plt

from reinforce_learning import Action, Reinforce_Learning_Kick

# Hip range -9.1 to -20.7
from sklearn import tree

Number_of_episodes = 100
number_of_states_gk = 10
number_of_states = 11
max_hip_angle = -4.1
min_hip_angle = -35.7

#init_hip_radia = min_hip_angle + (max_hip_angle)

class Central:
    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
    
        self.pressed = False
        self.pressed_button = 0
        self.centerX = 0
        self.centerY = 0
        self.button = 0
        self.flagTraining = False

        self.set_stiffness(True)

        
        try:
            self.motion = ALProxy("ALMotion", "10.152.246.59", 9559)
        except Exception as e:
            print("Could not create proxy to ALMotion")
            print("Error was: ",e)
            sys.exit(1)

        try:
            self.postureProxy = ALProxy("ALRobotPosture", "10.152.246.59", 9559)
        except Exception as e:
            print("Could not create proxy to ALMotion")
            print("Error was: ",e)
            sys.exit(1)


       


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity
        pass

    def touch_cb(self,data): 
        print("touch_cb")
        if data.button == 1: # press the head tactile button 1
            if data.state == 0: # move arm when release the button
                self.pressed_button = 1
                self.pressed = True
                print("Value of pressed button after reset: ", self.pressed_button)
                print("Value of pressed button after reset: ", self.pressed)
        elif data.button == 2: # press the head tactile button 2
            if data.state == 0: # move arm when release the button
                self.pressed_button = 2
                self.pressed = True
                print("Value of pressed button after reset: ", self.pressed_button)
                print("Value of pressed button after reset: ", self.pressed)
        elif data.button == 3: # press the head tactile button 3
            if data.state == 0: # move arm when release the button
                self.pressed_button = 3
                self.pressed = True
                print("Value of pressed button after reset: ", self.pressed_button)
                print("Value of pressed button after reset: ", self.pressed)
        # self.pressed_button = data.button
        
        # if data.state != 0:
        #     self.pressed = True 

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
        self.image = cv_image
        # # Convert mask to binary image
        ret,thresh = cv2.threshold(mask,127,255,0)

        # # Find contours in the binary image
        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour,False)
            
            if(area > 120 and area < 450):
                # Get moment of largest blob   
                M = cv2.moments(contour)

                # Calculate x,y coordinate of centroid
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # normalize the data
                height, width, _ = cv_image.shape
                self.centerX = 1.0*cx/width
                self.centerY = 1.0*cy/height

                res = cv2.bitwise_and(cv_image,cv_image, mask= mask)
                
                # put text and highlight the center
                cv2.circle(res, (cx, cy), 5, (255, 0, 0), -1)
        
                self.image = res
        # Show keypoints
        cv2.namedWindow("image window")        # Create a named window
        cv2.moveWindow("image window", 200, 200) 
        cv2.imshow("image window", self.image)
        cv2.waitKey(3)


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
    
    def set_head_pos(self):
        names = ["HeadPitch"]
        angles = [(16.2)*pi/180]
        fractionMaxSpeed  = 0.1
        self.motion.setAngles(names,angles,fractionMaxSpeed)
        rospy.sleep(2)

    def save_env(self,env):
        dirname = os.path.dirname(os.path.abspath(__file__))
        env_instance = os.path.join(dirname,"environment.obj")

        pickle.dump(env, open(env_instance, "wb"))

    def load_env(self):
        dirname = os.path.dirname(os.path.abspath(__file__))
        env_instance = os.path.join(dirname,"environment.obj")

        env = pickle.load(open(env_instance, "rb"))
        return env

    def plot_cummulative_reward(self, cumm_reward):
        
        plt.plot(cumm_reward)
        plt.xlabel("Episodes")
        plt.ylabel("Cummulative reward")

        dirname = os.path.dirname(os.path.abspath(__file__))
        plt.savefig(os.path.join(dirname, "Plot_of_cummulative_reward_200ep.png"))
        plt.show()



    def set_ready_pos(self):
        names = ["RHipRoll", "RAnkleRoll","LHipRoll","LHipPitch","LAnkleRoll",  "LShoulderRoll", 'RElbowRoll', "RElbowYaw"]
        angles = [(max_hip_angle-min_hip_angle)/2, 0.05,-0.16, 0.05, 0.3,  0.50, 1.17, 0.64]
        fractionMaxSpeed  = 0.1
        self.motion.setAngles(names,angles,fractionMaxSpeed)
        rospy.sleep(2)
        names = ["RAnklePitch","RHipPitch", "RKneePitch",'RAnkleRoll','HeadYaw',"HeadPitch"]
        angles = [-0.17, -0.31, 0.57, 0.18, -0.16, (16.2)*pi/180]
        fractionMaxSpeed  = 0.1
        self.motion.setAngles(names,angles,fractionMaxSpeed)
        rospy.sleep(2)

    def kick(self):
        names = ['RAnklePitch','RKneePitch']
        angles = [0.26, 0.09]
        fractionMaxSpeed = 1.0
        self.motion.setAngles(names, angles, fractionMaxSpeed)
        rospy.sleep(0.5)
        names = ['RAnklePitch','RKneePitch']
        angles = [-0.17, 0.57]
        fractionMaxSpeed = 0.2
        self.motion.setAngles(names, angles, fractionMaxSpeed)

        rospy.sleep(1)

        #self.postureProxy.goToPosture("Stand", 0.1)   

    def move(self, value): # value is degree
        names = ["RHipRoll"]
        angles = [self.joint_angles[15] + value*pi/180]
        fractionMaxSpeed = 0.1
        self.motion.setAngles(names, angles, fractionMaxSpeed)

        rospy.sleep(2)

    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)
        self.postureProxy.goToPosture("Stand", 0.2)
        self.set_head_pos()
        rospy.sleep(1)

        self.set_ready_pos()
        
        while True:
            
            train = raw_input("Would you like to train? y/n: ")
            
            if train == "y":
             
                env = Reinforce_Learning_Kick(number_of_states, min_hip_angle, max_hip_angle, number_of_states_gk, self.centerX, self.joint_angles[15])
                self.flagTraining = True
                break
            
            elif train == "n":
             
                env = self.load_env()
                self.flagTraining = False
                break
            
            else:
                continue
        
        

        # # Main RL-DT loop
        cumm_reward = []
        total_reward = 0

        if self.flagTraining:

            for episode in range(Number_of_episodes):
                
                # Get action from optimal policy
                action, new_state = env.take_action(self.centerX)
                            

                if Action(action) == Action.KICK:
                    self.kick()
                    print("kick")
                    print("self.pressed value before while ", self.pressed)
                    
                    # while(not self.pressed):
                    #     print("self.pressed value in while ", self.pressed)
                    #     rospy.sleep(0.5)
                    while True:
                        self.button = raw_input("Type in the reward: ")
                        if self.button == "g":
                            reward = env.get_reward(1)
                            break

                        elif self.button == "m":
                            reward = env.get_reward(2)
                            break
                        
                        elif self.button == "f":
                            reward = env.get_reward(3)
                            break
                        
                        else:
                            reward = env.get_reward(4)
                            break

                    self.pressed = False
                    
                    print("Value of pressed button after reset: ", self.pressed_button)
                    print("Value of pressed button after reset: ", self.pressed)
                    #reward = env.get_reward(self.pressed_button)
                    print("Reward after pressing button: ", reward)
                    #self.pressed_button = 0

                    self.set_ready_pos()
                
                elif Action(action) == Action.RIGHT:
                    self.move(-env.hip_interval)
                    print("go right")
                    reward = env.get_reward(4)
                    
                else:
                    print("go left")
                    self.move(env.hip_interval)
                    reward = env.get_reward(4)

                # Next state
            #     # Take action
                
            #     rospy.sleep(0.1)

            #     # Determine next state
            #     state_ = self.determine_state(env)

            #     # Determine reward from action taken
            #     reward = env.get_reward(self.key)
                total_reward += reward
                cumm_reward.append(total_reward)

                
            # Update model
                Change = env.update_model(action, reward)
                explore_or_exploit = env.check_model()
                #print("Explore/exploit ", explore_or_exploit)

                if Change:
                    env.compute_values(explore_or_exploit)
                
                if Action(action) == Action.KICK:
                    env.reset_state(self.centerX)

                if episode+1 % 10 == 0:
                    print("Episode: {}/{}, Cummulative Reward: {}".format(episode+1, Number_of_episodes, total_reward))


            cumm_reward = np.array(cumm_reward)
            self.plot_cummulative_reward(cumm_reward)
            self.save_env(env)
        
        else:

            while True:
        # Get action from optimal policy
                action, new_state = env.take_action(self.centerX)
                            
                if Action(action) == Action.KICK:
                    self.kick()
                    #self.set_ready_pos()
                        
                    self.button = raw_input("Enter to restart")
                    env.s1 = round((self.joint_angles[15] - min_hip_angle) / (max_hip_angle - min_hip_angle) * (number_of_states-1))
            
                elif Action(action) == Action.RIGHT:
                    self.move(-env.hip_interval)
                        
                else:
                    self.move(env.hip_interval)

                # Finish executing actions                
                if self.button == "n":
                    break
            
        rospy.sleep(2)


        names = ["RKneePitch"]
        angles = [0]
        fractionMaxSpeed  = 0.9
        self.motion.setAngles(names,angles,fractionMaxSpeed)   

        rate = rospy.Rate(100)
        
        
        
        while not rospy.is_shutdown():
            
            rate.sleep()

        rospy.spin()
        self.postureProxy.goToPosture("Crouch", 0.5)

        #self.set_stiffness(False)




if __name__=='__main__':
    # instantiate class and start loop function
    # env = Environment(Ns_leg=Number_of_states_leg, Ns_gk=Number_of_states_gk)
    # nao = Agent(state=np.zeros(2), a1=8, a2=0)

    # Instantiate central class and start loop
    central_instance = Central()
    central_instance.central_execute()