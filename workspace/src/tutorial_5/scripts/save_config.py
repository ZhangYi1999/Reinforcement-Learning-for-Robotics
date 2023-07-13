#!/usr/bin/env python
import sys
import rospy
import yaml
from math import pi
from sensor_msgs.msg import JointState

from naoqi import ALProxy
from naoqi_bridge_msgs.msg import HeadTouch,JointAnglesWithSpeed


class Central:
    def __init__(self):
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

        self.pressed = False


    def touch_cb(self,data):       
        if data.button == 1: # press the head tactile button 1
            self.pressed = True

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        joint_names = data.name 
        joint_angles = data.position

        if(self.pressed):
            self.pressed = False
            value={}
            for i in range(len(joint_names)):
                value[joint_names[i]] = joint_angles[i]
            
            with open("/home/nao/bilhr23ss/workspace/src/tutorial_5/config/pose.yaml","w") as file:
                
                document = yaml.dump(value,file)

    def set_joint_angles(self,joint_name,joint_angle):
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append(joint_name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(joint_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)


    def central_execute(self,):
        # print(sys.path[0])
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("/tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/joint_states",JointState,self.joints_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)
        # self.postureProxy.goToPosture("Stand", 0.5)

        # with open("/home/nao/bilhr23ss/workspace/src/tutorial_5/config/ready.yaml","r") as file:
        #     joint_pair = yaml.load(file)
            
        #     for joint_name,joint_value in joint_pair.items():
        #         fractionMaxSpeed  = 0.1
        #         self.motion.setAngles([joint_name],[float(joint_value)],fractionMaxSpeed)

        rospy.spin()
        # self.postureProxy.goToPosture("Crouch", 0.5)

if __name__=='__main__':
    # Instantiate central class and start loop
    central_instance = Central()
    central_instance.central_execute()