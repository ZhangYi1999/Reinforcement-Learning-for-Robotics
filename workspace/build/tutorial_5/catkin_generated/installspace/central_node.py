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
import numpy as np
import sys

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
            if data.state == 0: # save data when release the button
                pass

        if data.button == 2: # press the head tactile button 2
            if data.state == 0: # write data into file when release the button
                pass 

        if data.button == 3: # press the head tactile button 3
            if data.state == 0: # turn off stiffness when release the button
                names = "Body"
                stiffnessLists = 0.0
                self.motion.setStiffnesses(names,stiffnessLists)


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
    
        
        # lean the body a little bit left
        names = ["LHipRoll","LAnkleRoll","RHipRoll","RAnkleRoll","LShoulderRoll"]
        angles = [-0.3,0.3,-0.3,0.3,0.7]
        fractionMaxSpeed  = 0.1
        self.motion.setAngles(names,angles,fractionMaxSpeed)

        rospy.sleep(2)

        # move the right leg out and the left arm out
        names = ["RHipRoll","RAnkleRoll","RElbowRoll","RElbowYaw"]
        angles = [-0.6,0.6,1.0,1.0]
        fractionMaxSpeed  = 0.1
        self.motion.setAngles(names,angles,fractionMaxSpeed)

        rospy.sleep(2)

        # bend the right leg
        names = ["RKneePitch","RAnklePitch"]
        angles = [0.5,-0.5]
        fractionMaxSpeed  = 0.05
        self.motion.setAngles(names,angles,fractionMaxSpeed)

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

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()