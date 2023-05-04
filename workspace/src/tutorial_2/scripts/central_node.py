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

        self.set_stiffness(True) 

         

        pass


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity
        # print('elbow_angle:',self.joint_angles[4])
        # print('target_angle:',self.targetAngle)

        # print('joint_names:',self.joint_names)
        # print('joint_angles:',self.joint_angles)


        pass

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    def touch_cb(self,data):
        # rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        # print("touch button: "+str(data.button)+" state: "+str(data.state))
        
        if data.button == 1:
            if data.state == 0: # move arm when release the button
                self.wavingFlag = False
                self.repeatFlag = False
                self.set_home_position()
        elif data.button == 2:
            if data.state == 0: # move arm when release the button
                self.wavingFlag = True 
                self.set_repetitive_motion()
        elif data.button == 3:
            if data.state == 0: # move arm when release the button
                self.repeatFlag = True
        

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

        print("center position:["+str(cX)+","+str(cY)+"]")

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
        joint_angles_to_set.joint_names.append(joint_name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(head_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
        
    def set_home_position(self):
        self.set_joint_angles("LShoulderPitch",pi/2)
        self.set_joint_angles("LShoulderRoll", 0.5)
        self.set_joint_angles("LElbowYaw", 0.0)
        self.set_joint_angles("LElbowRoll", -0.0349) 
        self.set_joint_angles("LWristYaw", 0.0)

        self.set_joint_angles("RShoulderPitch",pi/2)
        self.set_joint_angles("RShoulderRoll", -0.5)
        self.set_joint_angles("RElbowYaw", 0.0)
        self.set_joint_angles("RElbowRoll", 0.0349) 
        self.set_joint_angles("RWristYaw", 0.0)

    def set_repetitive_motion(self):
        self.set_joint_angles("LShoulderPitch", 0.0)
        self.set_joint_angles("LShoulderRoll", 0.5)
        self.set_joint_angles("LElbowYaw", self.targetAngle)
        self.set_joint_angles("LElbowRoll", -1.5446)
        self.set_joint_angles("LWristYaw", pi/2)

    def set_mirror_motion(self):
        self.set_joint_angles("RShoulderPitch", self.joint_angles[2])
        self.set_joint_angles("RShoulderRoll", -self.joint_angles[3])
        self.set_joint_angles("RElbowYaw", -self.joint_angles[4])
        self.set_joint_angles("RElbowRoll", -self.joint_angles[5])
        self.set_joint_angles("RWristYaw", -self.joint_angles[6])




    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)


        # test sequence to demonstrate setting joint angles
        # self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        # rospy.sleep(1.0)
        # self.set_joint_angles(0.5)
        # rospy.sleep(3.0)
        # self.set_joint_angles(0.0)
        # rospy.sleep(3.0)
        # self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        begin = rospy.get_rostime()

        angle = pi/8

        while not rospy.is_shutdown():
            if(self.repeatFlag):
                self.set_mirror_motion()

            if (self.wavingFlag is True) and (rospy.get_rostime().secs - begin.secs >= 3.0):
                begin = rospy.get_rostime()
                angle = -angle
                self.targetAngle = angle-pi/2
                self.set_joint_angles("LElbowYaw", self.targetAngle)


            rate.sleep()
        rospy.spin()
            
        
        self.set_stiffness(False)

    # rospy.spin() just blocks the code from exiting, if you need to do any periodic tasks use the above loop
    # each Subscriber is handled in its own thread
       

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
