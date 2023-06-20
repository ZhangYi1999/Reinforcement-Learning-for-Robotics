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
        self.load_weights()
        self.set_stiffness(True)

        
        try:
            self.motion = ALProxy("ALMotion", "10.152.246.115", 9559)
        except Exception,e:
            print "Could not create proxy to ALMotion"
            print "Error was: ",e
            sys.exit(1)


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
                # saving data
                # self.training_data.append([self.centerX,self.centerY,self.RShoulderPitch,self.RShoulderRoll])
                # print('saved '+str(len(self.training_data))+' :'+str([self.centerX,self.centerY,self.RShoulderPitch,self.RShoulderRoll]))
                pass

        if data.button == 2: # press the head tactile button 2
            if data.state == 0: # write data into file when release the button
                # This code was used for data collection
                # print("saving data")
                # with open('/home/nao/bilhr23ss/workspace/src/tutorial_4/data/data_150_samples.txt', 'w') as f:
                #    for line in self.training_data:
                #        for value in line:
                #            f.write(str(value)+' ')
                #        f.write('\n')
                # f.close()
                # print("data saved")
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
        joint_angles_to_set.joint_names.append(joint_name) # each joint has a specific name, look into the joint_state topic or google
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

    def update(self):

        print("Updated position!")
        self.set_joint_angles("RShoulderPitch", self.target[0])
        self.set_joint_angles("RShoulderRoll", self.target[1])
        
    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        
        names  = 'Body'
        # If only one parameter is received, this is applied to all joints
        stiffnesses  = 1.0
        self.motion.setStiffnesses(names, stiffnesses)

        name = ["RShoulderPitch","RShoulderRoll"]
        value = [0.9,0.9]
        self.motion.setStiffnesses(name,value)
        self.set_home_position()      

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            rate.sleep()
            self.inference()
            self.update()

        rospy.spin()

        self.set_stiffness(False)

    #forward and backward pass
    def forward_pass(self, x, field_size):
        resolution = 50
        x = np.floor(x*resolution).astype(int)
        #print(x)
        #print(np.floor(field_size/2))
        y1_bot = np.max([x[0]-np.floor(field_size/2), 0]).astype(int)
        y1_top = np.min([x[0]+np.floor(field_size/2),49]).astype(int)
        y2_bot = np.max([x[1]-np.floor(field_size/2),0]).astype(int)
        y2_top = np.min([x[1]+np.floor(field_size/2),49]).astype(int)

        self.output1 = np.sum(self.weights[y1_bot:y1_top, y2_bot:y2_top, 0])
        self.output2 = np.sum(self.weights[y1_bot:y1_top, y2_bot:y2_top, 1])

    def load_weights(self):
        self.weights = np.load('/home/nao/bilhr23ss/workspace/src/tutorial_4/data/weights-150-receptive_field_5.npy')
        #self.weights = np.loadtxt('/home/nao/bilhr23ss/workspace/src/tutorial_4/data/weights-150.txt')
        #self.weights = np.array([self.weights])
        #print(self.weights.shape)
        
    
    def inference(self):

        print("Infered!")
        input_array = np.array([self.centerX, self.centerY])
        self.forward_pass(input_array, field_size = 5)
        self.target[0] = self.output1 * (RShoulderPitch_max - RShoulderPitch_min) + RShoulderPitch_min 
        self.target[1] = self.output2 * (RShoulderRoll_max - RShoulderRoll_min) + RShoulderRoll_min 
        print("Inference result after de-normalizing: ", self.target)


if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
