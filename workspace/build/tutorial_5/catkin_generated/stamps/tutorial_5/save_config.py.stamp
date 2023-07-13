import sys
import rospy
from math import pi
from rospy import rostime
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from naoqi import ALProxy


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

        self.key = ""


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.key = data.data

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        joint_names = data.name 
        joint_angles = data.position

        if(self.key == "s"):
            for i in len(joint_names):
                print(joint_names[i]+": "+joint_angles[i])




    def central_execute(self,):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        self.postureProxy.goToPosture("Stand", 0.5)
    
        rospy.spin()
        self.postureProxy.goToPosture("Crouch", 0.5)

if __name__=='__main__':
    # Instantiate central class and start loop
    central_instance = Central()
    central_instance.central_execute()