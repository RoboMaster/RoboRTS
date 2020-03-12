import rospy
import roslib
import tf
import time

class ShootOnce(Action):
    def __init__(self):
        pass

    def Execute(self):
        pass

    def Update(self):
        pass

    def Cancel():
        pass

class ShootContinuous(Action):
    def __init__(self):
        pass
    def Execute(self,switch):
        if(switch == "ON"):
            pass
        elif(switch=="OFF"):
            pass
        else:
            rospy.logerr("Invalid switch at shoot action")