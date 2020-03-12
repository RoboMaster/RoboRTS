import rospy
import roslib

from std_msgs import String, Bool
from roborts_msgs import GimbalAngle, TwistAccel

class Blackboard:
    @staticmethod
    def Initialize():
        '''
        Create Subscribers attached named with
        On_xxx
        '''
