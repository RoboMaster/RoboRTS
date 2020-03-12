import rospy
import roslib
import actionlib
import BT
import tf
from math import sin,cos,pi
import time
import PyConfig


from roborts_msgs.msg import GimbalAngle
from roborts_msgs.msg import ArmorDetectionAction, ArmorDetectionGoal
'''
REMARK: These Node Almost Always Return SUCCESS since it just publish messages
'''

class GimbalPatrol(BT.Action):
    # class wise static variable
    zero_gimbal_angle = GimbalAngle()
    def __init__(self,name):
        super(GimbalPatrol,self).__init__(name)
        self._cmd_gimbal_angle_pub = rospy.Publisher('cmd_gimbal_angle', GimbalAngle, queue_size=10)
        self._status = "IDLE"
        GimbalPatrol.zero_gimbal_angle.yaw_mode = True
        GimbalPatrol.zero_gimbal_angle.pitch_mode = False
        GimbalPatrol.zero_gimbal_angle.yaw_angle = PyConfig.MIDDLE
        GimbalPatrol.zero_gimbal_angle.pitch_angle = 0
        self.current_yaw = 0
        self.current_pitch = 0
        self.gimbal_msg = GimbalAngle()
        self.gimbal_msg.yaw_mode = True
        self.gimbal_msg.pitch_mode = False

    def Execute(self): # Assume Angle here is a structure with mode, yaw,pitch and modes
        try:
            self.gimbal_msg.yaw_angle = 60*sin(self.current_yaw*pi/180)
            self.gimbal_msg.pitch_angle = 10*sin(self.current_pitch*pi/180)
            self.current_pitch = (self.current_pitch + PyConfig.PARTROL_RATE) % 360
            self.current_yaw = (self.current_yaw + PyConfig.PARTROL_RATE) % 360
            self._cmd_gimbal_angle_pub.publish(self.gimbal_msg)
        except:
            print("Unable to Publish")
            self._status = "FAILURE"
            return
        self._status = "SUCCESS"

        
    def Update(self):
        return self._status

    def _cleanup(self):
        try:
            #self._cmd_gimbal_rate_pub.Publish(zero_gimbal_rate_)
            self._cmd_gimbal_angle_pub.publish(PatrolAction.zero_gimbal_angle)
        except:
            rospy.logerr("Wrong Execution Mode")

class GimbalLeftTurn(BT.Action):
    def __init__(self,name):
        super(GimbalLeftTurn,self).__init__(name)
        self.cmd_publisher = rospy.Publisher("cmd_gimbal_angle",GimbalAngle,queue_size = 10)
        self._status = "IDLE"
        self.gimbal_msg = GimbalAngle()
        self.gimbal_msg.yaw_mode = True
        self.gimbal_msg.pitch_mode = False
        # To be modified
        self.gimbal_msg.yaw_angle = PyConfig.LEFT
        self.gimbal_msg.pitch_angle = 0

    def Execute(self):
        try:
            self.cmd_publisher.publish(self.gimbal_msg)
        except:
            print("Unable to Publish")
            self._status = "FAILURE"
            return
        self._status = "SUCCESS"

    def Update(self):
        return

class GimbalRightTurn(BT.Action):
    def __init__(self,name):
        super(GimbalRightTurn,self).__init__(name)
        self.cmd_publisher = rospy.Publisher("cmd_gimbal_angle",GimbalAngle,queue_size = 10)
        self._status = "IDLE"
        self.gimbal_msg = GimbalAngle()
        self.gimbal_msg.yaw_mode = True
        self.gimbal_msg.pitch_mode = False
        # To be modified
        self.gimbal_msg.yaw_angle = PyConfig.RIGHT
        self.gimbal_msg.pitch_angle = 0

    def Execute(self):
        try:
            self.cmd_publisher.publish(self.gimbal_msg)
        except:
            print("Unable to Publish")
            self._status = "FAILURE"
            return
        self._status = "SUCCESS"

    def Update(self):
        return

class GimbalMiddleTurn(BT.Action):
    def __init__(self,name):
        super(GimbalMiddleTurn,self).__init__(name)
        self.cmd_publisher = rospy.Publisher("cmd_gimbal_angle",GimbalAngle,queue_size = 10)
        self._status = "IDLE"
        self.gimbal_msg = GimbalAngle()
        self.gimbal_msg.yaw_mode = True
        self.gimbal_msg.pitch_mode = False
        # To be modified
        self.gimbal_msg.yaw_angle = PyConfig.MIDDLE
        self.gimbal_msg.pitch_angle = 0

    def Execute(self):
        try:
            self.cmd_publisher.publish(self.gimbal_msg)
        except:
            print("Unable to Publish")
            self._status = "FAILURE"
            return
        self._status = "SUCCESS"

    def Update(self):
        return self._status

'''
REMARK: This only switch to detect but not track mode
Always return True when data published successfully
'''
class GimbalDetect(BT.Action):
    def __init__(self,name):
        super(GimbalDetect,self).__init__(name)
        self._action_client = actionlib.SimpleActionClient('armor_detection_node_action',ArmorDetectionAction)
        print("Wait For Server")
        self._action_client.wait_for_server()
        print("Server Connected")
        self._goal = ArmorDetectionGoal()
        self._goal.command = 4
        self._status = "IDLE"

    def Execute(self):
        try:
            self._action_client.send_goal(self._goal)
        except:
            rospy.logwarn("Goal failed to send")
            self._status = "FAILURE"
            return 
        self._status = "SUCCESS"

    def Update(self):
        return self._status

class GimbalTrack(BT.Action):
    def __init__(self,name):
        super(GimbalTrack,self).__init__(name)
        self._action_client = actionlib.SimpleActionClient('armor_detection_node_action',ArmorDetectionAction)
        print("Wait for Server")
        self._action_client.wait_for_server()
        print("Server Connected")
        self._goal = ArmorDetectionGoal()
        self._goal.command = 1
        self._status = "IDLE"
        self.test = 0

    def Execute(self):
        try:
            self._action_client.send_goal(self._goal)
        except:
            rospy.logwarn("Goal failed to send")
            self._status = "FAILURE"
            return 
        if self.test == 0:
            self._status = "SUCCESS"
            self.test = 1
        else:
            self._status = "FAILURE"
            self.test = 0

    def Update(self):
        return self._status

if __name__=="__main__":
    rospy.init_node("gimbal_interface_test",anonymous=True)
    gimbal1 = GimbalTrack("Action1")
    gimbal2 = GimbalPatrol("Action2")
    gimbal01 = BT.LogicFallback("LNode1",[gimbal1,gimbal2])
    gimbal3 = GimbalTrack("Action3")
    gimbal4 = GimbalPatrol("Action4")
    gimbal02 = BT.LogicFallback("LNode2",[gimbal3,gimbal4])
    gimbal = BT.LogicSequential("Root",[gimbal01,gimbal02])
    rate = rospy.Rate(60)
    gimbal.getName(1)
    while not rospy.is_shutdown():
        gimbal.OnTick()
        rate.sleep()