import rospy
import roslib
import actionlib

from roborts_msgs.msg import ArmorDetectionAction

class TestAction(object):
    def __init__(self):
        self.server = actionlib.SimpleActionServer('armor_detection_node_action',ArmorDetectionAction,self.execute, False)
        self.server.start()

    def execute(self,goal):
        print("msg received"+str(goal.command))
        self.server.set_succeeded()

if __name__ == "__main__":
    print("Init Start")
    rospy.init_node('test_server_node')
    print("Init End")
    server = TestAction()
    print("Server Created")
    rospy.spin()