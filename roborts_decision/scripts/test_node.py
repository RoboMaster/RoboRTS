import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import roslib
import tf
import numpy

points = [[5.54,3.9],[6.73,0.82],[2.49,4.00],[1.26,1.02]]
yaws   = [0,         3.14,        3.14,      0]
index  = [0]
status = ["IDLE","IDLE"] # May be modified
def chassis_callback(data):
    status[0] = data.data

def switch_detect(status):
    if(status[1]=="RUNNING" and status[0] != "RUNNING"):
        status[1] = status[0]
        print(status)
        return True
    else:
        print "false"+str(status)
        status[1] = status[0]
        return False


def publisher():
    pub = rospy.Publisher('PyDecision/Goal', Pose, queue_size=1)
    listenser = rospy.Subscriber("Actuator/Chassis",String,chassis_callback)
    rospy.init_node('pose_publisher', anonymous=True)
    rate = rospy.Rate(2) # Hz
    while not rospy.is_shutdown():
        p = Pose()
        p.position.x = points[index[0]][0]
        p.position.y = points[index[0]][1]
        p.position.z = 0.0
        # Make sure the quaternion is valid and normalized
        q = tf.transformations.quaternion_from_euler(0,0,yaws[index[0]])
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        if not(status[0]=="RUNNING"):
            pub.publish(p)
        if(switch_detect(status)):
            if(index[0]>=3):
                index[0] = 0
            else:
                index[0]+= 1
        rate.sleep()
        
        #print("Index is: "+str(index[0]))


if __name__ == '__main__':
    try:
        publisher()
    except rospy:
        pass
