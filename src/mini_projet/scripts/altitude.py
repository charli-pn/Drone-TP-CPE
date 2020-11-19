#!/usr/bin/env python
import rospy

# Import custom message data.
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32



def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " altitude : " +  str(data.pose.pose.position.z))
    pub.publish(data.pose.pose.position.z)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/bebop/odom", Odometry, callback)
    

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('/altitude', Float32, queue_size=10)
    listener()
