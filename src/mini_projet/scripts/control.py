#!/usr/bin/env python
import rospy
import time

# Import custom message data.
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from mini_projet.srv import SurveillanceService

class Joystick:
    def __init__(self, data):
        self.takeoff = data.buttons[4] # LB
        self.land = data.buttons[5] # RB
        self.up = data.buttons[6] # LT
        self.down = data.buttons[7] # RT
        self.reset = data.buttons[8] # bouton "9" ??
        self.go_x_axis = data.axes[1] # front (+) and rear (-) (left joystick)
        self.go_y_axis = data.axes[0] # left (+) and right (-) (left joystick)
        self.turn_z_axis = data.axes[2] # left (+) and right (-) (right joystick)
        self.toggle_surveillance = data.buttons[0] # bouton 1


def callback(data):
    joys = Joystick(data)

    # Takeoff
    if joys.takeoff == 1:
        takeofPub.publish(Empty())

    # Land
    if joys.land == 1 :
        landPub.publish(Empty())
    
    # Reset
    if joys.reset == 1:
        emergencyPub.publish(Empty())

    # Toggel surveillance
    if joys.toggle_surveillance == 1:
        surveillancePub.publish(Empty())


    resp = is_surveillance()
    twist = Twist()
    if(resp.isSurveillance):
        twist.angular.z = 1
    else:
        twist.linear.x = joys.go_x_axis
        twist.linear.y = joys.go_y_axis
        # Up and Down
        if joys.up == 1 :
            twist.linear.z = 1
        elif joys.down == 1 :
            twist.linear.z = -1
        twist.angular.z = joys.turn_z_axis
    pilotePub.publish(twist)

    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)
    
    while (not rospy.is_shutdown()):
        time.sleep(0.1)
        resp = is_surveillance()
        if(resp.isSurveillance):
            twist = Twist()
            twist.angular.z = 1
            pilotePub.publish(twist)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    takeofPub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
    landPub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    emergencyPub = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
    pilotePub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
    surveillancePub = rospy.Publisher('/surveillance', Empty, queue_size=10)

    is_surveillance = rospy.ServiceProxy('surveillance_service', SurveillanceService)

    listener()