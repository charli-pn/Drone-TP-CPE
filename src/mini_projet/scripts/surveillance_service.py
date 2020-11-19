#! /usr/bin/python3
import rospy

from std_msgs.msg import Empty
from mini_projet.srv import SurveillanceService, SurveillanceServiceResponse


class SurveillanceSrv:
    isEnabled = False

    def __init__(self):
        self.start_service()

    def isSurveillanceActive(self, req):
        response = SurveillanceServiceResponse()
        response.isSurveillance = self.isEnabled
        return response

    def toggleSurveillance(self, arg):
        self.isEnabled = not self.isEnabled

    def start_service(self):
        rospy.init_node('surveillance_service')
        self.s = rospy.Service('surveillance_service', SurveillanceService, self.isSurveillanceActive)

        toggle_surveillance_topic = "/surveillance"
        # Set up your subscriber and define its callback
        rospy.Subscriber(toggle_surveillance_topic, Empty, self.toggleSurveillance)

        rospy.spin()

service = SurveillanceSrv()