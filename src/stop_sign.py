#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32 

# This class provide a rostopic with 'int' value to store the number of cars at stop sign

class StopSign:
    def __init__(self):
        # Initialize the stop_sign topic as a publisher
        self.stop_pub = rospy.Publisher('stop_sign', Int32, queue_size=10)
        

if __name__ == '__main__':
    rospy.init_node('stop_sign')
    StopSign()
    rospy.spin()