#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32  # or whatever message type you need

class StopSign:
    def __init__(self):
        # Initialize the stop_sign topic as a publisher
        self.stop_pub = rospy.Publisher('stop_sign', Int32, queue_size=10)
        
        # Or if you want it as a subscriber:
        # self.stop_sub = rospy.Subscriber('stop_sign', Bool, self.callback)
    
    # If using subscriber, you'd have a callback:
    # def callback(self, msg):
    #     pass

if __name__ == '__main__':
    rospy.init_node('stop_sign')
    StopSign()
    rospy.spin()