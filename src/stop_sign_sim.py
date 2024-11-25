#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Int32

class StopSignSim:
    def __init__(self):
        self.stop_sign_sub = rospy.Subscriber('stop_sign_sim', Int32, self.stop_sign_cb)
        self.stop_sign_pub = rospy.Publisher('stop_sign_sim', Int32, queue_size = 1)
        self.cur_cars_count = 0
                
    def signal_cb(self, msg):
        """Callback function for `odom_sub`."""
        self.cur_cars_count = msg.data
        #self.publish_data(self.cur_signal)

    def publish_data(self,cur_signal):
        """
        Publish stop_sign count in 'stop_sign_sim'.
        """
        signal_pub_holder = self.cur_signal
        signal_pub_holder = not signal_pub_holder
        self.signal_pub.publish(signal_pub_holder)
        rospy.loginfo(f"Published toggled signal: {signal_pub_holder}")
        

if __name__ == '__main__':
    rospy.init_node('stop_sign_sim')
    StopSignSim()
    rospy.spin()
