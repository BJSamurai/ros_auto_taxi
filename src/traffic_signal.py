#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool

# This class provide a rostopic 'traffic_signal' which will change between True / False every 10 seconds

class SignalSim:
    def __init__(self):
        self.signal_sub = rospy.Subscriber('traffic_signal', Bool, self.signal_cb)
        self.signal_pub = rospy.Publisher('traffic_signal', Bool, queue_size = 1)
        self.cur_signal = False

        # Decide the signal interval
        self.timer = rospy.Timer(rospy.Duration(10), self.timer_callback)
                
    def signal_cb(self, msg):
        """Callback function for `odom_sub`."""
        self.cur_signal = msg.data

    def timer_callback(self, event):
        """Timer callback function to toggle the signal every 10 seconds."""
        self.cur_signal = not self.cur_signal
        self.signal_pub.publish(self.cur_signal)

        # Log the published value
        rospy.loginfo(f"Published toggled signal: {self.cur_signal}")

    def publish_data(self,cur_signal):
        """
        Publish Boolean value of Ture or False on the `signal` topic.
        """
        signal_pub_holder = self.cur_signal
        signal_pub_holder = not signal_pub_holder
        self.signal_pub.publish(signal_pub_holder)
        rospy.loginfo(f"Published toggled signal: {signal_pub_holder}")
        #rospy.sleep(10)

if __name__ == '__main__':
    rospy.init_node('traffic_signal')
    SignalSim()
    rospy.spin()
