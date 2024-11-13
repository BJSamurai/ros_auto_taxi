#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Bool

class SignalSim:
    def __init__(self):
        self.signal_sub = rospy.Subscriber('signal_sim', Bool, self.signal_cb)
        self.signal_pub = rospy.Publisher('signal_sim', Bool, queue_size = 1)
        self.cur_signal = False

        # Decide the signal interval
        self.timer = rospy.Timer(rospy.Duration(15), self.timer_callback)
                
    def signal_cb(self, msg):
        """Callback function for `odom_sub`."""
        self.cur_signal = msg.data
        #self.publish_data(self.cur_signal)

    def timer_callback(self, event):
        """Timer callback function to toggle the signal every 10 seconds."""
        # Toggle the current signal state
        self.cur_signal = not self.cur_signal

        # Publish the toggled signal
        self.signal_pub.publish(self.cur_signal)

        # Log the published value
        rospy.loginfo(f"Published toggled signal: {self.cur_signal}")

    def publish_data(self,cur_signal):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object we create below is not used as a geometric point,
        # but simply as a data container for `self.dist` and `self.yaw` so we can
        # publish it on `my_odom`.
        signal_pub_holder = self.cur_signal
        signal_pub_holder = not signal_pub_holder
        self.signal_pub.publish(signal_pub_holder)
        rospy.loginfo(f"Published toggled signal: {signal_pub_holder}")
        #rospy.sleep(10)

if __name__ == '__main__':
    rospy.init_node('signal_sim')
    SignalSim()
    rospy.spin()
