#!/usr/bin/env python3
import rospy
import tf
import cv2 as cv
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
from matplotlib import pyplot as plt
from std_msgs.msg import String
import numpy as np
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from samples.cfg import CvexampleConfig as ConfigType


#Video to Simulation Video
#https://drive.google.com/file/d/1Nm7reRSltxldMJ6xIZR56-naogdGgbmK/view?usp=sharing
#Video to IRL Video
#https://drive.google.com/file/d/1NQxBOkMlB3myCDDfD2RX2HNyrrOiR4SI/view?usp=sharing


class LineFollowerSim:
    def __init__(self):
        self.param_ready = False
        cv.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        #initialize all filtered images
        self.masked_pub = rospy.Publisher("/cvexample/masked/compressed", CompressedImage, queue_size=1)
        self.grayed_pub = rospy.Publisher("/cvexample/grayed/compressed", CompressedImage, queue_size=1)
        self.blurred_pub = rospy.Publisher("/cvexample/blurred/compressed", CompressedImage, queue_size=1)
        self.contour_pub = rospy.Publisher("/cvexample/contour/compressed", CompressedImage, queue_size=1)
        self.centroid_pub = rospy.Publisher("/cvexample/centroid/compressed", CompressedImage, queue_size=1)
        self.dynamic = DynamicReconfigureServer(ConfigType, self.dynamic_cb)

        self.centroid_image = None

        rospy.loginfo("Initialized")

    def image_callback(self, msg):
        """Callback to `self.image_sub`."""
        if (self.param_ready):
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.hsv_image = cv.cvtColor(self.rgb_image, cv.COLOR_BGR2HSV)
            self.create_masked_image()
            self.create_grey_image()
            self.create_centroid_image()
            self.create_blurred_image()
            self.create_contours()
            rospy.loginfo("image_callback all done")

        #raise NotImplementedError
    
    def follow_line(self):
        """Follow a yellow line."""

        if hasattr(self, 'grey_masked_image') and self.centroid_image is not None:
        # Calculate the moments of the masked (line) image to find the centroid
            M = cv.moments(self.grey_masked_image)
            
            #if it is > 0, there is a centroid
            if M['m00'] > 0: 
                #get the centroid's x and y
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # get the center x position of the image
                center_x = self.centroid_image.shape[1] // 2
                
                # calculate the error between the center of the image and the centroid
                error = center_x - cx
                
                # set a base parameter for angular z calculation
                kp = 0.001 
                
                # actual cmd_vel twist
                twist = Twist()

                twist.linear.x = 0.1 

                if error > 0.1 or error < -0.1:
                    twist.angular.z = kp * error  # yaw adjustment base on error
                else:
                    twist.angular.z = 0.0 #if the error too small, don't turn
                
                self.cmd_vel_pub.publish(twist)
                
                # display the info of twist
                rospy.loginfo(f"Centroid at ({cx}, {cy}), Error: {error}, Angular Z: {twist.angular.z}")
            else:
                # If no line is detected, stop or search for the line
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.3  # Turn in place to search for the line
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("Line not detected; searching...")
        

        #raise NotImplementedError

    def run(self):
        """Run the Program."""
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.follow_line()


    
    #### Calculation of the filtered images
    
    def create_masked_image(self):
        # range of colors, found by trial and error
        lower_color_bound = np.array([self.config.lb_h, self.config.lb_s, self.config.lb_v])
        upper_color_bound = np.array([self.config.ub_h, self.config.ub_s, self.config.ub_v])
        # rospy.loginfo(f"Color bounds: {lower_color_bound} to {upper_color_bound}")

        #print (f"{self.hsv_image.shape}")
        # find pixels in range bounded by BGR color bounds
        self.mask = cv.inRange(self.hsv_image, lower_color_bound, upper_color_bound)

        # find pixels that are in both mask AND original img
        self.masked_hsv_img = cv.bitwise_and(self.hsv_image, self.hsv_image, mask=self.mask)
        self.masked_rgb_image = cv.cvtColor(self.masked_hsv_img, cv.COLOR_HSV2BGR)

        masked_msg = self.bridge.cv2_to_compressed_imgmsg(self.masked_rgb_image)
        self.masked_pub.publish(masked_msg)
        rospy.loginfo("Masked Image DONE")
    
    def create_grey_image(self):
        self.grey_image = cv.cvtColor(self.rgb_image, cv.COLOR_RGB2GRAY)
        self.grey_masked_image = cv.cvtColor(self.masked_rgb_image, cv.COLOR_RGB2GRAY)
        grey_masked_msg = self.bridge.cv2_to_compressed_imgmsg(self.grey_masked_image, dst_format='jpg')  # Add dst_format
        self.grayed_pub.publish(grey_masked_msg)        


    def create_centroid_image(self):
        self.centroid_image = self.rgb_image.copy()
        M = cv.moments(self.grey_masked_image)
        if M['m00'] > 0:
             cx = int(M['m10']/M['m00'])
             cy = int(M['m01']/M['m00'])
             cv.circle(self.centroid_image, (cx, cy), 50, (0,0,0), -1)
        centroid_msg = self.bridge.cv2_to_compressed_imgmsg(self.centroid_image)
        self.centroid_pub.publish(centroid_msg)

    def create_blurred_image(self):
        # Now blurr
        self.blurred_image = cv.GaussianBlur(self.grey_masked_image, (self.config.blurr*2+1, self.config.blurr*2+1), 0)  # blur image with a 5x5 kernel
        blurred_msg = self.bridge.cv2_to_compressed_imgmsg(self.blurred_image)
        self.blurred_pub.publish(blurred_msg)

    def create_contours(self):
        # # Lets try countours
        ret, thresh = cv.threshold(
            self.blurred_image, 0, 255, cv.THRESH_BINARY_INV)  # create an threshold
        contours, hierachy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        self.contour_image = cv.drawContours(thresh, contours, -1, (255, 255, 255), 20)
        contour_msg = self.bridge.cv2_to_compressed_imgmsg(self.contour_image)
        self.contour_pub.publish(contour_msg)

    
    #This defined values are for the yellow line only
    def dynamic_cb(self, config, level):
        rospy.loginfo("Dynamic Config callback {lb_h}:{lb_s}:{lb_v} {ub_h}:{ub_s}:{ub_v}".format(**config))
        config.lb_h = 15
        config.lb_s = 0
        config.lb_v = 0
        config.ub_h = 180
        config.ub_s = 255
        config.ub_v = 255
        config.blurr = 10
        self.config = config
        self.config = config
        self.param_ready = True
        return config
    

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    LineFollowerSim().run()
    rospy.spin()