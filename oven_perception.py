#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import message_filters
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from std_msgs.msg import Int64MultiArray, Float64MultiArray, MultiArrayDimension
import ros_numpy
import matplotlib.pyplot as plt
from PIL import Image as ImageShow
import os
import math
from collections import deque


# Visualization Parameters and Constants
VIZ = True
BBOX_THICKNESS = 2
COLOR = (0, 255, 0)
ORG = (550, 880)
DEQUE_SIZE = 30

# 3-tuples are BGR, not RGB format
COLORS = {"stove_light": {'lower_H': 1.5, 'upper_H': 178.5,
                       'lower_S': 73.95, 'upper_S': 127.5,
                       'lower_V': 209.1, 'upper_V': 255},
                       
         "green": (0, 255, 0),
         "red": (0, 0, 255)}

ITEM_SIZES = {"light": (60, 100)}


class StoveLightMasking(object):
    def __init__(self, image, y1, y2, x1, x2):
        # Boundary of the image crop for the region surrounding the stove light
        self.image = image
        self.y1 = y1
        self.y2 = y2
        self.x1 = x1
        self.x2 = x2

    def get_cropped_image(self):
        return self.image[self.y1:self.y2, self.x1:self.x2]

    def get_num_white_pixels(self, filtered_image):
        return np.sum(filtered_image == 255)

    def get_filtered_image(self, lower_H, upper_H, lower_S, upper_S, lower_V, upper_V):
        # convert input image to HSV color space with OpenCV
        hsv_image = cv2.cvtColor(self.get_cropped_image(), cv2.COLOR_BGR2HSV)
        # define lower and upper HSV values
        lower = np.array([lower_H,lower_S,lower_V])
        higher = np.array([upper_H,upper_S,upper_V])
        # filter image in HSV color space
        filtered_image = cv2.inRange(hsv_image,lower,higher)
        return filtered_image

class OvenPerception():
    """ROS node """

    def __init__(self):

        # Init the node
        rospy.init_node('oven_perception')

        # Publishers
        self.oven_dection_state = rospy.Publisher('/oven_perception/light_state', Bool, queue_size=5)
       
        # Subscribers
        self.rgb_img_sub = message_filters.Subscriber('/master/rgb/image_raw', Image)

        self.synchronizer = message_filters.TimeSynchronizer(
            [self.rgb_img_sub], 10)
        self.synchronizer.registerCallback(self.get_image_cb)

        # Deque used for sliding window of 30 frames counts
        self.de = deque([0] * 30)

        # main thread just waits now..
        rospy.spin()

    def get_image_cb(self, ros_rgb_image):
        self.rgb_image = ros_numpy.numpify(ros_rgb_image)
        self.rgb_image_timestamp = ros_rgb_image.header.stamp
        
        ###################
        
        # Perception code logic:
        # continuously add the white pixel count to the deque
        # if 80% of the values in the deque indicate stove is on (i.e. the white pixel count is not 0),
        # then send TRUE message to publisher, indicating the stove is on, otherwise send FALSE as the message

        ###################
        
        stove_masker = StoveLightMasking(self.rgb_image, 800, 850, 550, 600)
        filtered_bounding_box_image = stove_masker.get_filtered_image(COLORS["stove_light"]['lower_H'],COLORS["stove_light"]['upper_H'],
                                COLORS["stove_light"]['lower_S'],COLORS["stove_light"]['upper_S'],
                                COLORS["stove_light"]['lower_V'],COLORS["stove_light"]['upper_V'])

        print("Number of white pixels:")
        white_pixel_count = stove_masker.get_num_white_pixels(filtered_bounding_box_image)
        # Delete the oldest value off the Deque (found at the far left)
        self.de.popleft()
        # Append the new value of the white pixels count found in the masked region to the far right
        self.de.append(white_pixel_count)
        print(white_pixel_count)
        cv2.imwrite("filtered_bounding_box_image.png", filtered_bounding_box_image)

        # Count the percentage of occurrences of 0 in the list
        zero_count = self.de.count(0) / DEQUE_SIZE

        # If 20% or more of the white pixel counts in Deque are 0 (meaning less than 80% of the white pixel counts are not 0), then the stove is likely off
        # Visualize image with green bounding box around stove light if it's found to be on; red bounding box if it's off
        if zero_count >= 0.20:
            # The oven light is NOT on
            print("STOVE OFF!")
            self.oven_dection_state.publish(Bool(False))
            bounding_box_image = self.visualize_bbox([550, 800, 600, 850], COLORS["red"])
            cv2.imwrite("bounding_box_image.png", bounding_box_image)
        else:
            # The oven light IS on
            print("STOVE ON!!!!!!!!!!!!!")
            self.oven_dection_state.publish(Bool(True))
            bounding_box_image = self.visualize_bbox([550, 800, 600, 850], COLORS["green"])
            cv2.imwrite("bounding_box_image.png", bounding_box_image)
    

    def visualize_bbox(self, detection_bbox, color):
        '''
        Vizualize axis-aligned and rotated bounding boxes on the image

        Parameters:
                    detection_bbox (list(list)): List of all axis aligned bounding boxes in the [(x_min, y_min), (x_max, y_max)] format
                    color (tuple): Color of the bounding box
        Returns:
                    detection_image (image): Image with bounding box drawn on the image
        '''
        detection_image = self.rgb_image 

        detection_image = self.draw_aa_box(self.rgb_image, detection_bbox, color, BBOX_THICKNESS)
        return detection_image

    def draw_aa_box(self, image, box, color, thickness):
        '''
        Helper function to draw axis aligned bounding box on the image

        Parameters:
                    image (image): Input image from get_image_cb
                    box (list): Coordinates of the bounding box in the [x_min, y_min, x_max, y_max] format
                    color (tuple): Color of the bounding box
                    thickness (int): Thickness of the bounding box
        Returns:
                    image (image): Image with the bounding box plotted on the image
        '''
        start_point = (box[0], box[1])
        end_point = (box[2], box[3])

        image = cv2.rectangle(image, start_point, end_point, color, thickness)
    
        cv2.putText(image, "oven_light", ORG,  cv2.FONT_HERSHEY_SIMPLEX, 1, color, thickness)
       
        return image

    def cv2_to_imgmsg(self, cv_image):
        '''
        Helper function to publish a cv2 image as a ROS message (without using ROS cv2 Bridge)
        https://answers.ros.org/question/350904/cv_bridge-throws-boost-import-error-in-python-3-and-ros-melodic/

        Parameters:
                    cv_image (image): Image to publish to a ROS message
        Returns:
                    img_msg (message): Image message published to a topic 
        '''
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

if __name__ == '__main__':
    try:
        node = OvenPerception()
    except rospy.ROSInterruptException:
        pass
