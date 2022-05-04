#!/usr/bin/env python

import cv2
import numpy as np
import argparse

COLORS = {"stove_light": {'lower_H': 1.5, 'upper_H': 178.5,
                       'lower_S': 73.95, 'upper_S': 127.5,
                       'lower_V': 209.1, 'upper_V': 255}}

ITEM_SIZES = {"light": (60, 100)}

# Number of white pixels (when stove is off):
# image 1: 0
# image 2: 0
# image 3: 0
# image 4: 0
# image 5: 0
# image 6: 67
# image 7: 62
# image 8: 76
# image 9: 74
# image 10: 55

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

    def get_bounding_box(self, filtered_image, image_type):
        contours,_ = cv2.findContours(filtered_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if ITEM_SIZES[image_type][0] < cv2.contourArea(c) < ITEM_SIZES[image_type][1]:
                # granular contours
                cv2.drawContours(filtered_image, [c], -1, (150, 150, 150), 8)

                rect = cv2.minAreaRect(c)
                (x,y),(w,h),a = rect
                
                # box = cv2.boxPoints(rect)
                box = cv2.boxPoints(rect)
                box = np.int0(box) #turn into ints
                cv2.drawContours(filtered_image,[box], 0, (240, 240, 240), 3)
    
        cv2.imshow('contours', filtered_image)
        cv2.waitKey(0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", help="name of item color",type=str)
    parser.add_argument("--type", help="name of item type",type=str)
    parser.add_argument("--image", help="name of item type",type=int)
    args = parser.parse_args()
    

    
    image = cv2.imread(f"images/stove_{args.image}.jpeg")
    
    stove_masker = StoveLightMasking(image, 780, 810, 690, 730)
    # get cropped image showing the area around the stove light
    cropped_image = stove_masker.get_cropped_image()
    cv2.imshow('BGR', cropped_image)
    cv2.waitKey(0)
    color = args.color
    image_type = args.type
    filtered_img = stove_masker.get_filtered_image(COLORS[color]['lower_H'],COLORS[color]['upper_H'],
                                COLORS[color]['lower_S'],COLORS[color]['upper_S'],
                                COLORS[color]['lower_V'],COLORS[color]['upper_V'])

    print("Number of white pixels:")
    num_white_pix = stove_masker.get_num_white_pixels(filtered_img)
    print(num_white_pix)

    contoured_image = stove_masker.get_bounding_box(filtered_img, image_type)

    