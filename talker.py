#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import cv2
import time
import rospy
import copy
from std_msgs.msg import String
from keras.models import load_model

import numpy as np


#model = load_model(r'/home/GestureDetection/project_kojak/models/VGG_cross_validated.h5') 

class GestureDetector:
    def __init__(self):
        self.model = load_model(r'/home/GestureDetection/project_kojak/models/VGG_cross_validated.h5') 
        self.bgModel = None
        self.gesture_names = {0: 'Fist',
                     1: 'L',
                     2: 'Okay',
                     3: 'Palm',
                     4: 'Peace'}
        self.camera = cv2.VideoCapture(0)  # Change this according to number of attached cameras, 0 = default camera
        self.camera.set(10, 200)
        self.isBgCaptured = 0
        self.bgSubThreshold = 50
        self.threshold = 60  # binary threshold
        self.blurValue = 41  # GaussianBlur parameter
        self.cap_region_x_begin = 0.5  # start point/total width
        self.cap_region_y_end = 0.8  # start point/total width    
        self.originalWindow = cv2.namedWindow("originalWindow")
                

    def predict_rgb_image_vgg(self, image):                   
        image = np.array(image, dtype='float32')
        image /= 255
        pred_array = self.model.predict(image)
    
        result = self.gesture_names[np.argmax(pred_array)]
        score = float("%0.2f" % (max(pred_array[0]) * 100))
    
        return result, score
    
    def remove_background(self, frame):
        learningRate = 0
        fgmask = self.bgModel.apply(frame, learningRate=learningRate)
        kernel = np.ones((3, 3), np.uint8)
        fgmask = cv2.erode(fgmask, kernel, iterations=1)
        res = cv2.bitwise_and(frame, frame, mask=fgmask)
        return res
    
        
    def runDetector(self, pub):                
        
        if self.camera.isOpened():
            ret, frame = self.camera.read()           
            if np.shape(frame) == ():  
                rospy.loginfo("frame is empty")
            else:
                #rospy.loginfo("frame is ok")
                cv2.rectangle(frame, (int(self.cap_region_x_begin * frame.shape[1]), 0), 
                          (frame.shape[1], int(self.cap_region_y_end * frame.shape[0])), (255, 0, 0), 2)
                          
                if self.isBgCaptured == 0:
                    cv2.imshow("originalWindow", frame)
                else:
                    cv2.destroyWindow("originalWindow")
                
            if self.isBgCaptured == 1:  
                img = self.remove_background(frame)
                img = img[0:int(self.cap_region_y_end * frame.shape[0]), 
                          int(self.cap_region_x_begin * frame.shape[1]):frame.shape[1]]  # clip the ROI    
                # convert the image into binary image
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                blur = cv2.GaussianBlur(gray, (self.blurValue, self.blurValue), 0)
                ret, thresh = cv2.threshold(blur, self.threshold, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
         
                #ori is binary image with backgroud removed
                cv2.imshow('ori', thresh)    
                
                thresh1 = copy.deepcopy(thresh)
                #image parameter added
                image, contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                length = len(contours)
                maxArea = -1
                if length > 0:
                    for i in range(length):  # find the biggest contour (according to area)
                        temp = contours[i]
                        area = cv2.contourArea(temp)
                        if area > maxArea:
                            maxArea = area
                            ci = i
        
                    res = contours[ci]
                    hull = cv2.convexHull(res)
                    drawing = np.zeros(img.shape, np.uint8)
                    cv2.drawContours(drawing, [res], 0, (0, 255, 0), 2)
                    cv2.drawContours(drawing, [hull], 0, (0, 0, 255), 3)
                    #cv2.imshow('output', drawing)
                    
                    
                target = np.stack((thresh,) * 3, axis=-1)
                target = cv2.resize(target, (224, 224))
                target = target.reshape(1, 224, 224, 3)
                prediction, score = self.predict_rgb_image_vgg(target)
                #rospy.loginfo("prediction is: " + prediction + " with score: " + str(score))
                
                # org 
                org = (50, 50)     
                # font 
                font = cv2.FONT_HERSHEY_SIMPLEX 
                # Blue color in BGR 
                color = (255, 0, 0) 
                # Line thickness of 2 px 
                thickness = 2
                
                thresh = cv2.putText(thresh, prediction, org, font,  
                   1, color, thickness, cv2.LINE_AA) 
                #ori is binary image with backgroud removed
                cv2.imshow('ori', thresh)   
                
            
            k = cv2.waitKey(10) & 0xFF
            
            #Quit
            if k == ord('q'):
                self.camera.release()
                cv2.destroyAllWindows()
                
            # press 'b' to capture the background
            elif k == ord('b'):  
                self.bgModel = cv2.createBackgroundSubtractorMOG2(0, self.bgSubThreshold)
                time.sleep(2)
                self.isBgCaptured = 1
                print('Background captured')
                
            #Make prediction with spacebar
            elif k == 32:
                # If space bar pressed
                #cv2.imshow('original', frame)
                # copies 1 channel BW image to all 3 RGB channels
                target = np.stack((thresh,) * 3, axis=-1)
                target = cv2.resize(target, (224, 224))
                target = target.reshape(1, 224, 224, 3)
                prediction, score = self.predict_rgb_image_vgg(target)
    
                if prediction == 'Palm':
                    rospy.loginfo("palm " + str(score))
                    pub.publish("palm")
                elif prediction == 'Fist':
                    rospy.loginfo("fist " + str(score))
                    pub.publish("fist")
                elif prediction == 'L':
                    rospy.loginfo("L " + str(score))
                    pub.publish("L")
                elif prediction == 'Okay':
                    rospy.loginfo("okay")
                    pub.publish("Okay " + str(score))
                elif prediction == 'Peace':
                    rospy.loginfo("Peace " + str(score))
                    pub.publish("Peace")
                else:
                    pass

            

        else:
            rospy.loginfo("Camera not open")
            
        

def talker():
    #open_camera()

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10 h z asd
    detector = GestureDetector()
    
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
        
        detector.runDetector(pub)
        rate.sleep()

if __name__ == '__main__':
    #talker()

    
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
