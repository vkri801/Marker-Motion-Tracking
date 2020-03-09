from __future__ import division
import time
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime

import cv2
import numpy as np
import math

import ctypes
import _ctypes
import sys

if sys.hexversion >= 0x03000000:
    import _thread as thread
else:
    import thread



class AcquisitionKinect():
    def __init__(self, resolution_mode=1.0):
        self.resolution_mode = resolution_mode

        self._done = False

        # Kinect runtime object, we want only color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Infrared)


        self._frameRGB = None
        self._frameDepth = None
        self._frameDepthQuantized = None
        self._frameSkeleton = None
        self._frameInfrared = None
        self.frameNum = 0


    def get_frame(self, frame):
        self.acquireFrame()
        frame.ts = int(round(time.time() * 1000))

        self.frameNum += 1

        # try:
        #     frame.frameRGB = self._frameRGB.copy()
        #     frame.frameDepth = self._frameDepth.copy()
        #     frame.frameSkeleton = self._frameSkeleton.copy()
        # except:
        frame.frameRGB = self._frameRGB
        frame.frameDepth = self._frameDepth
        frame.frameDepthQuantized = self._frameDepthQuantized
        frame.frameInfrared = self._frameInfrared


        frame.frame_num = self.frameNum


    def get_depth_frame(self):
        self._frameDepth = self._kinect.get_last_depth_frame()
        self.FD = self._frameDepth
        self._frameDepth = self._frameDepth.reshape(((424, 512))).astype(np.uint16)
        self.FD2 = self._frameDepth
        self._frameDepthQuantized = ((self._frameDepth.astype(np.int32)-500)/8.0).astype(np.uint8)

    def get_color_frame(self):
        self._frameRGB = self._kinect.get_last_color_frame()
        self._frameRGB = self._frameRGB.reshape((1080, 1920,-1)).astype(np.uint8)
        self._frameRGB = cv2.resize(self._frameRGB, (0,0), fx=1/self.resolution_mode, fy=1/self.resolution_mode)

    def get_infrared_frame(self):
        self._frameInfrared = self._kinect.get_last_infrared_frame()
        self._frameInfrared1 = self._frameInfrared.reshape(((424, 512))).astype(np.uint16)
        self._frameInfrared2 = self._frameInfrared.reshape(((424, 512))).astype(np.uint8)
   

    def draw_depth_frame(self):
        img[img<0]=0
        cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        image = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

        cv2.imshow("depth",self._frameDepth.astype(np.uint8))

    def draw_color_frame(self):
        # cv2.normalize(img, img, 0, 255, cv2.NORM_MINMAX)
        cv2.imshow("color",self._frameRGB.astype(np.uint8))





    def acquireFrame(self):
        if self._kinect.has_new_infrared_frame():
            self.get_infrared_frame()

        if self._kinect.has_new_depth_frame():
            self.get_depth_frame()

        if self._kinect.has_new_color_frame():
            self.get_color_frame()





    def close(self):
        self._kinect.close()
        self._frameDepth = None
        self._frameRGB = None
        self._frameInfrared = None

