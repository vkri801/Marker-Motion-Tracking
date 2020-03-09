#Script to test the motion capture system

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
from ArucoCamera import ArucoCamera
from KinectCamera import KinectCamera
from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from pykinect2.PyKinectV2 import _CameraSpacePoint
from Filter import Filter
import time
import math

a = 441
b = 45
x0 = 30

kinect = KinectCamera()
KinXFilter = Filter(5)
KinYFilter = Filter(5)
KinZFilter = Filter(5)
KinPitchFilter = Filter(5)
KinYawFilter = Filter(5)
KinRollFilter = Filter(5)

Cam1 = ArucoCamera(1)

CamXFilter = Filter(5)
CamYFilter = Filter(5)
CamZFilter = Filter(5)
CamPitchFilter = Filter(5)
CamYawFilter = Filter(5)
CamRollFilter = Filter(5)

CamTvec = open("Cam_Values", "w+")

startTime = time.time()

kinectTvec = open("Kinect_TVEC Values Single Experiment", "w+")

###------------------ ARUCO TRACKER FOR KINECT AND WEBCAM---------------------------
while (True):

    kinect.Tracking()
    Cam1.Tracking(Cam1.cap)
    cv2.imshow('Single Cam',Cam1.frame)
    
    cv2.imshow('Test Kinect', kinect.frame)
    cv2.namedWindow('Test Kinect Depth')
    cv2.imshow('Test Kinect Depth', kinect.frameDepth)
    cv2.namedWindow('Test Kinect IR')
    cv2.imshow('Test Kinect IR', kinect.frameInfrared)

    np.set_printoptions(threshold=np.inf)

    #print(kinect.csps1[100].z)
    if (np.any((kinect.ids != None)) & np.any((Cam1.ids != None))):
        ts = time.time()-startTime

        print(ts)
        print('\n')
        
       # print(kinect.kinect.FD2[212][256])
        
        KinXFilter.Processing(kinect.tvec[0][0][0])
        x2 = 1000*KinXFilter.average
        print(x2)
        print('\n')
        
        KinYFilter.Processing(kinect.tvec[0][0][1])
        y2 = 1000*KinYFilter.average
        print(y2)
        print('\n')
        
        KinZFilter.Processing(kinect.tvec[0][0][2])
        z2 = 1000*KinZFilter.average
        print(z2)
        print('\n')

        KinPitchFilter.Processing(kinect.pitchx)
        alpha2 = math.degrees(KinPitchFilter.average)
        print(alpha2)
        print('\n')

        KinYawFilter.Processing(kinect.yawy)
        beta2 = math.degrees(KinYawFilter.average)
        print(beta2)
        print('\n')

        KinRollFilter.Processing(kinect.rollz)
        gamma2 = math.degrees(KinRollFilter.average)
        print(gamma2)
        print('\n')
        print('--------------------------------')
        
        #kinectTvec.write((str(kinect.tvec)))
        kinectTvec.write("\n")
        
        
        #print(ts)
        #print('\n')\
        
        CamXFilter.Processing(Cam1.tvec[0][0][0])
        x1 = 1000*CamXFilter.average
        #print('\n')
        
        CamYFilter.Processing(Cam1.tvec[0][0][1])
        y1 = 1000*CamYFilter.average
        #print('\n')
        
        CamZFilter.Processing(Cam1.tvec[0][0][2])
        z1 = 1000*CamZFilter.average
        #print('\n')

        CamPitchFilter.Processing(Cam1.pitchx)
        alpha1 = math.degrees(CamPitchFilter.average)
        #print('\n')

        CamYawFilter.Processing(Cam1.yawy)
        beta1 = math.degrees(CamYawFilter.average)
        #print('\n')

        CamRollFilter.Processing(Cam1.rollz)
        gamma1 = (math.degrees(CamRollFilter.average))
        #print('\n')
        #print('--------------------------------')


        a3 = a + x1 - (x0*(math.cos(CamYawFilter.average))*(math.cos(CamRollFilter.average)))

        print(a3)
        print('--------------------------------')
        z_average = (a3 + z2)/2
        print(z_average)
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        #cv2.imwrite('SaveImage.jpg',kinect.frame)
        break

# When everything done, release the capture
kinectTvec.close()
kinect.kinect.close()
cv2.destroyAllWindows()
