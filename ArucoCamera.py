#Class for creating video captures

import numpy as np
import cv2
import cv2.aruco as aruco
import glob
import math

class ArucoCamera:
    
    def __init__(self, camIndex):

        
        self.cap = cv2.VideoCapture(camIndex)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
        self.rotM = np.zeros(shape=(3,3))
        
        ####---------------------- CALIBRATION ---------------------------
        # termination criteria for the iterative algorithm
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        # checkerboard of size (7 x 6) is used
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.

        # iterating through all calibration images
        # in the folder
        images = glob.glob('calib_images/*.jpg')

        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

            # find the chess board (calibration pattern) corners
            ret, corners = cv2.findChessboardCorners(gray, (7,6),None)

            # if calibration pattern is found, add object points,
            # image points (after refining them)
            if ret == True:
                objpoints.append(objp)

                # Refine the corners of the detected corners
                corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
                imgpoints.append(corners2)

                # Draw and display the corners
                img = cv2.drawChessboardCorners(img, (7,6), corners2,ret)
                cv2.imwrite('SaveImageCam.jpg',img)

        ret, self.mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)



    ###------------------ ARUCO TRACKER FOR TWO CAMERAS---------------------------

    def Tracking(self, cap):
        self.ret, self.frame = self.cap.read()
        # operations on the frame
    
        self.gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # set dictionary size depending on the aruco marker selected
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

        # detector parameters can be set here (List of detection parameters[3])
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        # lists of ids and the corners belonging to each id
        self.corners, self.ids, self.rejectedImgPoints = aruco.detectMarkers(self.gray, aruco_dict, parameters=parameters)

        # font for displaying text (below)
        font = cv2.FONT_HERSHEY_SIMPLEX

        # check if the ids list is not empty
        # if no check is added the code will crash
        if np.all(self.ids != None):

            # estimate pose of each marker and return the values
            # rvet and tvec-different from camera coefficients
            self.rvec, self.tvec ,_ = aruco.estimatePoseSingleMarkers(self.corners, 0.029, self.mtx, self.dist)
        
            #(rvec-tvec).any() # get rid of that nasty numpy value array error

            for i in range(0, self.ids.size):
                # draw axis for the aruco markers
                aruco.drawAxis(self.frame, self.mtx, self.dist, self.rvec[i], self.tvec[i], 0.1)
                
            # draw a square around the markers
            aruco.drawDetectedMarkers(self.frame, self.corners)


            self.TVEC = 1000*self.tvec[0][0]
            self.rotM = cv2.Rodrigues(self.rvec)[0]

            self.sy = math.sqrt(self.rotM[0,0] * self.rotM[0,0] +  self.rotM[1,0] * self.rotM[1,0])
            singular = self.sy < 1e-6
 
            if  not singular :
                self.pitchx = math.atan2(self.rotM[2,1] , self.rotM[2,2])
                self.yawy = math.atan2(-self.rotM[2,0], self.sy)
                self.rollz = math.atan2(self.rotM[1,0], self.rotM[0,0])
            
            else :
                self.pitchx = math.atan2(-self.rotM[1,2], self.rotM[1,1])
                self.yawy = math.atan2(-self.rotM[2,0], self.sy)
                self.rollz = 0

            # code to show ids of the marker found
            strg = ''
            for i in range(0, self.ids.size):
                strg += str(self.ids[i][0])+', '

            cv2.putText(self.frame, "Id: " + strg, (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)

        else:
            # code to show 'No Ids' when no markers are found
            cv2.putText(self.frame, "No Ids", (0,64), font, 1, (0,255,0),2,cv2.LINE_AA)


    
