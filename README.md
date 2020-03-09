# Marker-Motion-Tracking
Developed a motion tracking system using a setup of webcams and a Kinect camera. The prototype used ArUco markers in a configuration that allowed accurate 3D pose tracking. The algorithm for sensor fusion was developed to be used with varying system dimensions. OpenCV, ArUco and PyKinect libraries were utilized.


Calibration was crucial as the initial step in this motion capture system. Camera parameters included intrinsics, extrinsics, and distortion coefficients. The camera parameters were estimated using multiple images of a calibration pattern acquired from the respective sensors. The calibration pattern was a checkerboard aligned at different orientations to the RGB camera. Using the correspondences, the camera parameters were found. 


A pinhole camera is a simple camera without a lens and with a single small aperture. Light rays pass through the aperture and project an inverted image on the opposite side of the camera. 
The pinhole camera parameters were represented in a 4-by-3 matrix called the camera matrix. This matrix mapped the 3-D world scene into the image plane. The calibration algorithm calculated the camera matrix using the extrinsic and intrinsic parameters. The extrinsic parameters represented the location of the camera in the 3-D scene. 

w[x y 1]=[X Y Z 1]P   

P=[R;T]K                            

The K matrix members give the focal distances, centre, and slew. The pinhole camera model does not account for lens distortion because an ideal pinhole camera does not have a lens. To accurately represent a real camera, the full camera model used by the algorithm included the radial and tangential lens distortion. Radial distortion is the most likely cause for errors in the image space. These then affect the coordinate mapping in camera space. 


A chessboard was used for the calibration of both the web camera and KV2 RGB sensors. The pattern found was from known points as specified in the code snippet. After the calibration, a camera matrix of 3x3 elements is extracted. The focal distances and the camera center coordinates were part of this matrix and made up the intrinsic parameters. This is labelled as mtx in the code snippet. The distortion coefficients, ‘dist’, is the vector


## Marker Tracking
The ArUco module that was used was based on the ArUco library which is a popular library for detection of square fiducial markers. A marker was printed using a utility script. Each marker is 30mm in length. 

Sensor fusion derived equations are available in "Results and pics" folder. The use of two sensors was to obtain a more accurate depth reading.


