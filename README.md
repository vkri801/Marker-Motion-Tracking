# Marker-Motion-Tracking
Developed a motion tracking system using a setup of webcams and a Kinect camera. The prototype used ArUco markers in a configuration that allowed accurate 3D pose tracking. The algorithm for sensor fusion was developed to be used with varying system dimensions. OpenCV, ArUco and PyKinect libraries were utilized.


Calibration was crucial as the initial step in this motion capture system. Camera parameters included intrinsics, extrinsics, and distortion coefficients. The camera parameters were estimated using multiple images of a calibration pattern acquired from the respective sensors. The calibration pattern was a checkerboard aligned at different orientations to the RGB camera. Using the correspondences, the camera parameters were found. 


A pinhole camera is a simple camera without a lens and with a single small aperture. Light rays pass through the aperture and project an inverted image on the opposite side of the camera. 
The pinhole camera parameters were represented in a 4-by-3 matrix called the camera matrix. This matrix mapped the 3-D world scene into the image plane. The calibration algorithm calculated the camera matrix using the extrinsic and intrinsic parameters. The extrinsic parameters represented the location of the camera in the 3-D scene. 

w[x y 1]=[X Y Z 1]P   

P=[R;T]K                            

