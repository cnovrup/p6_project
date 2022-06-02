# p6_project
## Camera
`IntelCamera.py` with class ` Camera` - Handles communication with the camera and LiDAR

` Camera` - makes a ` Camera` object. Takes image dimensions as arguments

` get_frames()` - returns a depth and color image as a numpy array

` get_dist()` - takes the depth image and pixel coordinates and returns distance in meters to the point specified by the pixel coordinate

` stop_stream()` - stops the connection to the camera

## Manipulator
` KinovaArm.py` with class ` Arm` - Handles communication with the manipulator (described in \cref{kortexsection`)

` Arm` - makes a ` Arm` object. Takes a router object as argument.

` home()` - moves the arm to the predefined home position

` get_data()` - returns a ` Feedback` object that contains different data about the arm

` gripper()` - opens and closes the gripper to the specified percentage in the argument. 0 is open, 1 is closed.

` send_speeds()` - sends velocities to the end effector. Takes linear and angular velocities and duration of the command as arguments.

` utilities.py` - Contains utility functions for the manipulator (from the Kortex API)

` createTcpConnection` - creates a router object and takes ` KinovaArm.get_args()` as arguments

## Neural network
` Detection.py` with class ` Network` - Handles the object detection/YOLO network

` Network` - configures and loads the YOLO network

` run_detection()` - takes an image as argument and returns an array of objects it detected

` get_weed()` - takes an array of detected objects, and returns the bounding box and confidence of the first object in the list

` get_middle()` - takes two coordinates on an image, and returns the center of the rectangle spanned by the two coordinates.

## Estimator
` ModelEstimator.py` with class ` ModelEstimator` - Model based estimator

` ModelEstimator` - Takes the sampling time of the system as argument, discretizes the model and intitializes the model used for estimation

` update()` - takes the current error, end effector velocity and input to the system and updates the internal states and next states of the model

` estimate()` - takes the current input given to the system, and estimates the current error for each time it is called, to be used in control

## Controller
` Controller.py` with class ` Controller` - Controller for the manipulator

` Controller` - takes the proportional gains and the maximum control signal values as arguments and stores them locally in the object

` get_control()` - takes the error vector and the error of the vertical angle of the end effector, and calculates the control signals for the system (linear and angular velocities)

## Projection
` Projection.py` with class ` Projection` - Handles calculation of the error and projections to the image

` Projection` - Takes a camera object, translation of the camera wrt. gripper, rotation of the camera (pitch) and the image dimensions as arguments. It computes the needed rotation matrices to perform the different projective operations

` get_gripper_proj()` - returns the image coordinates of the gripper center (useful to display on the returned image in demonstratations).

` get3d_point()` - takes the coordinate of the center of the bounding box along with a distance reading to this point, and uses the inverse projection to calculate a 3D error vector which it returns

` get_proj()` - takes a 3D vector (error vector) and returns the image projection of this vectors endpoint. Useful to see where the estimator believes the plant is. 

## bounding_boxer
Contains the matlab code to automatically draw bounding boxes around the plant. bounding_boxer.m contains the main code for this, and in the file, the 3D world point along with the dimensions of the 3D bounding box is specified. It needs 4 folders in the same directory, called "images", "data", "bounding_box" and "bb_images". They contain the images to be labeled, the data about the camera position relative to the manipulator base, bounding box data and the images with the bounding box drawn. These folders need to be created manually. It does not matter what the image files are called, as long as an image file has the same name as the corresponding data file. The default file type is png, but can be changed in the script. if `savebox = false` it will not save images with the bounding box, which is recommended for large sets of data. If set to true, it will save the images with bounding box drawn on them. 
