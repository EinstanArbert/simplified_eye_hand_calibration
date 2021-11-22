## [Camera Calibrator](http://wiki.ros.org/camera_calibration)
There are tutorials on how to run the calibration tool for [monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) and [stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) cameras.

### 1. Before Starting
*Make sure that you have the following:*

- a large [checkerboard](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf) with known dimensions. This tutorial uses a 8x6 checkerboard with 108mm squares. Calibration uses the interior vertex points of the checkerboard, so an "9x7" board uses the interior vertex parameter "8x6" as in the example below.
- a well lit 5m x 5m area clear of obstructions and check board patterns
- a monocular camera publishing images over ROS

### 2. Compiling
*Start by getting the dependencies and compiling the driver:*
`$ rosdep install camera_calibration`

Make sure that your monocular camera is publishing images over ROS. Let's list the topics to check that the images are published:
`$ rostopic list`

*To run the cameracalibrator.py node for a monocular camera using an 8x6 chessboard with 108mm squares:*
`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.023 image:=/camera/color/image_raw camera:=/camera/color --no-service-check`
When you click on the "Save" button after a succesfull calibration, the data (calibration data and images used for calibration) will be written to /tmp/calibrationdata.tar.gz.

*To run the cameracalibrator.py node for a stereo camera:*
`rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.02432 right:=/camera/infra2/image_rect_raw left:=/camera/infra1/image_rect_raw left_camera:=/camera/infra1 right_camera:=/camera/infra2 --no-service-check`

## Camera Check

*To run the command-line utility to check the calibration of a monocular camera:*
`rosrun camera_calibration cameracheck.py --size 8x6 monocular:=/forearm image:=image_rect`

*To run the command-line utility to check the calibration of a stereo camera:*
`rosrun camera_calibration cameracheck.py --size 8x6 stereo:=/wide_stereo image:=image_rect`
