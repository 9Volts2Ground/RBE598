# Intro

Camera calibration is done using the open-source `camera_calibration` package from ROS. Details can be found here:

http://wiki.ros.org/camera_calibration

The tutorial to use this package can be found here:

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

# Calibration

To run the calibrator, hold up an `nxm` checker board with squares of size `X` to the camera, and run the following command:

`rosrun camera_calibration cameracalibrator.py --size nxm --square X image:=/camera/image_raw camera:=/camera --no-service-check`

Once the calibrator collects enough points, click "collect" and then "save". It zips a bunch of data up and places it under `/tmp/calibrationdata.tar.gz`. Move this into the `/ROS/src/hardware_control/params` directory, and unzip the data by calling `tar -xvzf calibrationdata.tar.gz`.

Unzipping this file should output several images captured during calibration, as well as two calibration files: `ost.txt` and `ost.yaml`. The `.txt` file is in INI format, and the `.yaml` file is the same format as a standard ROS parameter file. Rename `ost.txt` appropriately and save it.

# Launch File Update

The camera calibration parameters are pulled in when launching `/hardware_control/launch/camera.launch`. To use a new calibrated camera, update this Launch file with the appropriate calibration data parameters file.
