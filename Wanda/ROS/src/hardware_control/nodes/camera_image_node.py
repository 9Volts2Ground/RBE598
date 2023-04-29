#!/usr/bin/env python3
# Based on code from:
# - https://automaticaddison.com
# - https://pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/

# Import the necessary libraries
import cv2
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import numpy as np
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# Custom libraries
from system_globals import frames
from topic_messages import topics

#================================================
# Get global parameters
IS_WANDA = rospy.get_param( 'IS_WANDA', default=False )

frame = frames.frames()
top = topics.topics()

#==============================================================================
class camera_image_node():
    def __init__(self):
        # Tells rospy the name of the node.
        # Anonymous = True makes sure the node has a unique name. Random
        # numbers are added to the end of the name.
        rospy.init_node('camera_image_node', anonymous=False)

        # Control max rate of image capture, Hz
        self.frame_rate = 10 # Hz # ToDo: make this a parameter
        self.rate = rospy.Rate( self.frame_rate )

        self.image_encoding = "bgr8" # ToDo: make this a parameter

        # Set up camera info parameters
        self.cinf = CameraInfo()
        self.cinf.header.stamp = rospy.Time.now()
        self.cinf.header.frame_id = frame.camera
        self.cinf.height = rospy.get_param( '/camera/image_height', default=720 )
        self.cinf.width = rospy.get_param( '/camera/image_width', default=1280 )
        self.cinf.distortion_model = rospy.get_param( '/camera/distortion_model', default='plumb_bob' )
        self.cinf.D = rospy.get_param( '/camera/distortion_coefficients/data', default=np.zeros((1,5)) )
        self.cinf.K = rospy.get_param( '/camera/camera_matrix/data', default=np.zeros((1,9)) )
        self.cinf.R = rospy.get_param( '/camera/rectification_matrix/data', default=np.zeros((1,9)) )
        self.cinf.P = rospy.get_param( '/camera/projection_matrix/data', default=np.zeros((1,12)) )
        self.cinf.binning_x = 0
        self.cinf.binning_y = 0
        self.info_pub = rospy.Publisher( top.camera_info, CameraInfo, queue_size=1 )

        # Create object to capture image
        self.cam = cv2.VideoCapture( 0 ) # The argument '0' gets the default webcam

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Node is publishing to the video_frames topic using the message type Image
        self.pub = rospy.Publisher( top.camera_image_raw, Image, queue_size=1 )

        self.publish_video()

    #================================================
    def publish_video( self ):
        ''' Main while loop to publish video data'''

        while not rospy.is_shutdown():
            # Record image
            image_captured, image = self.cam.read()

            if image_captured == True:

                if IS_WANDA:
                    # The camera is mounted upside down in the seeker
                    # Flip the image around
                    image = cv2.rotate( image, cv2.ROTATE_180 )

                ( rows, cols, channels ) = image.shape

                # Publish the image
                header_stamp = rospy.Time.now()
                image_message = self.br.cv2_to_imgmsg( image, encoding=self.image_encoding )
                image_message.header.stamp = header_stamp
                image_message.header.frame_id = frame.camera
                image_message.encoding = self.image_encoding
                image_message.height = rows # Include size of the image
                image_message.width = cols
                self.pub.publish( image_message )

                # Send camera info out as a topic
                self.cinf.header.stamp = header_stamp
                self.cinf.height = rows
                self.cinf.width = cols
                self.info_pub.publish( self.cinf )

            else:
                rospy.logwarn( f"Could not capture image frame at time = {rospy.Time.now()}" )

            # Sleep just enough to maintain the desired rate
            self.rate.sleep()

#==============================================================================
if __name__ == '__main__':
    try:
        cam_node = camera_image_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass