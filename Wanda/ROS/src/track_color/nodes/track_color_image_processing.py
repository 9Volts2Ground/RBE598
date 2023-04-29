#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image # Image is the message type

# Custom libraries
from topic_messages import topics
from track_color.msg import target_states

#----------------------------------------
# Set global flags and class instances
top = topics.topics()

#==============================================================================
class color():
    ''' Class for containing HSV color ranges
    '''
    def __init__( self, low=[], high=[], color='' ):
        self.high = high
        self.low = low
        self.color = color

# define the color ranges
green = color( low = np.array([29, 86, 20]), high = np.array([64, 255, 255]), color='green' )
blue = color( low = np.array([60, 100, 40]), high=np.array([150, 255, 255]), color='blue')
red = color( low=np.array([160, 100, 50]), high=np.array([180, 255, 255]), color='red')

#==============================================================================
class track_color_image_processing():
    def __init__(self):
        # Initialize ROS communication
        rospy.init_node( "track_color_image_processing", anonymous=True)

        # Set up publishers
        self.pub = rospy.Publisher( top.target_image_track, target_states, queue_size = 1 )
        self.image_pub = rospy.Publisher( top.camera_target_track, Image, queue_size = 1 )

        # Initialize class to convert between ROS image topic and cv2
        self.br = CvBridge()
        self.image_encoding = "bgr8"

        self.detection_radius = 50

        rospy.Subscriber( top.camera_image_raw, Image, self.process_image)

    #==========================================================================
    def process_image(self, image_topic):

        color = blue
        # color = red

        # Convert camera data to a cv2 object
        frame = self.br.imgmsg_to_cv2( image_topic )

        # Initialize target state message
        tgt_state = target_states()
        tgt_state.header.stamp = rospy.Time.now()
        tgt_state.camera_width = image_topic.width
        tgt_state.camera_height = image_topic.height

        # Blur the image, convert it to the HSV color space
        blurred = cv2.GaussianBlur(frame, (15, 15), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange( hsv, color.low, color.high) # Grab initial filter for desired color
        mask = cv2.erode( mask, None, iterations=2 ) # First pass of filtering
        mask = cv2.dilate( mask, None, iterations=2 )

        # find contours in the mask
        contours, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # only proceed if at least one contour was found
        if np.size( contours ):

            # find the largest contour in the mask, then use it to compute
            # the minimum enclosing circle and centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            # only draw the enclosing circle and text if the radious meets
            # a minimum size
            if radius >= self.detection_radius:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)

                M = cv2.moments(c)
                (cX, cY) = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                cv2.putText(frame, color.color, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

                tgt_state.target_found = True
                tgt_state.target_position.x = cX
                tgt_state.target_position.y = cY

        # Publish simple topic with detected target state info
        self.pub.publish( tgt_state )

        # Publish processed image so we can scope it using other ROS tools
        ( rows, cols, channels ) = frame.shape
        image_message = self.br.cv2_to_imgmsg( frame, encoding=self.image_encoding )
        image_message.header.stamp = rospy.Time.now()
        image_message.encoding = self.image_encoding
        image_message.height = rows # Include size of the image
        image_message.width = cols
        self.image_pub.publish( image_message )


#==============================================================================
if __name__ == '__main__':
    try:
        ps = track_color_image_processing()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

