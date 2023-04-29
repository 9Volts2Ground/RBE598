#!/usr/bin/env python3
import numpy as np
import os
import psutil
import rospy
from sensor_msgs.msg import Temperature

from topic_messages import topics

top = topics.topics()

#==============================================================================
class cpu_temperature_node():
    def __init__(self):
        rospy.init_node( 'cpu_temperature_node', anonymous=True )

        self.IS_WANDA = rospy.get_param( 'IS_WANDA', default=False )

        # Set flags to check temperatures if we are not on a RPi4
        if not self.IS_WANDA:
            try:
                temp = self.get_chromebook_temp()
                self.is_chromebook = True
            except:
                self.is_chromebook = False

        # How fast to publish the data. Should be slow, not urgent
        self.rate = rospy.Rate( 1.0/5.0 )

        # Temperature topic
        self.temperature = Temperature()
        self.temperature.temperature = 0.0

        # Logging thresholds
        self.warn_threshold = 75
        self.error_threshold = 85

        self.pub = rospy.Publisher( top.cpu_temp, Temperature, queue_size=1 )

    #================================================
    def publish_cpu_temp( self ):

        while not rospy.is_shutdown():
            # Grab system temperature
            if self.IS_WANDA:
                temp = self.get_wanda_temp()
            elif self.is_chromebook:
                temp = self.get_chromebook_temp()
            else:
                temp = np.nan

            # Send out the data as a topic
            self.temperature.temperature = temp
            self.temperature.header.stamp = rospy.Time.now()
            self.pub.publish( self.temperature )

            self.print_logs( temp )


            # Wait to loop again
            self.rate.sleep()

    #================================================
    def print_logs( self, temp ):
        '''Pring logs if the temperature gets too hot'''
        if temp >= self.error_threshold:
            rospy.logerr(f"CPU temperature is dangerously hot: {temp} C")
        elif temp >= self.warn_threshold:
            rospy.logwarn(f"CPU temperature is concerningly hot: {temp} C")

    #================================================
    def get_wanda_temp( self ):
        temp = os.popen("vcgencmd measure_temp").readline()
        temp = temp.replace("temp=","")
        temp = temp.replace("'C","")
        return float(temp)

    #================================================
    def get_chromebook_temp( self ):
        ''' This approach works on the Chromebook. May not run on other machines'''
        return float( psutil.sensors_temperatures()['coretemp'][0][1] )

#==============================================================================
if __name__ == '__main__':
    try:
        cpu_tmp = cpu_temperature_node()
        cpu_tmp.publish_cpu_temp()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
