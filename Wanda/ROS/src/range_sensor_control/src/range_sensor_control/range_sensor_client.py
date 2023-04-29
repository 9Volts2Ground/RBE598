#!/usr/bin/env python3
import rospy

# Range sensor client service
from hardware_control.init_range_measurement import init_range_measurement
from hardware_control.srv import ultrasonic_range, ultrasonic_rangeResponse

#==============================================================================
def range_sensor_client():
    rospy.wait_for_service( 'ultrasonic_range_sensor' )
    try:
        server_handle = rospy.ServiceProxy( 'ultrasonic_range_sensor', ultrasonic_range )
        rng = server_handle()
        return rng.range, True # Just send the sensor_msgs.msg/range component
    except rospy.ServiceException as e:
        rospy.logerr("Failed to call ultrasonic_range_sensor: %s" %e)

        # Setup default range measurement to return
        rng = init_range_measurement()
        rng.header.stamp = rospy.Time.now()
        return rng.range, False
#==============================================================================
if __name__ == "__main__":

    range, success = range_sensor_client()

    if success:
        print(range)
    else:
        print("Failed to receive range measurement")