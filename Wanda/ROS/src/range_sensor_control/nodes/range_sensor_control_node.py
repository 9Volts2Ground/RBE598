#!/usr/bin/env python3
#=====================
import actionlib
import rospy
from sensor_msgs.msg import Range

# Custom classes
from range_sensor_control.msg import SensorStateAction, SensorStateGoal
from range_sensor_control.range_sensor_client import range_sensor_client
from topic_messages import topics

top = topics.topics()

#====================================================================
class range_sensor_control_node():
    def __init__(self):
        rospy.init_node( "range_sensor_control_node", anonymous=True )

        # Initialize state topics
        self.range_state = SensorStateGoal()

        # Let users choose the initial state
        self.range_state.state = rospy.get_param( f"{rospy.get_name()}/init_state", default=SensorStateGoal.INACTIVE )

        # Set up an action server to grab the desired sensor state
        self.server = actionlib.SimpleActionServer( 'range_sensor_state_server',
                                                    SensorStateAction,
                                                    self.sensor_state_action,
                                                    auto_start=False )
        self.server.start()

        self.pub = rospy.Publisher( top.range_data_raw, Range, queue_size=10 )

        self.range_sensor_control()

    #======================================================
    def range_sensor_control(self):
        rate = rospy.Rate( 20 ) # Hz

        while not rospy.is_shutdown():

            if self.range_state.state == SensorStateGoal.ACTIVE:
                # Ping for range measurement, publish data
                range, returned = range_sensor_client()

                if returned:
                    self.pub.publish( range )

            rate.sleep()

    #======================================================
    def sensor_state_action(self, sensor_state_goal ):

        self.range_state.state = sensor_state_goal.state
        self.server.set_succeeded()

#==============================================================================
if __name__ == "__main__":
    try:
        rs = range_sensor_control_node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
