#!/usr/bin/env python3
#=====================
from geometry_msgs.msg import Transform

#=======================================
def identity_pose():

    pose = Transform()

    # Clear out translation
    pose.translation.x = 0.0
    pose.translation.y = 0.0
    pose.translation.z = 0.0

    # Identity orientation
    pose.rotation.x = 0.0
    pose.rotation.y = 0.0
    pose.rotation.z = 0.0
    pose.rotation.w = 1.0

    return pose

#==============================================================================
if __name__ == "__main__":
    print( identity_pose() )
