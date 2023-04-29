#!/usr/bin/env python3
import platform

#==========================================================================
def global_params_init( ):
    ''' Run necessary initialization to set global parameters
    '''
    wanda = is_wanda()

    # Send "running on real robot hardware" flag out to parameter
    print(str(wanda).lower(), end='')

#==========================================================================
def is_wanda( ):
    ''' Checks the CPU architecture this script is running on.
    If it is "aarch64", assume this is a RPi and we are actually
    on the robot hardware. Otherwise, assume this is not the robot.
    '''
    uname = platform.uname()
    if "aarch64" in uname.processor:
        return True
    else:
        return False

#==============================================================================
if __name__ == "__main__":
    global_params_init()
