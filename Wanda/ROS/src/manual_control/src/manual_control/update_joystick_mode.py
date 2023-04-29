#!/usr/bin/env python3
from manual_control.stick_mode import stick_mode

#======================================================
def update_joystick_mode( controller,
                          controller_previous,
                          r_stick_mode,
                          l_stick_mode ):
    ''' Check the shouler bumpers to switch joystick modes.'''
    if controller.r_stick.down == 1 and controller_previous.r_stick.down == 0:
        r_stick_mode = toggle_joystick_mode( r_stick_mode )
    if controller.l_stick.down == 1 and controller_previous.l_stick.down == 0:
        l_stick_mode = toggle_joystick_mode( l_stick_mode )

    return r_stick_mode, l_stick_mode

#======================================================
def toggle_joystick_mode( mode ):
    if mode == stick_mode.WALK:
        mode = stick_mode.POSE
    else:
        mode = stick_mode.WALK
    return mode
