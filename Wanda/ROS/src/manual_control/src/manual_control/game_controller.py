#!/usr/bin/env python3

#==============================================================================
class joy_stick():
    def __init__(self):
        ''' Define directions the joystick can go'''
        self.fwd_back = 0.0
        self.right_left = 0.0
        self.down = 0 # Center click joy stick

#==============================================================================
class game_controller():
    def __init__(self):
        self.l_stick = joy_stick()
        self.r_stick = joy_stick()
        self.d_pad = joy_stick()
        self.r_trigger = 1.0
        self.l_trigger = 1.0

        # Buttons
        self.a = 0
        self.b = 0
        self.x = 0
        self.y = 0
        self.r_shoulder = 0
        self.l_shoulder = 0
        self.center = 0
        self.back = 0 # Left control button
        self.home = 0 # Right control button

    #----------------------------------------------------------
    def map_joy_to_controller( self, joy ):
        ''' Unpackage the /joy topic to something useful'''
        self.l_stick.fwd_back = joy.axes[1]
        self.l_stick.right_left = -joy.axes[0]
        self.l_stick.down = joy.buttons[9]

        self.r_stick.fwd_back = joy.axes[4]
        self.r_stick.right_left = -joy.axes[3]
        self.r_stick.down = joy.buttons[10]

        self.d_pad.fwd_back = joy.axes[7]
        self.d_pad.right_left = -joy.axes[6]

        self.r_trigger = joy.axes[5]
        self.l_trigger = joy.axes[2]

        self.a = joy.buttons[0]
        self.b = joy.buttons[1]
        self.x = joy.buttons[2]
        self.y = joy.buttons[3]

        self.r_shoulder = joy.buttons[5]
        self.l_shoulder = joy.buttons[4]

        self.center = joy.buttons[8]
        self.back = joy.buttons[6]
        self.home = joy.buttons[7]