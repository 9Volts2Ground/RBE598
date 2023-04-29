#!/usr/bin/env python3
import rospy
#================================================
# Get global parameters
IS_WANDA = rospy.get_param( 'IS_WANDA', default=False )
if IS_WANDA:
    from rpi_ws281x import Adafruit_NeoPixel, Color

# Note that the '&' operator is considered a 'Bitwise AND'
# Basically, it's a modulo that includes the mod value as itself (instead of 0)
# For comparison
#        &                   %
# 254 & 255 = 254  |  254 % 255 = 254
# 255 & 255 = 255  |  255 % 255 = 0
# 256 & 255 = 0    |  256 % 255 = 1

# Note that >> and << are considered 'Shifts'

# LED strip configuration:
LED_COUNT      = 7       # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

# Define functions which animate LEDs in various ways.
class Led:
    # =========================================================================
    def __init__(self):
        # Possible values of LedMod are 0, 1, 2, 3, 4, 5
        # 0 = calls def colorWipe and resets to Color(0,0,0)
        # 1 = calls def ledIndex
        # 2 = calls def colorWip and resets to RGB colors (combos of 255 and 0)
        # 3 = calls def theaterChase
        # 4 = calls def rainbow
        # 5 = calls def rainbowCycle
        self.LedMod = '1'
        self.colour = [0, 0, 0]
        self.ORDER = 'RGB' # Control the sending order of color data

        if IS_WANDA:
            # Create NeoPixel object with appropriate configuration.
            self.strip = Adafruit_NeoPixel( LED_COUNT,
                                            LED_PIN,
                                            LED_FREQ_HZ,
                                            LED_DMA,
                                            LED_INVERT,
                                            LED_BRIGHTNESS,
                                            LED_CHANNEL )
            # Intialize the library (must be called once before other functions).
            self.strip.begin()

    # =========================================================================
    def LED_TYPR(self, order, R_G_B):
        """
        Converts Color object to desired order of RGB.

        Parameters
        ----------
        order : string
            Sending order of color data, i.e. 'RGB'
        R_G_B : Color object
            Color object with associated values

        Returns
        -------
        color : Color object
            Color object with RGB values in designated order (given by input
            variable order)
        """
        R = R_G_B >> 16 & 255
        G = R_G_B >> 8 & 255
        B = R_G_B & 255
        Led_type = ['GRB', 'GBR', 'RGB', 'RBG', 'BRG', 'BGR']
        color = [Color(G,R,B),
                 Color(G,B,R),
                 Color(R,G,B),
                 Color(R,B,G),
                 Color(B,R,G),
                 Color(B,G,R) ]
        if order in Led_type:
            return color[Led_type.index(order)]

    # =========================================================================
    def colorWipe(self, strip, color, wait_ms=50):
        """
        Wipe color across display a pixel at a time.

        Cycles through each light on the strip and assigns the variable
        'color' to each

        Parameters
        ----------
        strip : Adafruit_NeoPixel object (the light strip  object)
        color : Color object
            Each light gets assigned this Color value
        wait_ms : integer (or float)
            Wait time in microseconds; default 50
        """
        color = self.LED_TYPR( self.ORDER, color )
        for i in range(self.strip.numPixels()):
            # Only command color if running on hardware
            if IS_WANDA:
                self.strip.setPixelColor(i, color)
                self.strip.show()
            rospy.sleep(wait_ms/1000.0)

    # =========================================================================
    def theaterChase(self, strip, color, wait_ms=50, iterations=10):
        """
        Movie theater light style chaser animation.

        Currently set to leave 2 lights off between lights that are on

        Parameters
        ----------
        strip : Adafruit_NeoPixel object (the light strip)
        color : Color object
        wait_ms : integer (or float)
            Wait time in microseconds; default 50
        iterations : integer
            Number of times to do the 'animation'; default 10
        """
        color = self.LED_TYPR(self.ORDER, color)
        for j in range(iterations):
            # There's a range(3) here because 3 is the designated skip value
            # in the following (interior) for-loops
            # Say you have 8 lights. First loop over q turns these lights on and off
            # X     X     X
            # 0 1 2 3 4 5 6 7
            # Second loop over q turns these lights on and off
            #   X     X     X
            # 0 1 2 3 4 5 6 7
            # And the third loop over q turns these lights on and off
            #     X     X
            # 0 1 2 3 4 5 6 7
            for q in range(3):
                # Turns lights on
                if IS_WANDA:
                    for i in range(0, self.strip.numPixels(), 3):
                        self.strip.setPixelColor(i+q, color)
                    self.strip.show()
                rospy.sleep(wait_ms/1000.0)

                # Turns lights off
                if IS_WANDA:
                    for i in range(0, self.strip.numPixels(), 3):
                        self.strip.setPixelColor(i+q, 0)

    #=========================================================================
    def wheel(self, pos):
        """
        Generates rainbow colors across 0-255 positions.

        Called by def rainbow() and def rainbowCycle()

        Parameters
        ----------
        pos : integer

        Returns
        -------
        Unnamed Color object
        """
        if pos < 0 or pos > 255:
            r = 0
            g = 0
            b = 0
        # Note that 255/3 = 85 (3 for RGB)
        elif pos < 85:
            r = pos * 3
            g = 255 - pos * 3
            b = 0
        elif pos < 170:
            pos -= 85
            r = 255 - pos * 3
            g = 0
            b = pos * 3
        else:
            pos -= 170
            r = 0
            g = pos * 3
            b = 255 - pos * 3
        return self.LED_TYPR(self.ORDER, Color(r,g,b))

    # =========================================================================
    def rainbow(self, strip, wait_ms=20, iterations=1):
        """
        Draw rainbow that fades across all pixels at once.

        All pixels go through the following RGB in this order:
            0   255 0
            3   252 0
            6   249 0
            ...
            249 6   0
            252 3   0
            255 0   0
            252 0   3
            249 0   6
            ...
            6   0   249
            3   0   252
            0   0   255
            0   3   252
            0   6   249
            ...
            0   255 0
        R goes from 0 to 255 and back to 0 in skip 3, then holds
        G goes from 255 to 0 in skip 3, holds, then goes 0 to 255
        B holds at 0 then goes from 0 to 255 and back to 0 in skip 3

        Parameters
        ----------
        strip : Adafruit_NeoPixel object (the light strip)
        wait_ms : integer (or float)
            Wait time in microseconds
        iterations : integer
            Number of times to do the 'animation'; default 1
        """
        for j in range(256*iterations):
            if IS_WANDA:
                for i in range(self.strip.numPixels()):
                    self.strip.setPixelColor(i, self.wheel((i+j) & 255))
                self.strip.show()
            rospy.sleep(wait_ms/1000.0)

    # =========================================================================
    def rainbowCycle(self, strip, wait_ms=20, iterations=5):
        """
        Draw rainbow that uniformly distributes itself across all pixels.

        Parameters
        ----------
        strip : Adafruit_NeoPixel object (the light strip)
        wait_ms : integer (or float)
            Wait time in microseconds
        iterations : integer
            Number of times to do the 'animation'; default 5
        """
        for j in range(256*iterations):
            if IS_WANDA:
                for i in range(self.strip.numPixels()):
                    self.strip.setPixelColor(i, self.wheel((int(i * 256 / self.strip.numPixels()) + j) & 255))
                self.strip.show()
            rospy.sleep(wait_ms/1000.0)

    # =========================================================================
    def theaterChase(self, strip, data, wait_ms=50):
        """
        Movie theater light style chaser animation (1 iteration).

        Note that this is a repeat definition name!

        Parameters
        ----------
        strip : Adafruit_NeoPixel object (the light strip)
            DESCRIPTION.
        data : array/list
            The array has 3 values: R, G, and B
        wait_ms : integer (or float)
            Wait time in microseconds; default 50.
        """
        for q in range(3):
            if IS_WANDA:
                for i in range(0, self.strip.numPixels(), 3):
                    self.strip.setPixelColor(i+q, Color(data[0], data[1], data[2]))
                self.strip.show()
            rospy.sleep(wait_ms/1000.0)
            if IS_WANDA:
                for i in range(0, strip.numPixels(), 3):
                    strip.setPixelColor(i+q, 0)

    # =========================================================================
    def ledIndex(self, index, R, G, B):
        """
        Description here...

        Parameters
        ----------
        index : integer
            DESCRIPTION.
        R : integer
            Red color value
        G : integer
            Green color value
        B : integer
            Blue color value
        """
        color = self.LED_TYPR(self.ORDER, Color(R,G,B))
        if IS_WANDA:
            for i in range(self.strip.numPixels()):  # previously for i in range(8)
                if index & 0x01 == 1:  # 0 if index is even, 1 if index is odd
                    self.strip.setPixelColor(i, color)
                index = index >> 1
            self.strip.show()

    # =========================================================================
    def light(self, data):
        """
        Description here...

        Parameters
        ----------
        data : TYPE
            DESCRIPTION.
        """
        oldMod = self.LedMod
        if len(data) < 4:
            self.LedMod = data[1]
        else:
            for i in range(3):
                self.colour[i] = int(data[i+1])

        if self.LedMod == '0':
            self.colorWipe(self.strip, Color(0,0,0))
            self.LedMod = oldMod
        elif self.LedMod == '1':
            self.ledIndex(255, self.colour[0], self.colour[1], self.colour[2])
        elif self.LedMod == '2':
            while True:
                self.colorWipe(self.strip, Color(255, 0, 0))  # Red wipe
                self.colorWipe(self.strip, Color(0, 255, 0))  # Green wipe
                self.colorWipe(self.strip, Color(0, 0, 255))  # Blue wipe
        elif self.LedMod == '3':
            while True:
                self.theaterChase(self.strip, self.colour)
        elif self.LedMod == '4':
            while True:
                self.rainbow(self.strip)
        elif self.LedMod == '5':
            while True:
                self.rainbowCycle(self.strip)

    # =========================================================================
    def setColor(self, pixel, colors):
        """
        Control individual LED pixel colors.

        Parameters
        ----------
        pixel - which pixel to light up, 0:6
        color - array with color brightness, [R,G,B] as ints
        """
        R = int( colors[0] )
        G = int( colors[1] )
        B = int( colors[2] )
        color = self.LED_TYPR(self.ORDER, Color(R,G,B))

        if IS_WANDA:
            self.strip.setPixelColor(pixel, color)
            self.strip.show()


# Main program logic follows:
if __name__ == '__main__':
    pass
