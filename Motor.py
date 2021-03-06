import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)


class Pin:
    """
    Simple pin class for the Raspberry pi or equivalent
    """

    def __init__(self, nr):
        self.pin_nr = nr
        self.last_state = False
        GPIO.setup(nr, GPIO.OUT)
        GPIO.output(nr, GPIO.LOW)

    def on(self):
        """
        Turn on the pin
        """
        GPIO.output(self.pin_nr, GPIO.HIGH)
        self.last_state = True

    def off(self):
        """
        Turn off the pin
        """
        GPIO.output(self.pin_nr, GPIO.LOW)
        self.last_state = False

    def toggle(self):
        """
        Toggle the state of the pin (if off->set to on, if on-> set to on)
        """
        self.last_state = not self.last_state
        GPIO.output(self.pin_nr, GPIO.HIGH if self.last_state else GPIO.LOW)


class Motor:
    """
    Driver class for a simple stepper motor.
    Any stepper motor which needs just a STEP and a DIR pin should work.
    """

    def __init__(self):
        self.dir = Pin(9)
        self.step = Pin(25)
        self.rst_slp1 = Pin(11)
        self.rst_slp2 = Pin(8)
        self.rst_slp1.on()
        self.rst_slp2.on()
        self.relative_distance = 0

        self.max_speed = 0.0012
        self.ramp_steps = 5

    def set(self):
        """
        Set the current position to be the motor's zero point
        """
        self.relative_distance = 0

    def delay(self, curr_steps, steps):
        """
        Method to get the delay for the step pin.
        Currently just returns the max speed, but should interpolate between a min and max speed for a ramp-up/ramp-down curve.
        :return: The calculated delay
        """
        return self.max_speed

    def move(self, steps, dir):
        """
        Move the motor a number of steps in the specified direction
        """
        self.dir.on() if dir else self.dir.off()

        for i in range(steps):
            self.step.toggle()
            time.sleep(self.delay(i, steps))
            self.step.toggle()
            time.sleep(self.delay(i, steps))

        self.relative_distance += -steps if dir else steps

    def move_to(self, relative_steps):
        """
        Move the motor to a position relative to its setpoint
        """
        diff = relative_steps - self.relative_distance
        self.move(abs(diff), diff < 0)

    def setup(self):
        """
        Setup routine for the motor. Can be used to determine the setpoint of the motor before starting an application
        :return:
        """
        while True:
            q = input("Left (l), Right (r) or set(s)?")

            if q == "l":
                self.move(10, True)
            elif q == "r":
                self.move(10, False)
            elif q == "ll":
                self.move(1, True)
            elif q == "rr":
                self.move(1, False)
            elif q == "s":
                self.set()
                break
