import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)

class Pin:
    def __init__(self, nr):
        self.pin_nr = nr
        self.last_state = False
        GPIO.setup(nr, GPIO.OUT)
        GPIO.output(nr, GPIO.LOW)

    def on(self):
        GPIO.output(self.pin_nr, GPIO.HIGH)
        self.last_state = True

    def off(self):
        GPIO.output(self.pin_nr, GPIO.LOW)
        self.last_state = False

    def toggle(self):
        self.last_state = not self.last_state
        GPIO.output(self.pin_nr, GPIO.HIGH if self.last_state else GPIO.LOW)

class Motor:
    def __init__(self):
        self.dir = Pin(9)
        self.step = Pin(25)
        self.rst_slp1 = Pin(11)
        self.rst_slp2 = Pin(8)
        self.rst_slp1.on()
        self.rst_slp2.on()
        self.relative_distance = 0


        self.min_speed = 0.0012
        self.max_speed = 0.0012
        self.ramp_steps = 5

    def set(self):
        self.relative_distance = 0

    def stop(self):
        pass

    def delay(self, curr_steps, steps):
        if curr_steps < self.ramp_steps:
            return self.min_speed - (self.min_speed - self.max_speed) / self.ramp_steps * curr_steps
        if steps - curr_steps < self.ramp_steps:
            return self.min_speed - (self.min_speed - self.max_speed) / self.ramp_steps * (steps - curr_steps)

        return self.max_speed

    def move(self, steps, dir):
        self.dir.on() if dir else self.dir.off()

        for i in range(steps):
            self.step.toggle()
            time.sleep(self.delay(i, steps))
            self.step.toggle()
            time.sleep(self.delay(i, steps))

        self.relative_distance += -steps if dir else steps

    def move_to(self, relative_steps):
        diff = relative_steps - self.relative_distance
        self.move(abs(diff), diff < 0)

    def setup(self):
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
            elif q =="s":
                self.set()
                break





if __name__ == '__main__':
    test = Motor()

    try:
        while True:
            q = input("Left (l), Right (r) or set(s)?")

            if q == "l":
                test.move(2, True)
            elif q == "r":
                test.move(2, False)
            elif q =="s":
                test.set()
                break


        while True:
            q = int(input("setpoint or quit(q)?"))
            if q == "q":
                break
            test.move_to(q)
    except InterruptedError:
        pass
    test.stop()