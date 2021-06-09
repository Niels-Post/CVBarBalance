import time


class PID:
    """
    Simple PID controller class.

    This class has no application specific information, and should be usable for most PID purposes where the values are single numerical values.

    The setpoint can be changed using the upodate_setpoint method.
    When it is changed, the derivative part of the method is ignored for 1 cycle to prevent a massive steering action when the
    new setpoint is far away from the object position
    """
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.proportional = 0
        self.integral = 0
        self.derivative = 0

        self.previous_error = 0

        self.last_time = time.time()

        self.setpoint_changed = False

        self.set_point = 0

    def get(self, current_position):
        """
        Calculate the steering action using the current position of the measured object
        :param current_position:
        :return:
        """
        dt = time.time() - self.last_time
        self.last_time = time.time()

        current_error = int(self.set_point - current_position)

        self.proportional = current_error
        self.integral = self.integral + current_error * dt
        self.derivative = (current_error - self.previous_error) / dt

        if self.setpoint_changed:
            self.derivative = 0
            self.setpoint_changed = False

        output = self.kp * self.proportional + self.ki * self.integral + self.kd * self.derivative

        self.previous_error = current_error

        print("Kp: %2f, Ki: %2f, Kd: %2f, total: %2f" % (
            self.kp * self.proportional, self.ki * self.integral, self.kd * self.derivative, output))

        print(output)
        return int(output)

    def update_setpoint(self, newpoint):
        """
        Change the setpoint
        """
        if newpoint != self.set_point:
            self.set_point = newpoint
            self.setpoint_changed = True
