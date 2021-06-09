from CameraBallFinder import CameraBallFinder
from Motor import Motor
from PID import PID


class NewBarBalance:
    """
        Bar balancer interface.

        When running this demo:

        - Motor setup: Through the command line interface, the user is asked to make sure the bar is level.
            By giving commands, the bar can be adjusted. The following commands are available:
                - `s`   Confirm the bar is level, motor setup ends
                - `r`   Turn the bar to the right
                - `rr`  Turn the bar to the right slightly
                - `l`   Turn the bar left
                - `ll`  Turn the bar left slightly


        - Bar setup:
            The user marks where the bar is in the screen by clicking the left end of the bar, followed by the right end.

        - Balancing:
            The demo balances the ball on the center of the bar

        - Setpoint:
            The user can click a new setpoint in the camera view, after which, the demo will balance the ball on this setpoint.
            The user can keep doing this until quit:

        - Quit:
            The user presses 'q'. The system relevels the bar and shuts down

    """

    def __init__(self):
        """
        Construct required classes and perform some setup (e.q. run the levelling process for the bar)
        """
        self.ballfinder = CameraBallFinder((175, 230, 180), (10,20,80))

        self.motor = Motor()

        self.pid = PID(0.12, 0.0, 0.08)

        self.ballfinder.update()

        self.motor.setup()

    def update(self):
        """
        Update-method, checks if a new setpoint was clicked, otherwise just performs PID balancing
        :return:
        """
        try:

            set_point, ball_position = self.ballfinder.update()

            self.pid.update_setpoint(set_point)

            steering_action = self.pid.get(ball_position)

            if abs(steering_action) > 200:
                raise RuntimeError("Steering action too agressive. Abort!")

            self.motor.move_to(steering_action)
        except RuntimeError:
            self.motor.move_to(0)
            exit()