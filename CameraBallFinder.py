import math
from typing import Tuple
import cv2
import numpy as np

from SimpleMovingAverage import SimpleMovingAverage, SMA_IntCoordinateAverage


class CameraBallFinder:
    """
    Class to help find a ball on the camera.
    This class takes care of all computer vision tasks:

    - Find the ball on screen using its color
    - Use a moving average filter on the ball position to remove noise introduced by camera imperfections and shake
    - Calculate the position of the ball relative to the balance-bar and calculate a one dimensional coordinate for it.
    - Allow the user to mark the balance-bar in a live view
    - Allow the user to mark a setpoint in a live view
    - Provide live views of both the camera input (with markers for important positions) and the filtered output for finding the ball

    """
    def __init__(self, ball_color: Tuple[int, int, int], color_tolerance: Tuple[int, int, int], moving_average_length=3,
                 show_filtered_output=True):
        """
        Setup the cameraballfinder. Sets up a video capture device and output windows
        :param ball_color: Approximate color of the ball to find
        :param color_tolerance: How much can the color of the ball differ per color parameter
        :param moving_average_length: How long should the moving average used be when filtering the ball position
        :param show_filtered_output: Should a live view of the filtered frame be shown
        """
        self.capture = cv2.VideoCapture(0)

        self.ball_color = np.asarray(ball_color)
        self.color_tolerance = np.asarray(color_tolerance)

        self.show_filtered_output = show_filtered_output

        self.preprocessing_crop = None
        self.preprocessing_rotation_matrix = None
        self.preprocessing_crop_height = 200

        self.nth_click = 0

        self.bar_left = None
        self.bar_right = None
        self.bar_center = None

        self.last_frame = None
        self.last_filtered_frame = None

        self.smoothed_ball_positions = SimpleMovingAverage(moving_average_length, (0, 0), SMA_IntCoordinateAverage)

        self.window_output = "Output"
        self.window_filtered = "Filtered"

        self.set_point = 1


        cv2.namedWindow(self.window_output)
        cv2.setMouseCallback(self.window_output, self.on_click)

        cv2.namedWindow(self.window_filtered)

    def on_click(self, event, x, y, flags, param):
        """
        Capture click events for the main window
        First two clicks are parsed as the positions of the balance bar.
        After that, each click is used as a new setpoint
        """
        if event == cv2.EVENT_LBUTTONUP:
            if self.nth_click == 0:
                self.bar_left = x, y
            elif self.nth_click == 1:
                self.bar_right = x, y
                self.calculate_preprocessing_steps()
                self.set_point = 0
            else:
                self.set_point = self.get_position_on_bar(x, y)
                self.no_steers_left = 10
                self.integral = 0

            self.nth_click += 1

        if event ==cv2.EVENT_MBUTTONUP:
            print(self.last_hsv_frame[y,x])

    def calculate_preprocessing_steps(self):
        """
        Calculate the preprocessing steps needed for centering the image around the bar:
        - A rotation matrix. When the bar is level in real life, it should also be level relative to the image
        - A Crop region around the bar, to make processing easier on the computer hardware

        After this method, the bar_left, bar_right and bar_center coordinates are set relative to the processed image,
        not the original image
        :return:
        """

        ## Calculate Rotation matrix
        bar_dy = abs(self.bar_right[1] - self.bar_left[1])
        bar_dx = abs(self.bar_right[0] - self.bar_left[0])

        bar_angle = (math.atan(bar_dy / bar_dx) * 180 / math.pi)
        rotation_center = tuple(np.average(
            np.asarray((self.bar_right, self.bar_left)), axis=0))  # tuple(np.array(self.last_frame.shape[1::-1]) / 2)

        self.preprocessing_rotation_matrix = cv2.getRotationMatrix2D(rotation_center, bar_angle, 1.0)

        ## Calculate Crop ratio
        new_bar_left = tuple([int(i) for i in np.dot(self.bar_left, self.preprocessing_rotation_matrix)])[:-1]
        new_bar_right = tuple([int(i) for i in np.dot(self.bar_right, self.preprocessing_rotation_matrix)])[:-1]

        self.preprocessing_crop = (
            max(0, new_bar_left[1] - self.preprocessing_crop_height),
            min(new_bar_left[1] + 50, self.last_frame.shape[0]),
            max(0, new_bar_left[0] - 20),
            min(self.last_frame.shape[1], new_bar_right[0] + 20)
        )

        self.preprocess_image()

        ## Set bar positions relative to processed image
        self.bar_left = (20, self.last_frame.shape[0] - 77)
        self.bar_right = (self.last_frame.shape[1] - 20, self.last_frame.shape[0] - 77)
        self.bar_center = (int(self.last_frame.shape[1] / 2), self.last_frame.shape[0] - 77)

    def get_position_on_bar(self, x, y):
        """
        Calculate a 1-dimensional coordinate relative to the bar from an x,y coordinate
        """
        pos = np.linalg.norm(np.array((x, y)) - np.array(self.bar_center))
        if x > self.bar_center[0]:
            pos *= -1
        return pos

    def capture_image(self):
        """
        Capture a frame for the camera
        """
        ret, self.last_frame = self.capture.read()
        if not ret:
            raise RuntimeError("Cannot get frame from Camera")

    def preprocess_image(self):
        """
        Apply the steps calculated in calculate_preprocessing_steps to the image, to center it around the balance-bar
        """
        # Rotate image to max 0 level of the bar
        if self.preprocessing_rotation_matrix is not None:
            self.last_frame = cv2.warpAffine(self.last_frame, self.preprocessing_rotation_matrix,
                                             self.last_frame.shape[1::-1])

        # Crop the image to a region of interest around the bar
        if self.preprocessing_crop is not None:
            ymin, ymax, xmin, xmax = self.preprocessing_crop
            self.last_frame = self.last_frame[ymin:ymax, xmin:xmax]

        # Generate an HSV image
        self.last_hsv_frame = cv2.cvtColor(self.last_frame, cv2.COLOR_BGR2HSV)

    def locate_ball(self):
        """
        Filter the preprocessed frame, and find the ball in it

        This method only finds the position of the ball in the frame, not relative to the balance bar
        """
        self.last_filtered_frame = cv2.inRange(self.last_hsv_frame, self.ball_color - self.color_tolerance,
                                               self.ball_color + self.color_tolerance)

        ball_ctr = np.average(np.argwhere(self.last_filtered_frame), axis=0)

        if np.any(np.isnan(ball_ctr)):
            return

        self.smoothed_ball_positions.add(ball_ctr)

    def show_windows(self):
        """
        Draw important positions on the live view, and show both the live view and the filtered live view
        :return:
        """
        ball_pos = self.smoothed_ball_positions.get()
        cv2.circle(self.last_frame, (ball_pos[1], ball_pos[0]), 2, (255, 0, 0), 2)
        cv2.circle(self.last_frame, self.bar_center, 2, (0, 255, 0), 2)
        cv2.circle(self.last_frame, self.bar_left, 2, (0, 255, 0), 2)
        cv2.circle(self.last_frame, self.bar_right, 2, (0, 255, 0), 2)

        cv2.imshow(self.window_output, self.last_frame)

        if self.show_filtered_output and self.last_filtered_frame is not None:
            cv2.imshow(self.window_filtered, self.last_filtered_frame)

    def check_windowevents(self):
        """
        Check if 'q' was pressed to quit the windows.
        :return:
        """
        key = cv2.waitKey(1) & 0xFF
        #
        if key == ord("q"):
            cv2.destroyAllWindows()
            self.capture.release()
            raise RuntimeError("Quit Pressed")

    def get_ball_position(self):
        """
        Find the position of the ball on the bar, using the moving average queue.
        :return:
        """
        if self.bar_center is None:
            return 0
        pos = self.smoothed_ball_positions.get()
        return self.get_position_on_bar(pos[1], pos[0])

    def update(self):
        """
        Capture an image, preprocess it and find the ball in it using this class' methods
        """
        self.capture_image()
        self.preprocess_image()

        self.locate_ball()
        self.show_windows()
        self.check_windowevents()

        return self.set_point, self.get_ball_position()
