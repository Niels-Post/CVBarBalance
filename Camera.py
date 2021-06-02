import math
import time

import cv2
import numpy as np

from Motor import Motor


class BarBalance:
    def __init__(self, kp, ki, kd):
        self.n_click = 0
        self.capture = cv2.VideoCapture(0)
        self.window_name = "BarRegel"

        self.motor = Motor()
        self.motor.setup()

        self.bar_left = 0, 0
        self.bar_right = 0, 0
        self.bar_left_dst = 0, 0
        self.bar_right_dst = 0, 0
        self.bar_center_dst = 0, 0
        self.ball_color = np.array((175, 167, 125))
        self.ball_color_tolerance = np.array((10, 20, 125))

        self.bar_detect_height = 200

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.ball_pos = 0
        self.ball_pos_moving_avg = [(0,0)]*3
        self.ball_pos_moving_avg_idx = 0

        self.set_point = 0

        self.elapsed_time = time.time()
        self.integral = 0
        self.previous_error = 0

        self.no_steers_left = 0


        if not self.capture.isOpened():
            raise RuntimeError("Cannot connect to camera")

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.on_click)
        cv2.namedWindow("Processed")

    def on_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONUP:
            if self.n_click == 0:
                self.bar_left = x, y
            elif self.n_click == 1:
                self.bar_right = x, y
                dy = abs(self.bar_right[1] - self.bar_left[1])
                dx = abs(self.bar_right[0] - self.bar_left[0])

                self.bar_angle = -(math.atan(dy / dx) * 180 / math.pi)
                image_center = tuple(np.array(self.frame.shape[1::-1]) / 2)
                self.rotation_matrix = cv2.getRotationMatrix2D(image_center, -self.bar_angle, 1.0)

                self.bar_left = tuple([int(i) for i in np.dot(self.bar_left, self.rotation_matrix)])[:-1]
                self.bar_right = tuple([int(i) for i in np.dot(self.bar_right, self.rotation_matrix)])[:-1]


            else:
                self.set_point = self.calculate_bar_pos_from_image_pos(x,y)
                print(self.calculate_bar_pos_from_image_pos(x,y))

            self.n_click += 1

    def calculate_bar_pos_from_image_pos(self, x, y):
        pos = np.linalg.norm(np.array((x,y)) - np.array(self.bar_center_dst))
        if x > self.bar_center_dst[0]:
            pos *= -1
        return pos

    def preprocess(self):
        if self.n_click >= 2:
            self.frame = cv2.warpAffine(self.frame, self.rotation_matrix, self.frame.shape[1::-1])

            self.frame = self.frame[max(0, self.bar_left[1] - self.bar_detect_height):min(self.bar_left[1] + 50,
                                                                                          self.frame.shape[0]),
                         max(0, self.bar_left[0] - 20):min(self.frame.shape[1], self.bar_right[0] + 20)]

            self.bar_left_dst = (20, self.frame.shape[0] - 80)
            self.bar_right_dst = (self.frame.shape[1] - 20, self.frame.shape[0] - 80)
            self.bar_center_dst = (int(self.frame.shape[1] / 2), self.frame.shape[0] - 80)

        self.hsv_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

    def process(self):
        self.ball_pos_moving_avg[self.ball_pos_moving_avg_idx] = self.find_ball()

        x_positions = [i[0] for i in self.ball_pos_moving_avg]
        y_positions = [i[1] for i in self.ball_pos_moving_avg]

        ball_pos = (int(sum(x_positions) / len(x_positions)),int(sum(y_positions) / len(y_positions)))
        self.ball_pos_moving_avg_idx = (self.ball_pos_moving_avg_idx + 1) % len(self.ball_pos_moving_avg)

        if self.n_click >= 2:
            cv2.circle(self.frame, ball_pos, 2, (255, 0, 0), 2)
            cv2.circle(self.frame, self.bar_center_dst, 2, (0, 255, 0), 2)
            cv2.circle(self.frame, self.bar_left_dst, 2, (0, 255, 0), 2)
            cv2.circle(self.frame, self.bar_right_dst, 2, (0, 255, 0), 2)

            self.ball_pos = self.calculate_bar_pos_from_image_pos(*ball_pos)

            # print(self.ball_pos_moving_avg)

    def perform_steering(self):
        if self.n_click >= 2:
            dt = time.time() - self.elapsed_time
            self.elapsed_time = time.time()

            error = int((self.set_point - self.ball_pos))
            self.proportional = error
            self.integral = self.integral + error * dt
            self.derivative = (error - self.previous_error) / dt

            output = self.kp * self.proportional + self.ki * self.integral + self.kd * self.derivative
            print("Kp: %2f, Ki: %2f, Kd: %2f, total: %2f" % (
            self.kp * self.proportional, self.ki * self.integral, self.kd * self.derivative, output))
            # print("Ball Pos: " + str(self.ball_pos))
            self.previous_error = error

            if abs(output) > 200:
                self.motor.move_to(0)
                raise RuntimeError("Fek")

            self.motor.move_to(int(output))

    def find_ball(self):
        self.ball_frame = cv2.inRange(self.hsv_frame, self.ball_color - self.ball_color_tolerance,
                                 self.ball_color + self.ball_color_tolerance)
        # self.frame = ball_frame

        ball_ctr = np.average(np.argwhere(self.ball_frame), axis=0)
        # self.frame = ball_frame
        try:
            return int(ball_ctr[1]), int(ball_ctr[0])
        except ValueError:
            return (0,0)

    def poll(self):
        ret, self.frame = self.capture.read()
        self.preprocess()

        self.process()

        self.perform_steering()
        if not ret:
            raise RuntimeError("Cannot get frame")

        cv2.imshow(self.window_name, self.frame)
        if self.ball_frame is not None:
            cv2.imshow("Processed", self.ball_frame)
        key = cv2.waitKey(1) & 0xFF
        #
        if key == ord("q"):
            self.motor.move_to(0)
            cv2.destroyAllWindows()
            self.capture.release()
            exit()


if __name__ == '__main__':
    bar_balance = BarBalance(0.12, 0, 0.015)

    while True:
        bar_balance.poll()
