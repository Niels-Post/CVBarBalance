import cv2
import imutils
image = cv2.imread("test-pattern-tv.jpg")
image = imutils.resize(image, width=400)
message = "OpenCV Jetson Nano Success!"
font = cv2.FONT_HERSHEY_SIMPLEX
_ = cv2.putText(image, message, (30, 130), font, 0.7, (0, 255, 0), 2)
cv2.imshow("Penguins", image); cv2.waitKey(0); cv2.destroyAllWindows()