import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

class PuckDetection(object):
    def __init__(self, 
                 lhsv=[21,0,0], 
                 uhsv=[73,255,255],
                 blur=11):
        self.attr1 = 0
        self.hsv_lower = np.array(lhsv)
        self.hsv_upper = np.array(uhsv)
        self.blur = blur

        

    def detect_puck(self, input_image):
        bridge = CvBridge()
        lower = self.hsv_lower
        upper = self.hsv_upper

        img = bridge.imgmsg_to_cv2(input_image, "bgr8")

        blurred = cv.GaussianBlur(img, (self.blur, self.blur), 0)
        hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, lower, upper)

        kernel = np.ones((5,5), np.uint8)
        mask = cv.erode(mask, kernel, iterations=2)
        mask = cv.dilate(mask, kernel, iterations=2)

        cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0]
        center = None

        if len(cnts) > 0:
            c = max(cnts, key=cv.contourArea)
            ((u, v), radius) = cv.minEnclosingCircle(c)
            M = cv.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if radius > 3:
                cv.circle(img, (int(u), int(v)), int(radius), (0, 255, 255), 2)
                cv.circle(img, center, 5, (0, 0, 255), -1)

        cv.putText(img, "u: {}, v: {}".format(u, v), (10, img.shape[0] - 10),
        cv.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)	
        cv.imshow("Frame", img)
        cv.waitKey(1)

        return u, v