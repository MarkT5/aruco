import cv2
from cv2 import aruco
import time
import numpy as np


dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)

cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
cap.set(cv2.CAP_PROP_FPS, 30)

#
t = time.time()
counter = 0
ret, frame = cap.read()
flag = False
try:
    while True:
        ret, frame = cap.read()
        if flag:
            cv2.imwrite(f"calibration_{counter}.jpg", frame)
            print("save")
            flag=False
        grayColor = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, thresh1 = cv2.threshold(grayColor, 127, 255, cv2.THRESH_BINARY)
        krn = cv2.getStructuringElement(cv2.MORPH_RECT, (50, 30))
        dlt = cv2.dilate(thresh1, krn, iterations=30)
        res = np.uint8(255 - cv2.bitwise_and(dlt, thresh1))
        cv2.imshow('frame', res)
        key = cv2.waitKey(3)
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('s'):
            flag=True
            counter+=1
    cv2.destroyWindow('frame')
    cap.release()
except KeyboardInterrupt:
    cv2.destroyWindow('frame')
    cap.release()