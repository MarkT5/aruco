import cv2
from cv2 import aruco
import time
import numpy as np

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
from camera_props import *

cap = cv2.VideoCapture("/dev/video0")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 448)
cap.set(cv2.CAP_PROP_FPS, 30)
t = time.time()
counter = 0
object_points = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0]])
try:
    while True:
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        image = cv2.putText(frame_markers, str(time.time()-t), (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (255,255,255), 2, cv2.LINE_AA)

        if corners:
            for c in corners:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.03, camera_matrix, distortion_coefficient)
                image = cv2.drawFrameAxes(image, camera_matrix, distortion_coefficient, rvec, tvec, 0.03 / 2)
        cv2.imshow('frame', frame_markers)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('s'):
            cv2.imwrite(f"latency_test_{counter}.jpg", frame_markers)
    cv2.destroyWindow('frame')
    cap.release()
except KeyboardInterrupt:
    cv2.destroyWindow('frame')
    cap.release()