import cv2
from cv2 import aruco
import time
import numpy as np
from homogen import matrixFromVectors
from scipy.spatial.transform import Rotation as Rot
from Robot import Robot

robot = Robot("COM5")

dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(dictionary, parameters)
from camera_props import *

cap = cv2.VideoCapture(0)
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

        matrices = []

        if corners:
            for c in corners:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(c, 0.072, camera_matrix, distortion_coefficient)
                image = cv2.drawFrameAxes(image, camera_matrix, distortion_coefficient, rvec, tvec, 0.08)
                matrices.append(matrixFromVectors(tvec[0, 0], rvec[0, 0]))
        cv2.imshow('frame', frame_markers)
        if ids is not None:
            ids = ids.T[0]
            if 0 in ids and len(matrices) > 1:
                tr = np.linalg.inv(matrices[0]) @ matrices[1]
                #tr = tr/tr[3,3]
                rob_rot = Rot.from_matrix(tr[:3, :3]).as_rotvec()
                robot.movementAllowed = True
                #print(Rot.from_matrix(tr[:3, :3]).as_rotvec())
                robot.currAng = rob_rot[2]
            else:
                robot.movementAllowed = False
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