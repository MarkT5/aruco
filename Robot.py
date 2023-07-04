import serial
import time
import threading as thr
import numpy as np


class Robot:
    def __init__(self, port="COM2"):
        self.ser = serial.Serial(port, 115200, timeout=.1)
        time.sleep(2)
        self.thread = thr.Thread(target=self.mainLoop)
        self.angPID = np.array([0.0, 0, 0])
        self.angPIDk = np.array([230.0, 0, -10000])
        self.targetAng = 0
        self.currAng = 0
        self.movementAllowed = False
        self.lastTime = time.time()
        self.thread.start()
        self.targetSpeed = 0

    def move(self, fov, side):
        assert fov == int(fov)
        assert side == int(side)
        fov = int(fov)
        side = int(side)
        self.ser.write(b'l' + (fov + side).to_bytes(2, 'big', signed=True))
        self.ser.write(b'r' + (fov - side).to_bytes(2, 'big', signed=True))

    def lift(self, val):
        assert val == int(val)
        assert 90 <= val <= 210
        self.ser.write(b'u' + int(val).to_bytes(2, 'big', signed=True))

    def grab(self, val):
        assert val == int(val)
        assert 110 <= val <= 240
        self.ser.write(b'g' + int(val).to_bytes(2, 'big', signed=True))

    def holdAng(self, ang):
        self.targetAng = ang

    def setSpeed(self, speed):
        self.targetSpeed = speed


    def mainLoop(self):
        while True:
            if self.movementAllowed:
                if abs(self.currAng - self.targetAng) > 0.05:
                    self.angPID[0] = self.currAng - self.targetAng
                    if abs(self.currAng - self.targetAng) < 0.3:
                        self.angPID[1] += self.currAng - self.targetAng
                        self.angPID[1] = min(max(-80, self.angPID[1]), 80)
                    else:
                        self.angPID[1] = 0
                    self.angPID[2] = (self.currAng - self.targetAng)*(time.time()-self.lastTime)
                    self.lastTime = time.time()
                    print(self.angPID*self.angPIDk, "\t", self.currAng - self.targetAng, '\t', np.sum(self.angPID*self.angPIDk))
                    self.move(self.targetSpeed, max(-70, min(70, -int(np.sum(self.angPID*self.angPIDk)))))
                else:
                    self.move(self.targetSpeed, 0)
            else:
                self.move(0, 0)
            time.sleep(0.01)



