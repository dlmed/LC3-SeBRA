# adapted from https://www.thepoorengineer.com/en/angle-control-absolute/

import numpy as np
#from SBR import gyro_signal, accel_signal

class KalmanFilter:
    def __init__(self, xHat, p, q, r, deltaT):
        self.xHat = xHat # 2x1 array
        self.xHatBar = np.array([0.0, 0.0])
        self.pBar = np.array([[0.0, 0.0], [0.0, 0.0]])
        self.k = np.array([0.0, 0.0])
        self.p = p # 2x2 array
        self.q = q # 2x2 array
        self.r = r # float
        self.deltaT = deltaT / 1000000.0 # in microseconds
    
    def update(self, angularVel, angle):
        # Prediction
        self.xHatBar[0] = self.xHat[0] + self.deltaT * (angularVel - self.xHat[1])
        self.xHatBar[1] = self.xHat[1]

        self.pBar[0][0] = self.p[0][0] + self.deltaT * ((self.deltaT * self.p[1][1]) \
            - self.p[1][0] - self.p[0][1] + self.q[0][0])
        self.pBar[0][1] = self.p[0][1] + self.deltaT * (self.q[0][1] - self.p[1][1])
        self.pBar[1][0] = self.p[1][0] + self.deltaT * (self.q[1][0] - self.p[1][1])
        self.pBar[1][1] = self.p[1][1] + self.deltaT * (self.q[1][1])

        # Update
        self.k[0] = self.pBar[0][0] / (self.pBar[0][0] + self.r)
        self.k[1] = self.pBar[1][0] / (self.pBar[0][0] + self.r)

        self.xHat[0] = self.xHatBar[0] + self.k[0] * (angle - self.xHatBar[0])
        self.xHat[1] = self.xHatBar[1] + self.k[1] * (angle - self.xHatBar[0])

        self.p[0][0] = self.pBar[0][0] - (self.k[0] * self.pBar[0][0])
        self.p[0][1] = self.pBar[0][1] - (self.k[0] * self.pBar[0][1])
        self.p[1][0] = self.pBar[1][0] - (self.k[1] * self.pBar[0][0])
        self.p[1][1] = self.pBar[1][1] - (self.k[1] * self.pBar[0][1])
        

    def getAngle(self):
        return self.xHat[0]

# Accelerometer and Gyroscope data
class AccelGyro():
    def __init__(self):
        self.loopTime = 50000   # microseconds
        self.kalmanAngle = 0.0
        self.xHat = np.array([0.0, 0.0])
        self.p = np.array([[1.0, 1.0], [1.0, 1.0]])
        self.q = np.array([[0.001, 0.001], [0.001, 0.001]])
        self.r = 0.25
        self.filter = KalmanFilter(self.xHat, self.p, self.q, self.r, self.loopTime)
        self.gyroAngle = 0.0 # rad
        self.prevAccelAngle = 0.0
    
    def loop(self, gyroData, accelDataX, accelDataZ):
        # gyroData is the y coordinate of CoppSim gyro_signal
        gyroData = gyroData * 180 / np.pi
        self.gyroAngle += gyroData * (self.loopTime / 1000000.0)
        if self.gyroAngle < -180:
            self.gyroAngle += 360
        elif self.gyroAngle > 180:
            self.gyroAngle -= 360
        
        magnitude = np.sqrt(accelDataX * accelDataX + accelDataZ * accelDataZ)
        # only read from accelerometer if magnitude is between 0.5g and 1.5g
        if magnitude > 8.192 and magnitude < 24.576:
            accelAngle = np.arctan2(accelDataX, -accelDataZ) * 180 / np.pi
        else:
            accelAngle = self.prevAccelAngle + gyroData * (self.loopTime / 1000000.0)
        self.filter.update(gyroData, accelAngle)
        kalmanAngle = self.filter.getAngle()
        self.prevAccelAngle = accelAngle

        return kalmanAngle
