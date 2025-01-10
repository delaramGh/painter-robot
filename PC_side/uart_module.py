import serial
import time
import numpy as np




class Robot:
    def __init__(self, port_='COM7'):
        self.L = 6.5
        self.L1 = 11.3
        self.L2 = 8.5
        self.M1 = (-self.L/2, 0)
        self.M2 = (self.L/2, 0)
        self.goal = [0, 15]
        self.angle_a_prv = 110.74  #for (0, 15)
        self.angle_b_prv = 110.74  #for (0, 15)

        self.ser = serial.Serial(port=port_, baudrate=9600, timeout=10)
        time.sleep(1)


    def sign(self, a):
        if a > 0:
            return 1
        return 0
    

    def calculate_angle(self, x, y):
        self.goal[0] = x
        self.goal[1] = y
        M1E = ((self.M1[0]-self.goal[0])**2 + (self.M1[1]-self.goal[1])**2)**0.5
        M2E = ((self.M2[0]-self.goal[0])**2 + (self.M2[1]-self.goal[1])**2)**0.5
        angle_a = (np.arccos((self.L**2 + M1E**2 - M2E**2)/(2*self.L*M1E)) + np.arccos((self.L1**2 + M1E**2 - self.L2**2)/(2*self.L1*M1E)))*180/np.pi
        angle_b = (np.arccos((self.L**2 + M2E**2 - M1E**2)/(2*self.L*M2E)) + np.arccos((self.L1**2 + M2E**2 - self.L2**2)/(2*self.L1*M2E)))*180/np.pi
        
        delta_a = abs(self.angle_a_prv - angle_a)
        delta_b = abs(self.angle_b_prv - angle_b)
        dir_a = 1 - self.sign(self.angle_a_prv-angle_a)
        dir_b = self.sign(self.angle_b_prv-angle_b)

        self.angle_a_prv = angle_a
        self.angle_b_prv = angle_b

        return delta_a, dir_a, delta_b, dir_b


    def go_to_xy(self, x, y, sleep_=0.1):
        delta_a, dir_a, delta_b, dir_b = self.calculate_angle(x, y)

        message = f"a{int(delta_a/1.8)}A{dir_a}b{int(delta_b/1.8)}B{dir_b}f"
        self.ser.write(message.encode())

        # print(self.angle_a_prv, self.angle_b_prv)
        # if dir_a == 1:
        #     self.angle_a_prv += int(delta_a/1.8) * 1.8
        # else:
        #     self.angle_a_prv -= int(delta_a/1.8) * 1.8

        # if dir_b == 1:
        #     self.angle_b_prv -= int(delta_b/1.8) * 1.8
        # else:
        #     self.angle_b_prv += int(delta_b/1.8) * 1.8
        # print(self.angle_a_prv, self.angle_b_prv, "\n")

        time.sleep(sleep_)




if __name__ == "__main__":
    r = Robot()
    
    n = 20
    X = np.linspace(-10, 10, n)
    Y = 10 * np.ones((n,))
    for i in range(n):
        r.go_to_xy(X[i], Y[i], 0.1)