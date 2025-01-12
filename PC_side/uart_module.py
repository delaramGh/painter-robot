import serial
import time
import numpy as np
from dmh_kinematics import directKinematics, inverseKinematics, rad2deg, deg2rad
import matplotlib.pyplot as plt

class Robot:
    def __init__(self, debug=0, port_='COM7'):
        self.L = 6.5
        self.L1 = 11.3
        self.L2 = 8.5
        self.M1 = (-self.L/2, 0)
        self.M2 = (self.L/2, 0)
        self.goal = [0, 15.8]
        self.angle_a_prv = 108.4  #for (0, 15.4)
        self.angle_b_prv = 108.4  #for (0, 15.4)
        self.debug = debug
        if not self.debug:
            self.ser = serial.Serial(port=port_, baudrate=9600, timeout=10)
            time.sleep(1)
        self.E_list = []


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

        # self.angle_a_prv_old = angle_a
        # self.angle_b_prv_old = angle_b
        print('Before moving A: ', round(self.angle_a_prv, 2), ", B: ", round(self.angle_b_prv, 2))

        if dir_a == 1:
            self.angle_a_prv += int(delta_a/1.8) * 1.8
        else:
            self.angle_a_prv -= int(delta_a/1.8) * 1.8

        if dir_b == 1:
            self.angle_b_prv -= int(delta_b/1.8) * 1.8
        else:
            self.angle_b_prv += int(delta_b/1.8) * 1.8
        
        #------------------ DEBUG -------------------
        A = np.array([-self.L/2, 0])
        B = np.array([self.L/2, 0])
        LA = LB = self.L1
        LC = LD = self.L2
        E = np.array(self.goal)
        theta_A, theta_B, theta_C, theta_D = inverseKinematics(A, B, self.L, LA, LB, LC, LD, E)
        
        print('Target Angle A: ', round(angle_a, 2), ', MH A:', round(rad2deg(theta_A), 2), ', Angle A Actual: ', round(self.angle_a_prv, 2))
        print('Target Angle B: ', round(angle_b, 2), ', MH B:', round(rad2deg(theta_B), 2), ', Angle B Actual: ', round(self.angle_b_prv, 2))
     
        E_a, E_b, C, D = directKinematics(A, B, LA, LB, LC, LD, deg2rad(self.angle_a_prv), deg2rad(self.angle_b_prv), theta_C, theta_D)
        print('goal: ', self.goal)
        print('actual: ', round(E_a[0], 3), ', ', round(E_a[1], 3))
        self.E_list.append(E_a)

        print("delta A: ", round(delta_a, 2), ', ', int(delta_a/1.8))
        print("delta B: ", round(delta_b, 2), ', ', int(delta_b/1.8), '\n')

        return delta_a, dir_a, delta_b, dir_b


    def go_to_xy(self, x, y, sleep_=0.1):
        delta_a, dir_a, delta_b, dir_b = self.calculate_angle(x, y)

        message = f"a{int(delta_a/1.8)}A{dir_a}b{int(delta_b/1.8)}B{dir_b}f"
        if not self.debug:
            self.ser.write(message.encode())
       
        time.sleep(sleep_)




if __name__ == "__main__":
    r = Robot(debug=0)
    
    n = 20
    X = np.linspace(-10, 10, n)
    Y = 10 * np.ones((n,))
    for i in range(n):
        print(f"*** {i}th step")
        r.go_to_xy(X[i], Y[i], 0.05)

    E_arr = np.array(r.E_list)
    plt.figure()
    plt.plot(E_arr[:, 0], E_arr[:, 1], 'r')
    plt.ylim([0, 15])
    plt.show()
