import serial
import time
import numpy as np

def sign(a):
    if a > 0:
        return 1
    return 0


class Robot:
    def __init__(self, port_='COM7'):
        self.L = 6.5
        self.L1 = 11.6
        self.L2 = 10+0.6
        self.M1 = (-self.L/2, 0)
        self.M2 = (self.L/2, 0)
        self.goal = [0, 15]
        self.angle_a_prv = 118.43  #for (0, 15)
        self.angle_b_prv = 118.43  #for (0, 15)

        self.ser = serial.Serial(port=port_, baudrate=9600, timeout=10)
        time.sleep(2)
        

    def go_to_xy(self, x, y, sleep_=2):
        self.goal[0] = x
        self.goal[1] = y

        M1E = ((self.M1[0]-self.goal[0])**2 + (self.M1[1]-self.goal[1])**2)**0.5
        M2E = ((self.M2[0]-self.goal[0])**2 + (self.M2[1]-self.goal[1])**2)**0.5
        angle_a_next = (np.arccos((self.L**2 + M1E**2 - M2E**2)/(2*self.L*M1E)) + np.arccos((self.L1**2 + M1E**2 - self.L2**2)/(2*self.L1*M1E)))*180/np.pi
        angle_b_next = (np.arccos((self.L**2 + M2E**2 - M1E**2)/(2*self.L*M2E)) + np.arccos((self.L1**2 + M2E**2 - self.L2**2)/(2*self.L1*M2E)))*180/np.pi
        angle_a = self.angle_a_prv - angle_a_next
        angle_b = self.angle_b_prv - angle_b_next
        dir_a = 1 - sign(angle_a)
        dir_b = sign(angle_b)

        message = f"a{int(abs(angle_a)/1.8)}A{dir_a}b{int(abs(angle_b)/1.8)}B{dir_b}f"
        self.ser.write(message.encode())

        self.angle_a_prv = angle_a_next
        self.angle_b_prv = angle_b_next
        time.sleep(sleep_)




if __name__ == "__main__":
    r = Robot()
    X = np.linspace(-10, 10, 20)
    Y = 15 * np.ones((20,))
    r.go_to_xy(X[0], Y[0], 3)
    for i in range(1, 20):
        r.go_to_xy(X[i], Y[i], 0.01)