import serial
import time
import numpy as np

def sign(a):
    if a > 0:
        return 1
    return 0



ser = serial.Serial(port='COM7', baudrate=9600, timeout=1)
time.sleep(2)

manual = 0

L = 6.5
L1 = 11.6
L2 = 10
M1 = (-L/2, 0)
M2 = (L/2, 0)
goal = [0, 10]
angle_a_prv = 90
angle_b_prv = 90
while not manual:
    goal[0] = int(input("goal[0]: "))
    goal[1] = int(input("goal[1]: "))

    M1E = ((M1[0]-goal[0])**2 + (M1[1]-goal[1])**2)**0.5
    M2E = ((M2[0]-goal[0])**2 + (M2[1]-goal[1])**2)**0.5
    angle_a_next = (np.arccos((L**2 + M1E**2 - M2E**2)/(2*L*M1E)) + np.arccos((L1**2 + M1E**2 - L2**2)/(2*L1*M1E)))*180/np.pi
    angle_b_next = (np.arccos((L**2 + M2E**2 - M1E**2)/(2*L*M2E)) + np.arccos((L1**2 + M2E**2 - L2**2)/(2*L1*M2E)))*180/np.pi
    angle_a = angle_a_prv - angle_a_next
    angle_b = angle_b_prv - angle_b_next
    dir_a = 1 - sign(angle_a)
    dir_b = sign(angle_b)

    message = f"a{int(abs(angle_a)/1.8)}A{dir_a}b{int(abs(angle_b)/1.8)}B{dir_b}f"
    ser.write(message.encode())
    
    angle_a_prv = angle_a_next
    angle_b_prv = angle_b_next
    time.sleep(0.5)



while manual:
    angle_a = input("\nangle a: ")
    dir_a = input("dir a: ")
    angle_b = input("angle b: ")
    dir_b = input("dir b: ")

    message = f"a{angle_a}A{dir_a}b{angle_b}B{dir_b}f"
    ser.write(message.encode())
    time.sleep(0.1)

    response = ser.read_until(expected='-').decode()
    print("*** Received: ", response)