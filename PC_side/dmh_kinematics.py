import numpy as np
from numpy.linalg import norm
# import matplotlib.pyplot as plt

def rad2deg(rad):
    return rad * 180 / np.pi

def deg2rad(deg):
    return deg * np.pi / 180

def directKinematics(A, B, LA, LB, LC, LD, theta_A, theta_B, theta_C, theta_D):
    theta_A -= np.pi/2
    theta_B -= np.pi/2
    C = A + np.array([-LA * np.sin(theta_A), LA * np.cos(theta_A)])
    D = B + np.array([LB * np.sin(theta_B), LB * np.cos(theta_B)])
    E_a = C + np.array([LC * np.cos(theta_C), LC * np.sin(theta_C)])
    E_b = D + np.array([-LD * np.cos(theta_D), LD * np.sin(theta_D)])
    return E_a, E_b, C, D

def inverseKinematics(A, B, L, LA, LB, LC, LD, E):
    AE = norm(A - E)
    BE = norm(B - E)

    angle_CAE = np.arccos((LA**2 + AE**2 - LC**2) / (2 * LA * AE))
    angle_BAE = np.arccos((L**2 + AE**2 - BE**2) / (2 * L * AE))
    angle_BAC = angle_CAE + angle_BAE

    angle_DBE = np.arccos((LB**2 + BE**2 - LD**2) / (2 * LB * BE))
    angle_ABE = np.arccos((L**2 + BE**2 - AE**2) / (2 * L * BE))
    angle_ABD = angle_DBE + angle_ABE

    theta_A = np.pi/2 - (np.pi - angle_BAC)
    theta_B = np.pi/2 - (np.pi - angle_ABD)

    angle_ACE = np.arccos((LA**2 + LC**2 - AE**2) / (2 * LA * LC))
    angle_BDE = np.arccos((LB**2 + LD**2 - BE**2) / (2 * LB * LD))
    theta_C = angle_ACE - (np.pi/2 - theta_A)
    theta_D = angle_BDE - (np.pi/2 - theta_B)

    theta_A += np.pi/2
    theta_B += np.pi/2
    return theta_A, theta_B, theta_C, theta_D


if __name__ == '__main__':
    L = 6.5
    L1 = 11.3
    L2 = 8.5
    LA = LB = L1
    LC = LD = L2

    A = np.array([-L/2, 0])
    B = np.array([L/2, 0])
    E = np.array([0, 15.8])
    # E = np.array([-L/2, 15])
    # E = np.array([-10, 10])

    theta_A, theta_B, theta_C, theta_D = inverseKinematics(A, B, L, LA, LB, LC, LD, E)
    print("Theta A: ", rad2deg(theta_A))
    print("Theta B: ", rad2deg(theta_B))
    print("Theta C: ", rad2deg(theta_C))
    print("Theta D: ", rad2deg(theta_D))
    E_a, E_b, C, D = directKinematics(A, B, LA, LB, LC, LD, theta_A, theta_B, theta_C, theta_D)
    print("E_a: ", E_a)
    print("E_b: ", E_b)

def visualize(A, B, LA, LB, LC, LD, theta_A, theta_B, theta_C, theta_D):
    # Compute coordinate of C and D using directKinematics or manual geometry
    # (Assuming directKinematics returns (E_a, E_b, C, D) for demonstration.)
    E_a, E_b, C, D = directKinematics(A, B, LA, LB, LC, LD, theta_A, theta_B, theta_C, theta_D)
    
    # Convert angles to degrees
    ta = rad2deg(theta_A)
    tb = rad2deg(theta_B)
    tc = rad2deg(theta_C)
    td = rad2deg(theta_D)
    
    plt.figure()
    # Plot points
    plt.plot(A[0], A[1], 'ko', label='A', markersize=10)
    plt.plot(B[0], B[1], 'ko', label='B', markersize=10)
    plt.plot(C[0], C[1], 'ko', label='C', markersize=10)
    plt.plot(D[0], D[1], 'ko', label='D', markersize=10)
    plt.plot(E_a[0], E_a[1], 'ro', label='E', markersize=10)
    
    # Plot links
    plt.plot([A[0], C[0]], [A[1], C[1]], 'k-', linewidth=3)
    plt.plot([C[0], E_a[0]], [C[1], E_a[1]], 'k-', linewidth=3)
    plt.plot([B[0], D[0]], [B[1], D[1]], 'k-', linewidth=3)
    plt.plot([D[0], E_b[0]], [D[1], E_b[1]], 'k-', linewidth=3)
    
    # Display angles on the side
    # # angles_text = f"Theta A: {ta:.2f}\nTheta B: {tb:.2f}\nTheta C: {tc:.2f}\nTheta D: {td:.2f}"
    # plt.text(1.05, 0.5, angles_text, transform=plt.gca().transAxes, va='center')
    
    plt.axis('equal')
    plt.legend()
    plt.show()

# visualize(A, B, LA, LB, LC, LD, theta_A, theta_B, theta_C, theta_D)