import cv2
import numpy as np
import matplotlib.pyplot as plt


def edge_detection(file_name, th=180, blur=(5, 5)):
    img = cv2.imread(file_name, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, None , fx=0.4, fy=0.4)
    smoothed = cv2.GaussianBlur(img, blur, 0)
    edges = cv2.Canny(smoothed, threshold1=th, threshold2=th)
    return edges


if __name__ == "__main__":

    edges = edge_detection("1.jpg")

    print(np.min(edges), np.max(edges))

    img = cv2.cvtColor(edges, cv2.COLOR_BGR2RGB)
    plt.imshow(img)
    # plt.show()


