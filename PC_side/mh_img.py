import numpy as np
import cv2

def normalize(img, points, b=8, t=18, l=-5, r=5):
    arr = np.array(points) / np.array(img.shape)
    arr = arr * np.array([t - b, r - l]) + np.array([b, l])
    arr = np.round(arr, 3)
    return arr.tolist()

def generatePoints(img_path, thresh=127):
    img = cv2.resize(cv2.imread(img_path, 0), (100, 100))
    img = np.where(img > thresh, 255, 0)
    return normalize(img, sorted(zip(*np.where(img == 0))))

if __name__ == '__main__':
    img_path = 'rectangle.png'
    points = generatePoints(img_path)
    print(points, len(points))