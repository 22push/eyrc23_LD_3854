from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2

image = cv2.imread('led.jpg', cv2.IMREAD_COLOR)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 0)
thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=4)
labels = measure.label(thresh, connectivity=2, background=0)
mask = np.zeros(thresh.shape, dtype="uint8")

for label in np.unique(labels):
    if label == 0:
        continue
    labelMask = np.zeros(thresh.shape, dtype="uint8")
    labelMask[labels == label] = 255
    numPixels = cv2.countNonZero(labelMask)
    if numPixels > 100:
        mask = cv2.add(mask, labelMask)

cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

cv2.drawContours(image, cnts, -1, (0, 0, 255), 2)
centroids = []
areas = []

for (i, c) in enumerate(cnts):
    x,y = c[0][0]
    cv2.putText(image, str(i + 1), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    area = cv2.contourArea(c)
    M = cv2.moments(c)
    if M["m00"] != 0:
        cX = M["m10"] / M["m00"]
        cY = M["m01"] / M["m00"]
    else:
        cX, cY = 0, 0
    centroids.append((cX, cY))
    areas.append(area)

print(f"Number of LEDs detected: {len(centroids)}")


for i, centroid in enumerate(centroids):
    print(f"LED {i + 1} - Centroid: {centroid}, Area: {areas[i]}")
cv2.imwrite("led_detection_results.png", image)

a = len(centroids)
with open("led_detection_results.txt", "w") as file:
    file.write(f"No. of LEDs detected: {a}\n")
    
    file.write(f"Centroid #{i + 1}: {centroid}\nArea #{i + 1}: {area}\n")

file.close()

cv2.imshow("Image with LED Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
