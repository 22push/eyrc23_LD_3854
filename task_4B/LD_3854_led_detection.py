from imutils import contours
from skimage import measure
import numpy as np
import imutils
import cv2
import argparse

aliens = {
    2: 'alien_a',
    3: 'alien_b',
    4: 'alien_c',
    5: 'alien_d'
}

def write_to_txt_file(file_path, organism_type,centroid_x,centroid_y):
    with open(file_path, 'a') as file:
        file.write(f"Organism Type: {organism_type}\n")
        file.write(f"Centroid: ({round(centroid_x,4)}, {round(centroid_y,4)})\n\n")





def leds_cropped(image, mask):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (15,15), 0)
    thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=4)
    labels = measure.label(thresh, connectivity=2, background=0)

    for label in np.unique(labels):
        if label == 0:
            continue
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = 255
        numPixels = cv2.countNonZero(labelMask)
        if numPixels > 10:
            mask = cv2.add(mask, labelMask)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    cv2.drawContours(image, cnts, -1, (0, 0, 255), 2)
    # cv2.imshow("df", image)

    centroids = []
    areas = []
    cx=0
    cy=0
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
        cx = cx+cX
        cy=cy+cY
        areas.append(area)
    
    return len(np.unique(labels)),cx/(len(np.unique(labels))-1),cy/(len(np.unique(labels))-1)


def count_leds_in_mask(mask, original_image):
    image_h, image_w, _ = original_image.shape

    countours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    countours = imutils.grab_contours(countours)

    led_count = len(countours)

    # print(f"Number of LEDs in the given mask: {led_count}")

    for led_contour in countours:
        mask_led = np.zeros((image_h, image_w), dtype=np.uint8)
        cv2.drawContours(mask_led, [led_contour], 0, (255), thickness=cv2.FILLED)

        cropped_region = cv2.bitwise_and(original_image, original_image, mask=mask_led)
        numled_onep,centroid_x,centroid_y=(leds_cropped(cropped_region, np.zeros(mask.shape, dtype="uint8")))
        write_to_txt_file('abcd.txt',aliens[numled_onep-1],centroid_x,centroid_y)
        # cv2.imshow("Cropped LED Region", cropped_region)
        # cv2.waitKey(0)

    return led_count


parser = argparse.ArgumentParser()
parser.add_argument('--image', dest='input_image', required=True)
args = parser.parse_args()
path = args.input_image

image = cv2.imread(path, cv2.IMREAD_COLOR)
image_h, image_w, _ = image.shape
# print(image_h, image_w, _)
kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (15, 15), 0)
thresh = cv2.threshold(blurred, 200, 255, cv2.THRESH_BINARY)[1]
thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, kernel, iterations=25)

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

led_count= count_leds_in_mask(mask, image)

# print(f"Total number of LEDs in the image: {led_count}")

cv2.destroyAllWindows()
