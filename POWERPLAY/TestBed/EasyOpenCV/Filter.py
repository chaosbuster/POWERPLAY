import numpy as np
import cv2
import sys

def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)


if (len(sys.argv) <= 1) :
    print("'Usage: python hsvThresholder.py <ImageFilePath>' to ignore camera and use a local image.")
    useCamera = True
    
## Read
image = cv2.imread(sys.argv[1])

image = ResizeWithAspectRatio(image, width=640) # Resize by width

original = image.copy()

## convert to hsv
image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

## mask for yellow (15,0,0) ~ (36, 255, 255)
lower_yellow = np.array([0,162,191], dtype="uint8")
upper_yellow = np.array([93, 255, 255], dtype="uint8")

## mask for red  
lower_red = np.array([164,158,136], dtype="uint8")
upper_red = np.array([180,255,255], dtype="uint8")

## mask for blue
lower_blue = np.array([30,141,60], dtype="uint8")
upper_blue = np.array([122,217,160], dtype="uint8")

target_redcones = cv2.inRange(image, lower_red, upper_red)
target_bluecones = cv2.inRange(image, lower_blue, upper_blue)
target_junctions = cv2.inRange(image, lower_yellow, upper_yellow)

cnts_redcones = cv2.findContours(target_redcones, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts_redcones = cnts_redcones[0] if len(cnts_redcones) == 2 else cnts_redcones[1]

cnts_bluecones = cv2.findContours(target_bluecones, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts_bluecones = cnts_bluecones[0] if len(cnts_bluecones) == 2 else cnts_bluecones[1]

cnts_junctions = cv2.findContours(target_junctions, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cnts_junctions = cnts_junctions[0] if len(cnts_junctions) == 2 else cnts_junctions[1]

for c in cnts_redcones:
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(original, (x, y), (x + w, y + h), (255, 0, 255), 2)

for c in cnts_bluecones:
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(original, (x, y), (x + w, y + h), (190, 83, 82), 2)

for c in cnts_junctions:
    x,y,w,h = cv2.boundingRect(c)
    cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)

cv2.imwrite("originalTOhsv.png", image)
cv2.imwrite("target_redcones.png", target_redcones)
cv2.imwrite("target_bluecones.png", target_bluecones)
cv2.imwrite("target_junctions.png", target_junctions)
cv2.imwrite("targetswithcontours.png", original)



