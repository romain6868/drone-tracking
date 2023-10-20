import cv2.legacy
import numpy as np
from picamera2 import Picamera2, Preview
import time
from picamera2.encoders import H264Encoder
import os
import cv2
import cv2.aruco as aruco
import sys, time
from math import sqrt
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray


# Initialisation du tracker CSRT
tracker = cv2.legacy.TrackerCSRT_create()
cap = cv2.VideoCapture(0)
time.sleep(5)

ids = None
while ids is None :
    print("[INFO] loading image...")
    res, image = cap.read()
    # loop over the types of ArUco dictionaries
    arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
    arucoParams = cv2.aruco.DetectorParameters_create()
    corners, ids, rejected = cv2.aruco.detectMarkers(
        image, arucoDict, parameters=arucoParams)
    # if at least one ArUco marker was detected display the ArUco
    # name to our terminal
    if len(corners) > 0:
        print("[INFO] detected marker detected")
        print("id of the first maker",ids.flatten()[0])
    if ids is not None :
        aruco_id = ids.flatten()[0]  # Select the first ArUco id from the list
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        x_centerPixel_target = int(x_sum*.25)
        print(x_centerPixel_target)
        y_centerPixel_target = int(y_sum*.25)
        print(y_centerPixel_target)
        x_pixel_target_out = x_centerPixel_target
        y_pixel_target_out = y_centerPixel_target
        arrete_marker_pxl = sqrt((corners[0][0][0][0]-corners[0][0][1][0])**2+(corners[0][0][0][1]-corners[0][0][1][1])**2)
        Xmax = min(int(corners[0][0][0][0]),int(corners[0][0][1][0]),int(corners[0][0][2][0]),int(corners[0][0][3][0]))
        Ymax = min(int(corners[0][0][0][1]),int(corners[0][0][1][1]),int(corners[0][0][2][1]),int(corners[0][0][3][1]))
        Xmin = max(int(corners[0][0][0][0]),int(corners[0][0][1][0]),int(corners[0][0][2][0]),int(corners[0][0][3][0]))
        Ymin = max(int(corners[0][0][0][1]),int(corners[0][0][1][1]),int(corners[0][0][2][1]),int(corners[0][0][3][1]))
        H = Ymin-Ymax
        print(H)
        W = Xmin - Xmax
        print(W)
        
        cv2.line(image, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
        cv2.line(image, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)
        cv2.putText(image, str(aruco_id)+"a", (int(x_centerPixel_target), int(y_centerPixel_target)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
        cv2.rectangle(image, (x_centerPixel_target+25, y_centerPixel_target+25), (x_centerPixel_target-25, y_centerPixel_target-25), (0, 0, 255), 2)
        cv2.imshow('Test',image)



# Lire la première frame pour sélectionner la région d'intérêt (ROI)

res, image = cap.read()
# Sélectionner la ROI en utilisant cv2.selectROI
ROI = (Xmax-0.15*H,Ymax-0.15*W,H*1.3,W*1.3)
print(ROI)
tracker.init(image, ROI)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Mettre à jour le tracker avec la nouvelle frame
    ok, bbox = tracker.update(frame)

    if ok:
        # Tracking réussi, dessiner la boîte autour de l'objet suivi
        # info : bbox = [x , y, largeur , hauteur], avec (x,y) coord coin sup gauche
        p1 = (int(bbox[0]), int(bbox[1]))                           # coin sup gauche
        p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))       # coin inf droit
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 2)
    else:
        # Tracking échoué
        cv2.putText(frame, "Tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    cv2.imshow('Object Tracking', frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()


