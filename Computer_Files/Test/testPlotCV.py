# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 09:45:24 2019

@author: Quentin
"""

# -*- coding: utf-8 -*-
"""
Created on Tue Jun 11 17:05:33 2019

@author: Quentin
"""
import cv2
import cv2.aruco as aruco
import random

from calibrate_camera import calibration, undistort

adressCam1 = 'http://root:1234@169.254.206.22/mjpg/video.mjpg'    

def resizeFrame(frame):
    """
    frame : image a resize (l'image de la camera étant trop grande pour mon ecran)
    Return:
        newFrame: image resize
    """
    
    imgScale = 0.4
    newX,newY = frame.shape[1]*imgScale, frame.shape[0]*imgScale
    newFrame = cv2.resize(frame,(int(newX),int(newY)))
    
    return newFrame


newcameramtx, roi, mtx, dist =  calibration()

cap1 = cv2.VideoCapture()
cap1.open(adressCam1)


while(True):
    
    key = cv2.waitKey(1000) & 0xFF
    
    ret, frame1 = cap1.read()
    if ret:
        sizeYFrame = len(frame1)
        sizeXFrame = len(frame1[0])
        posX = random.randint(0,sizeXFrame)
        posY = random.randint(0,sizeYFrame)
        color = [random.randint(0,255),random.randint(0,255),random.randint(0,255)] #code BGR
        frame1 = undistort(frame1, newcameramtx, roi, mtx, dist)
        frame1[1000-10:1000+10,1000-10:1000+10] = [0,0,255]
        print("Color", posX, posY)
        cv2.imshow("Webcam 1", resizeFrame(frame1))
    else:
        print("No Cam 1")


    if key == 27:
        break

cap1.release()
cv2.destroyAllWindows()
