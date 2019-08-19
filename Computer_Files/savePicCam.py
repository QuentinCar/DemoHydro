# -*- coding: utf-8 -*-
"""
Created on Wed Aug 14 10:53:47 2019

@author: Quentin
"""
from calibrate_camera import calibration, undistort
import cv2

if __name__ == "__main__":

    adressCam1 = 'http://root:1234@169.254.206.22/mjpg/video.mjpg'    

    newcameramtx, roi, mtx, dist =  calibration()
    
    cap1 = cv2.VideoCapture()
    cap1.open(adressCam1)
    
    ret, frame1 = cap1.read()
    if ret:
        frame1 = undistort(frame1, newcameramtx, roi, mtx, dist)
        cv2.imwrite('fondPlot.png',frame1)

    else:
        print("No Cam 1")

