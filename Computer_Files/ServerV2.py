# -*- coding: utf-8 -*-
"""
Created on Sun Jan 27 22:46:28 2019
SOCKETS : 
https://openclassrooms.com/fr/courses/235344-apprenez-a-programmer-en-python/234698-le-reseau
THREADS:
https://openclassrooms.com/fr/courses/235344-apprenez-a-programmer-en-python/2235545-la-programmation-parallele-avec-threading

/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\/!\
Si mode manuel :
    BIEN ECRIRE 'fin' A LA FIN DE COMMUNICATION POUR FERMER LES SOCKETS (à écrire côté serveur) !!!
Si mode caméra:
    Appuyer sur Echap pour quitter la fenêtre 'Webcam', ce qui entraîne les fermetures de socket.
"""

from threading import Thread, RLock, Event
from Thread_Server_Listening import *
from Thread_Server_Writing import *

import scipy.stats
import numpy as np
import cv2
from posRegul import getTheta, bordBassin, resizeFrame
import math 

PORT_ALLER = 12800
PORT_RETOUR = 12801

# Synchronise l'extinction de Server et Listening
alive = Event()
alive.set()

# Création des threads
thread_listening = Listening_Server(PORT_ALLER, PORT_RETOUR, alive)

# Lancement des threads
thread_listening.start()
#############################################################
#------------ Init pour l'affichage de la bathy -------------
#############################################################
fondPlot = cv2.imread("fondPlot.png")
pas = 10

def getRotationMatrixInv(theta):
    return np.array([[math.cos(theta), math.sin(theta)],
                     [-math.sin(theta), math.cos(theta)]])


def meter2pixel(X,Y):
    bordBassin1 = 17 #id de l'aruco en (0,0) du bassin
    bordBassin2 = 10 #id de l'aruco (0,MaxY) du bassin

    X = X * (bordBassin[bordBassin2][1] - bordBassin[bordBassin1][1]) / 3
    Y = Y * (bordBassin[bordBassin2][1] - bordBassin[bordBassin1][1]) / 3
    
    pos = np.array([[X],[Y]])
    
    pos =  np.linalg.inv(getRotationMatrixInv(getTheta())) @ pos
    
    pos = np.array([[pos[0][0] + bordBassin[bordBassin1][0]],
                    [pos[1][0] + bordBassin[bordBassin1][1]]])
    
    return int(pos[0][0]), int(pos[1][0])
#--------------------------------------------------------
while alive.is_set():
    X, Y, Z, xBoat, yBoat = thread_listening.toMap()
#    print(X)
    if X != []:
#        print("BATH")
        #Représentation 3D en 2D
        #bathy 3d avec scatter couleur entre -3 et 0 m
        #voir plotGauss.py pour avoir une idée des coefficients std
        print("----------- Update Bathymetrie ---------------")
        for x,y,z in zip(X,Y,Z):
            if xBoat > 0 and yBoat > 0 and xBoat < 4 and yBoat < 3:
                std = 0.4
                r = 2.5*std*scipy.stats.norm.pdf(z,-1.5,std)
                g = 2.5*std*scipy.stats.norm.pdf(z,0,std)
                b = 2.5*std*scipy.stats.norm.pdf(z,-3,std)
            
                col = [b,g,r]#blue if z = -3, green if z = 0, red if z = -1.5
                xPlot, yPlot =meter2pixel(x, y)
                fondPlot[xPlot-pas:xPlot+pas, yPlot-pas:yPlot+pas] = [0,0,255]
#                fondPlot[10-pas:10+pas, 10-pas:10+pas] = [0,0,255]
        
        print(len(fondPlot), len(fondPlot[0]), xPlot, yPlot)
#        print(fondPlot[xBateau-pas:xBateau+pas, yBateau-pas:xBateau+pas], col)
        cv2.imshow('Affichage Bathymetrie',resizeFrame(fondPlot))
        cv2.waitKey(1)
                
            
            
print("------ IS_DEAD -------")
  
# Attend que les threads se terminent
thread_listening.join()



