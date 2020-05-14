#!/usr/bin/env python
# -*- coding:utf-8 -*-
from naoqi import ALProxy
import almath
import time
import math
import motion
import sys
import thread
import cv2
import array, struct

import numpy as np
from urllib2 import urlopen
from cStringIO import StringIO


def StiffnessOn(proxy):  # corpo in posizione rigida
    proxy.stiffnessInterpolation("Body", 1, 1)


def firstricercaredball():
    global _numRicercaPalle, _passoRobot
    _numRicercaPalle += 1  # viene incrementato il tempo di ricerca ogni secondo
    camProxy.setActiveCamera(1)  # impostazione della camera di sotto del robot

    if _numRicercaPalle <= 3:
        ricercaRange = -60
    else:
        ricercaRange = -120

    ricerca = ricercaRange
    while ricerca <= -ricercaRange:  # finchè non raggiunge l'angolazione opposta continua a richiamare la funzione di ricerca
        findballinfo = findball(ricerca)  # ricerca palla in base alla posizione del collo
        if findballinfo != []:
            return findballinfo
        else:
            ricerca += 60
    print("Nessuna palla rossa")
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.setMoveArmsEnabled(False, False)  # ferma le mani
    motionPrx.moveTo(0.25, 0.0, 0.0, _passoRobot)  # In cerca di correzione della distanza
    allballDatatest = [0, 0, 1]
    return allballDatatest


def findball(angle, sub_angle=0):  # Restituisce i dati della palla rossa
    camProxy.setActiveCamera(1)
    # angleinterpolation -> 1)nome del corpo 2)angolo di interpolazione 3)velocita
    # headyaw: rotazione della testa da destra a sinistra o viceversa
    motionPrx.angleInterpolationWithSpeed("HeadYaw", angle * math.pi / 180,
                                          0.8)  # angolo * math.pi Pi / 180 è la formula di conversione radiante
    # headpitch: inclinazione della testa dal basso verso l'alto e viceversa
    motionPrx.angleInterpolationWithSpeed("HeadPitch", sub_angle * math.pi / 180, 0.8)
    redballProxy.subscribe("redBallDetected")
    # Inserisce una coppia chiave-valore in memoria, dove valore è un int parametri: chiave - Nome del valore da inserire. valore - L'int da inserire
    memoryProxy.insertData("redBallDetected", [])  # Inserisci i dati in memoria

    # Aggiungi il riconoscimento palla rossa e annulla il riconoscimento palla rossa
    time.sleep(2)
    for i in range(10):  # fa l'iterazione 10 volte
        ballData = memoryProxy.getData("redBallDetected")  # se trova la palla
        # [TimeStamp,BallInfo,CameraPose_InTorsoFrame,CameraPose_InRobotFrame, Camera_I]

    # redballProxy.unsubscribe("redBallDetected")
    if (ballData != []):
        print("La palla rossa è distante")
        print(ballData)
        headangle = motionPrx.getAngles("HeadYaw", True)  # salva angolo collo
        allballData = [headangle, ballData,
                       0]  # ritorna l'angolo del collo,i dati dell palla e 0 per far finire l'iterazione della ricerca
        return allballData
    else:
        print("Non trovato")
        return []


def CalculateRobotToRedball1(allballData):
    h = 0.478
    # Parametri di camminata del robot
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.3
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    headangle = allballData[0]  # Angolo di deflessione della testa
    wzCamera = allballData[1][1][0]  # angolo alfa
    wyCamera = allballData[1][1][1]  # angolo beta
    isenabled = False
    x = 0.0
    y = 0.0
    if (headangle[0] + wzCamera < 0.0):
        theta = headangle[0] + wzCamera + 0.2  # modifica 1
    else:
        theta = headangle[0] + wzCamera

    motionPrx.setMoveArmsEnabled(False, False)
    # Poi, per la prima volta, il robot si gira verso la palla rossa.
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])  # x=y=0

    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (39.7 * math.pi / 180.0)
    x = h / (math.tan(thetav)) - 0.5  # Minimo 40 cm
    if (x >= 0):
        theta = 0.0
        motionPrx.setMoveArmsEnabled(False, False)
        print("Mosso in direzione della palla rossa")
        # Successivamente, per la seconda volta, il robot cammina in una posizione a 20 cm dalla palla rossa,
        # con un programma di 40 cm, perché il robot fa troppi errori nella camminata reale per calciare la palla
        motionPrx.moveTo(x, y, theta,
                         [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
    motionPrx.waitUntilMoveIsFinished()
    # Abbassa la testa di 30 gradi.
    effectornamelist = ["HeadPitch"]
    timelist = [0.5]
    targetlist = [30 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, isenabled)
    time.sleep(1.5)
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("Camminare fino a 20cm dalla palla rossa")
    # Poi, per la terza volta, il robot corregge l'angolo alla palla rossa.
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)

    maxstepx = 0.02
    maxstepy = 0.14
    maxsteptheta = 0.15
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = (h - 0.03) / (math.tan(thetav)) - 0.15  # La linea a tre punti, in ultima analisi, rivede i punti chiave
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("Correzione dell'angolo completata")
    # Poi, per la quarta volta, il robot cammina a 10 centimetri dalla palla rossa
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    print("Raggiunti i 10cm")
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    print(thetah)
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    motionPrx.setMoveArmsEnabled(False, False)
    print("Camminare fino a 20cm dalla palla rossa")
    # Poi, per la quinta volta, il robot ha finalmente corretto l'angolo con la palla rossa
    motionPrx.moveTo(x, y, theta,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    # Successivamente, per la sesta e ultima volta, rilevare la distanza tra la palla e il robot dx
    val = memoryProxy.getData("redBallDetected")
    ballinfo = val[1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    dx = (h - 0.03) / (math.tan(thetav))  # dx come lato di un triangolo
    print (dx)
    return dx
    # print("dx="+dx)


def AdjustPosition2():
    global _passoRobot2
    h = 0.478
    # Parametri di camminata del robot

    motionPrx.setMoveArmsEnabled(False, False)
    val = findball(0, 30)
    while val == []:
        motionPrx.moveTo(-0.05, 0, 0, _passoRobot2)
        val = findball(0, 30)
    ballinfo = val[1][1]
    thetah = ballinfo[0]
    thetav = ballinfo[1] + (69.7 * math.pi / 180.0)
    x = (h - 0.03) / (math.tan(thetav)) - 0.11
    y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)
    if (y > 0):
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.05
    else:
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.03
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(x, 0.0, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    motionPrx.moveTo(0.0, y, 0.0,
                     [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])


def colpo():
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
    # time.sleep(0.5)
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.1)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR11,0.15)

    # Aggiungendo programmi
    effectornamelist = ["RWristYaw"]
    timelist = [0.7]
    targetlist = [65 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)


def primoColpo():
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.2)
    # time.sleep(0.5)
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR31, 0.1)
    time.sleep(1)
    # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR11,0.15)

    # Aggiungendo programmi
    effectornamelist = ["RWristYaw"]
    timelist = [2]
    targetlist = [65 * math.pi / 180.0]
    motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)


def TurnAfterHitball1():
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # Parametri di camminata del robot
    global _passoRobot
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, 0.0, 1.6, _passoRobot)
    motionPrx.setMoveArmsEnabled(False, False)
    for i in range(4):
        motionPrx.moveTo(0.25, 0.0, 0.0, _passoRobot)
        time.sleep(0.5)


def rotate():
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    # Parametri di camminata del robot
    global _passoRobot
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.moveTo(0.0, 0.0, -1, _passoRobot)
    motionPrx.moveTo(0.0, 0.2, 0.0, _passoRobot)
    motionPrx.setMoveArmsEnabled(False, False)


def findYellowStick(contourRange=[75, 850]):
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Finding Yellow Stick")
    # 15-20 contains orange
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0)
    videoClient = camProxy.subscribe("python_client", 2, 11, 5)  # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient)
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0]
    imgHeight = rowImgData[1]

    image = np.zeros((imgHeight, imgWidth, 3),
                     dtype='uint8')  # settaggio predefinito di opencv per le colorazioni (BGR)

    image.data = rowImgData[6]

    b, g, r = cv2.split(image)
    img = cv2.merge([r, g, b])

    # deal img
    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    frameBin = cv2.inRange(frameHSV, low, up)
    # kernelErosion = np.ones((5, 5), np.uint8)
    # kernelDilation = np.ones((5, 5), np.uint8)
    # frameBin = cv2.erode(frameBin, kernelErosion, iterations=1)
    # frameBin = cv2.dilate(frameBin, kernelDilation, iterations=1)
    # frameBin = cv2.GaussianBlur(frameBin, (3, 3), 0)
    # frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if len(contours) == 0:
        cv2.imwrite("not_find_stick%s.jpg" % TIME, img)
        print("no Yellow in img")
        return []
    # abandon some yello shape because of width
    good_contour = []

    print (len(contours))

    perimeter = cv2.arcLength(contours[0], True)
    if perimeter <= contourRange[0] or perimeter >= contourRange[1]:
        print ("findYellowStick(): to large or to small")

    ratio = 4
    if perimeter >= 150:
        ratio = 5
    box2D = cv2.minAreaRect(contours[0])
    w = 0
    h = 0
    if (box2D[1][0] < box2D[1][1]):
        w = box2D[1][0]
    else:
        w = box2D[1][1]
    if (box2D[1][0] > box2D[1][1]):
        h = box2D[1][0]
    else:
        h = box2D[1][1]

    print ("findYellowStick(): a yellow", w, h)
    if w * ratio > h:
        good_contour.append(contours[0])

    if len(good_contour) == 0:
        cv2.imwrite("not_find_stick%s.jpg" % TIME, img)
        print("Yellow range width is to large")
        return []
    # find big contour
    maxContour = good_contour[0]
    maxPerimeter = contourRange[0]
    for contour in good_contour:
        perimeter = cv2.arcLength(contour, True)
        if perimeter > maxPerimeter:
            x, y, w, h = cv2.boundingRect(contour)
            print ("w: ", w, h)
            print (perimeter)
            if 2.5 * w > h:
                print ("findYellowStick(): find a hotizontal yellow stick")
                continue
            maxPerimeter = perimeter
            maxContour = contour
    # find alpha
    box2D = cv2.minAreaRect(maxContour)
    if box2D[1][0] > box2D[1][1]:
        w = box2D[1][1]
        h = box2D[1][0]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2

        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = box2D[1][0]
        h = box2D[1][1]
        center = box2D[0]
        theta = -box2D[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 + w * math.cos(theta))
        x = int((x1 + x2) / 2)

        # img è l'immagine, maxcontour sono i contorni, -1 è per disegnare tutti i contorni, colore contorno blu
    cv2.drawContours(img, maxContour, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    # Questo valore è l'angolo orizzontale
    return (320.0 - x) / 640.0 * 60.97 * math.pi / 180


def verticalToYellowStick_withoutMark():
    headAngle = -2 / 3 * math.pi
    maxAngle = 0
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
    time.sleep(0.5)
    alpha = findYellowStick([300, 850])
    while alpha == []:
        headAngle += 1.0 / 3 * math.pi
        if headAngle > maxAngle:
            return -100
        motionPrx.angleInterpolationWithSpeed("HeadYaw", headAngle, 0.5)
        motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        time.sleep(0.5)
        alpha = findYellowStick([300, 850])
    final_headangle = motionPrx.getAngles("HeadYaw", True)
    # Dopo averlo calcolato, gira a sinistra di 90 gradi
    d_angle = final_headangle[0] + alpha + 0.5 * math.pi
    refixRound(d_angle)
    return d_angle


def refixRound(angle):
    motionPrx.angleInterpolationWithSpeed("HeadYaw", 0, 0.5)
    motionPrx.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionPrx.setMoveArmsEnabled(False, False)
    limit = 0.4
    if angle >= 0:
        while angle > limit:
            angle -= limit
            motionPrx.moveTo(0, 0, limit)
            time.sleep(1)
    else:
        while angle < -limit:
            angle += limit
            motionPrx.moveTo(0, 0, -limit)
    motionPrx.moveTo(0, 0, angle)


PORT = 9559
robotIP = "127.0.0.1"
_numRicercaPalle = 0
stop1 = 0
redballFlag = 0
# Andatura e battuta
maxstepx = 0.04
maxstepy = 0.14
maxsteptheta = 0.3
maxstepfrequency = 0.6
stepheight = 0.02
torsowx = 0.0
torsowy = 0.0
_passoRobot = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
               ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
_passoRobot2 = [["MaxStepX", 0.02], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
                ["StepHeight", 0.02], ["TorsoWx", 0], ["TorsoWy", 0]]
PositionJointNamesR = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
PositionJointNamesL = ["LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"]
golfPositionJointAnglesR1 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             0.038397243543875255, 0.12]
golfPositionJointAnglesR2 = [1.4172073526193958, 0.061086523819801536, 1.6022122533307945, 1.4835298641951802,
                             -0.538397243543875255, 0.12]
golfPositionJointAnglesR31 = [1.35787, 0.05760, 1.50098, 1.50971, -0.667945, 0.12]
golfPositionJointAnglesR101 = [1.3, 0.05760, 1.50098, 1.35, 0.767945, 0.12]
golfPositionJointAnglesR111 = [1.35787, 0.05760, 1.50098, 1.50971, -0.738397, 0.12]
golfPositionJointAnglesR11 = [1.35787, 0.05760, 1.50098, 1.50971, -0.538397, 0.12]
golfPositionJointAnglesR32 = [1.35787, 0.05760, 1.50098, 1.50971, -0.767944870877505, 0.12]
golfPositionJointAnglesR12 = [1.35787, 0.05760, 1.50098, 1.50971, 0.767944870877505, 0.12]
golfPositionJointAnglesR42 = [1.02974, 0.24958, 1.61094, 1.10828, -0.43633, 0.12]
golfPositionJointAnglesR5 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.6]
golfPositionJointAnglesR6 = [1.35787, 0.05760, 1.50098, 1.50971, 0.0, 0.12]
golfPositionJointAnglesR7 = [1.02629, 0.314159, 1.62907, 1.48342, 0.230058, 0.12]
golfPositionJointAnglesR8 = [1.18857, -0.67719, 1.17635, 1.52193, 0.666716, 0.50]
golfPositionJointAnglesR9 = [1.47480, -0.17453, 1.18159, 0.41190, 0.10996, 0.12]
golfPositionJointAnglesR10 = [1.46084, 0.26005, -1.37008, -0.08901, -0.02792, 0.12]
GPositionJointNamesR = ["RElbowRoll", "RShoulderRoll", "RShoulderPitch", "RElbowYaw", "RWristYaw", "RHand"]
GgolfPositionJointAnglesR1 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.12]
GgolfPositionJointAnglesR11 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                               -0.538397243543875255, 0.12]
GgolfPositionJointAnglesR2 = [1.4835298641951802, 0.061086523819801536, 1.1, 1.6022122533307945, 0.038397243543875255,
                              0.12]
GgolfPositionJointAnglesR3 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.767944870877505, 0.12]
GgolfPositionJointAnglesR4 = [1.03549, 0.314159, 1.66742, 0.971064, -0.980268, 0.12]
GgolfPositionJointAnglesR5 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.6]
GgolfPositionJointAnglesR6 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.04]

memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
# ------------------------------------------------------------------------------------------------------------#
StiffnessOn(motionPrx)
postureProxy.goToPosture("StandInit", 1.0)
alpha = -math.pi / 2  # valore iniziale

primoColpo()
time.sleep(2)
TurnAfterHitball1()
effectornamelist = ["RWristYaw"]
timelist = [0.7]
targetlist = [65 * math.pi / 180.0]
motionPrx.angleInterpolation(effectornamelist, targetlist, timelist, True)
while True:
    while True:
        allballData = firstricercaredball()  # Posizionamento palla rossa Restituisce informazioni sulla posizione della palla rossa
        if (allballData[2] == 0):
            break
    CalculateRobotToRedball1(allballData)

    AdjustPosition2()
    verticalToYellowStick_withoutMark()

    AdjustPosition2()
    alpha = verticalToYellowStick_withoutMark() - 0.5 * math.pi
    if alpha > 0:
        motionPrx.moveTo(0, 0, 5 / 180.0 * math.pi)
    else:
        motionPrx.moveTo(0, 0, -5 / 180.0 * math.pi)

    AdjustPosition2()

    time.sleep(1)
    rotate()
    colpo()
