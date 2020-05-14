#!/usr/bin/env python
# -*- coding:utf-8 -*-
from naoqi import ALProxy
import time
import math
import cv2
import numpy as np

def StiffnessOn(proxy):  # corpo in posizione rigida
    proxy.stiffnessInterpolation("Body", 1, 1)

def PrimaRicercaPalla():
    global numRicercaPalle, passoRobotLungo
    numRicercaPalle += 1  # viene incrementato il tempo di ricerca ogni secondo
    camProxy.setActiveCamera(1)  # impostazione della camera di sotto del robot

    if numRicercaPalle <= 3:
        rangeRicerca = -60
    else:
        rangeRicerca = -120

    ricerca = rangeRicerca
    while ricerca <= -rangeRicerca:  # finchè non raggiunge l'angolazione opposta continua a richiamare la funzione di ricerca
        risRicercaPalla = trovaPalla(ricerca)  # ricerca palla in base alla posizione del collo
        if risRicercaPalla != []:
            return risRicercaPalla
        else:
            ricerca += 60
    print("Nessuna palla rossa")
    motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionProxy.moveTo(0.25, 0.0, 0.0, passoRobotLungo)  # In cerca di correzione della distanza
    allballDatatest = [0, 0, 1]
    return allballDatatest

def trovaPalla(angolo, angolo2=0):  # Restituisce i dati della palla rossa
    camProxy.setActiveCamera(1)
    # angleinterpolation -> 1)nome del corpo 2)angolo di interpolazione 3)velocita
    # headyaw: rotazione della testa da destra a sinistra o viceversa
    motionProxy.angleInterpolationWithSpeed("HeadYaw", angolo * math.pi / 180,
                                            0.8)  # angolo * math.pi Pi / 180 è la formula di conversione radiante
    # headpitch: inclinazione della testa dal basso verso l'alto e viceversa
    motionProxy.angleInterpolationWithSpeed("HeadPitch", angolo2 * math.pi / 180, 0.8)
    redballProxy.subscribe("redBallDetected")
    # Inserisce una coppia chiave-valore in memoria, dove valore è un int parametri: chiave - Nome del valore da inserire. valore - L'int da inserire
    memoryProxy.insertData("redBallDetected", [])  # Inserisci i dati in memoria

    # Aggiungi il riconoscimento palla rossa e annulla il riconoscimento palla rossa
    time.sleep(2)
    for i in range(10):  # fa l'iterazione 10 volte
        datiPalla = memoryProxy.getData("redBallDetected")  # se trova la palla
        # [TimeStamp,BallInfo,CameraPose_InTorsoFrame,CameraPose_InRobotFrame, Camera_I]

    # redballProxy.unsubscribe("redBallDetected")
    if (datiPalla != []):
        print("La palla rossa è distante")
        print(datiPalla)
        angoloTesta = motionProxy.getAngles("HeadYaw", True)  # salva angolo collo
        InfoPalla = [angoloTesta, datiPalla,
                     0]  # ritorna l'angolo del collo,i dati dell palla e 0 per far finire l'iterazione della ricerca
        return InfoPalla
    else:
        print("Non trovato")
        return []

def DistanzaRobotPalla(datiPalla):
    lunCamminata = 0.478
    # Parametri di camminata del robot
    maxstepx = 0.04
    maxstepy = 0.14
    maxsteptheta = 0.3
    maxstepfrequency = 0.6
    stepheight = 0.02
    torsowx = 0.0
    torsowy = 0.0

    angoloPalla = datiPalla[0]  # Angolo di deflessione della testa
    wzCamera = datiPalla[1][1][0]  # angolo alfa
    wyCamera = datiPalla[1][1][1]  # angolo beta
    isenabled = False
    x = 0.0
    y = 0.0
    if (angoloPalla[0] + wzCamera < 0.0):
        theta = angoloPalla[0] + wzCamera + 0.2  # modifica 1
    else:
        theta = angoloPalla[0] + wzCamera
    # Poi, per la prima volta, il robot si gira verso la palla rossa.
    motionProxy.angleInterpolationWithSpeed("HeadYaw", 0.0, 0.5)
    motionProxy.moveTo(x, y, theta,
                       [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])  # x=y=0

    time.sleep(1.5)
    currPalla = memoryProxy.getData("redBallDetected") #dati correnti palla dopo che si è spostato
    infoPalla = currPalla[1]
    thetah = infoPalla[0]
    thetav = infoPalla[1] + (40 * math.pi / 180.0) #centro della palla + 40 gradi
    x = lunCamminata / (math.tan(thetav)) - 0.5  # Minimo 40 cm
    if (x >= 0):
        theta = 0.0

        print("Mosso in direzione della palla rossa")
        # Successivamente, per la seconda volta, il robot cammina in una posizione a 20 cm dalla palla rossa,
        # con un programma di 40 cm, perché il robot fa troppi errori nella camminata reale per calciare la palla
        motionProxy.moveTo(x, y, theta,
                           [["MaxStepX", maxstepx],
                          ["MaxStepY", maxstepy],
                          ["MaxStepTheta", maxsteptheta],
                          ["MaxStepFrequency", maxstepfrequency],
                          ["StepHeight", stepheight],
                          ["TorsoWx", torsowx],
                          ["TorsoWy", torsowy]])
    motionProxy.waitUntilMoveIsFinished()
    # Abbassa la testa di 30 gradi.
    effectornamelist = ["HeadPitch"]
    timelist = [0.5]
    targetlist = [30 * math.pi / 180.0]
    motionProxy.angleInterpolation(effectornamelist, targetlist, timelist, isenabled)
    time.sleep(1.5)
    currPalla = memoryProxy.getData("redBallDetected")
    infoPalla = currPalla[1]
    thetah = infoPalla[0]
    thetav = infoPalla[1] + (70 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah

    print("Camminare fino a 20cm dalla palla rossa")
    # Poi, per la terza volta, il robot corregge l'angolo alla palla rossa.
    motionProxy.moveTo(x, y, theta,
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

    currPalla = memoryProxy.getData("redBallDetected")
    infoPalla = currPalla[1]
    thetah = infoPalla[0]
    thetav = infoPalla[1] + (70 * math.pi / 180.0)
    x = (lunCamminata - 0.03) / (math.tan(thetav)) - 0.15  # La linea a tre punti, in ultima analisi, rivede i punti chiave
    theta = thetah
    print("Correzione dell'angolo completata")
    # Poi, per la quarta volta, il robot cammina a 10 centimetri dalla palla rossa
    motionProxy.moveTo(x, y, theta,
                       [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    print("Raggiunti i 10cm")
    currPalla = memoryProxy.getData("redBallDetected")
    infoPalla = currPalla[1]
    thetah = infoPalla[0]
    print(thetah)
    thetav = infoPalla[1] + (70 * math.pi / 180.0)
    x = 0.0
    y = 0.0
    theta = thetah
    print("Cammino fino a 20cm dalla palla rossa")
    # Poi, per la quinta volta, il robot ha finalmente corretto l'angolo con la palla rossa
    motionProxy.moveTo(x, y, theta,
                       [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])
    time.sleep(1.5)
    # Successivamente, per la sesta e ultima volta, rilevare la distanza tra la palla e il robot dx
    currPalla = memoryProxy.getData("redBallDetected")
    infoPalla = currPalla[1]
    thetah = infoPalla[0]
    thetav = infoPalla[1] + (70 * math.pi / 180.0)
    dx = (lunCamminata - 0.03) / (math.tan(thetav))  # dx come lato di un triangolo
    print (dx)
    return dx
    # print("dx="+dx)

def correzionePos():
    global passoRobotCorto
    h = 0.478

    val = trovaPalla(0, 30)
    while val == []:
        motionProxy.moveTo(0.1, 0, 0, passoRobotCorto)
        val = trovaPalla(0, 30)
    ballinfo = val[1][1]
    thetah = ballinfo[0] #center X
    thetav = ballinfo[1] + (70 * math.pi / 180.0) #center Y
    x = (h - 0.03) / (math.tan(thetav)) - 0.11
    y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah)
    if (y > 0):
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.05
    else:
        y = ((h - 0.03) / math.sin(thetav)) * math.tan(thetah) + 0.03

    motionProxy.moveTo(x, 0.0, 0.0,
                       [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

    motionProxy.moveTo(0.0, y, 0.0,
                       [["MaxStepX", maxstepx],
                      ["MaxStepY", maxstepy],
                      ["MaxStepTheta", maxsteptheta],
                      ["MaxStepFrequency", maxstepfrequency],
                      ["StepHeight", stepheight],
                      ["TorsoWx", torsowx],
                      ["TorsoWy", torsowy]])

def colpo():
    motionProxy.angleInterpolationWithSpeed(NomiArtoDestro, sollevaBraccio, 0.2)
    time.sleep(1)
    motionProxy.angleInterpolationWithSpeed(NomiArtoDestro, inclinaPolso, 0.1)
    time.sleep(1)
    wristYaw = ["RWristYaw"]
    tempo = [0.7]
    angolazione = [30 * math.pi / 180.0]
    motionProxy.angleInterpolation(wristYaw, angolazione, tempo, True)

def colpoInizio():
    motionProxy.angleInterpolationWithSpeed(NomiArtoDestro, sollevaBraccio, 0.2)
    time.sleep(1)
    motionProxy.angleInterpolationWithSpeed(NomiArtoDestro, inclinaPolso, 0.1)
    time.sleep(1)
    wristYaw = ["RWristYaw"]
    tempo = [2]
    angolazione = [65 * math.pi / 180.0]
    motionProxy.angleInterpolation(wristYaw, angolazione, tempo, True)

def RotazioneRobot():

    # Parametri di camminata del robot
    global passoRobotLungo
    motionProxy.moveTo(0.0, 0.0, 1.8, passoRobotLungo)
    for i in range(4):
        motionProxy.moveTo(0.25, 0.0, 0.0, passoRobotLungo)
        time.sleep(0.5)

def trovaAsta(rangeDimAsta=[75, 850]):
    TIME = time.strftime('%m-%d_%H-%M-%S', time.localtime(time.time()))
    print("Cercando l'asta")

    #range colore giallo
    low = np.array([15, 80, 80])
    up = np.array([36, 255, 255])
    camProxy.setActiveCamera(0) #camera alta
    videoClient = camProxy.subscribe("python_client", 2, 11, 5)  # 640*480，RGB, FPS
    rowImgData = camProxy.getImageRemote(videoClient) #restituisce info immagine catturata
    camProxy.unsubscribe(videoClient)
    imgWidth = rowImgData[0] #lunghezza immagine
    imgHeight = rowImgData[1] #altezza immagine

    image = np.zeros((imgHeight, imgWidth, 3), dtype='uint8')  # settaggio predefinito di opencv per le colorazioni (BGR)

    image.data = rowImgData[6] #array of size height * width * nblayers containing image data

    b, g, r = cv2.split(image) #divide immagine nei tre canali e li assegna
    img = cv2.merge([r, g, b]) #riunisce i canali in una immagine

    frameHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #viene settato il colore dell'immagine da BGR a HSV
    frameBin = cv2.inRange(frameHSV, low, up)

    #frameBin debug
    cv2.imwrite("bin.jpg", frameBin)
    _, contours, _ = cv2.findContours(frameBin, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) #restituisce i contorni dell'asta
    if len(contours) == 0:
        cv2.imwrite("asta_non_trovata%s.jpg" % TIME, img)
        print("Giallo non trovato in img")
        return []

    contornoValido = []

    perimetroAsta = cv2.arcLength(contours[0], True) #calcolo della lunghezza del perimetro dell'asta
    if perimetroAsta <= rangeDimAsta[0] or perimetroAsta >= rangeDimAsta[1]:
        print ("trovaAsta(): troppo grande o piccola")



    rettangoloAsta = cv2.minAreaRect(contours[0]) #calcola l'area dell'asta
    w = 0
    h = 0

    if (rettangoloAsta[1][0] < rettangoloAsta[1][1]):
        w = rettangoloAsta[1][0]
    else:
        w = rettangoloAsta[1][1]

    if (rettangoloAsta[1][0] > rettangoloAsta[1][1]):
        h = rettangoloAsta[1][0]
    else:
        h = rettangoloAsta[1][1]

    print ("trovaAsta(): asta trovata", w, h)
    contornoValido.append(contours[0])

    if len(contornoValido) == 0:
        cv2.imwrite("asta_non_trovata%s.jpg" % TIME, img)
        print("l'asta è troppo grande")
        return []

    rettangoloAsta = cv2.minAreaRect(contornoValido[0])
    if rettangoloAsta[1][0] > rettangoloAsta[1][1]:
        w = rettangoloAsta[1][1]
        h = rettangoloAsta[1][0]
        center = rettangoloAsta[0]
        theta = -rettangoloAsta[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * w / h)
        bevel = math.sqrt(w * w + h * h) / 2

        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 - w * math.sin(theta))
        x = int((x1 + x2) / 2)
    else:
        w = rettangoloAsta[1][0]
        h = rettangoloAsta[1][1]
        center = rettangoloAsta[0]
        theta = -rettangoloAsta[2] * math.pi / 180
        alpha = theta + math.atan(1.0 * h / w)
        bevel = math.sqrt(w * w + h * h) / 2
        x1 = int(center[0] - bevel * math.cos(alpha))
        x2 = int(x1 + w * math.cos(theta))
        x = int((x1 + x2) / 2)

        # img è l'immagine, maxcontour sono i contorni, -1 è per disegnare tutti i contorni, colore contorno blu
    contorno = contornoValido[0]
    #cv2.drawContours(img, contorno, -1, (0, 0, 255), 1)
    cv2.imwrite("ok.jpg", img)
    # Questo valore è l'angolo orizzontale
    return (320.0 - x) / 640.0 * 61 * math.pi / 180

def CalcolaPosAsta():
    angoloTesta = -2 / 3 * math.pi # -120 gradi
    maxAngle = 0
    motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    motionProxy.angleInterpolationWithSpeed("HeadYaw", angoloTesta, 0.5)
    time.sleep(0.5)
    angoloAsta = trovaAsta([300, 850])
    while angoloAsta == []:
        angoloTesta += 1.0 / 3 * math.pi #+60 gradi
        if angoloTesta > maxAngle:
            return -100
        motionProxy.angleInterpolationWithSpeed("HeadYaw", angoloTesta, 0.5)
        motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
        time.sleep(0.5)
        angoloAsta = trovaAsta([300, 850])
    angoloRobot = motionProxy.getAngles("HeadYaw", True)
    # Dopo averlo calcolato, gira a sinistra di 90 gradi
    angoloFinale = angoloRobot[0] + angoloAsta + 0.5 * math.pi
    rotPostAsta(angoloFinale)
    return angoloFinale

def rotPostAsta(angolo):
    motionProxy.angleInterpolationWithSpeed("HeadYaw", 0, 0.5)
    motionProxy.angleInterpolationWithSpeed("HeadPitch", 0, 0.5)
    z = 0.4
    if angolo >= 0:
        while angolo > z:
            angolo -= z
            motionProxy.moveTo(0, 0, z)
            time.sleep(1)
    else:
        while angolo < -z:
            angolo += z
            motionProxy.moveTo(0, 0, -z)
    motionProxy.moveTo(0, 0, angolo)

PORT = 9559
robotIP = "127.0.0.1"
memoryProxy = ALProxy("ALMemory", robotIP, PORT)
motionProxy = ALProxy("ALMotion", robotIP, PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)
redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
camProxy = ALProxy("ALVideoDevice", robotIP, PORT)

numRicercaPalle = 0
# Andatura e battuta
maxstepx = 0.04
maxstepy = 0.14
maxsteptheta = 0.3
maxstepfrequency = 0.6
stepheight = 0.018
torsowx = 0.0
torsowy = 0.0
passoRobotLungo = [["MaxStepX", 0.04], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
                   ["StepHeight", 0.018], ["TorsoWx", 0], ["TorsoWy", 0]]
passoRobotCorto = [["MaxStepX", 0.02], ["MaxStepY", 0.14], ["MaxStepTheta", 0.3], ["MaxStepFrequency", 0.6],
                   ["StepHeight", 0.018], ["TorsoWx", 0], ["TorsoWy", 0]]
NomiArtoDestro = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"]
inclinaPolso = [1.35787, 0.05760, 1.50098, 1.50971, -0.667945, 0.12]
sollevaBraccio = [1.02629, 0.314159, 1.62907, 1.48342, 0.230058, 0.12]

# ------------------------------------------------------------------------------------------------------------#
StiffnessOn(motionProxy)
postureProxy.goToPosture("StandInit", 1.0)
alpha = -math.pi / 2  # valore iniziale

colpoInizio()
time.sleep(2)
RotazioneRobot()

while True:
    while True:
        allballData = PrimaRicercaPalla()  # Posizionamento palla rossa Restituisce informazioni sulla posizione della palla rossa
        if (allballData[2] == 0):
            break
    DistanzaRobotPalla(allballData)

    correzionePos()
    CalcolaPosAsta()

    correzionePos()
    alpha = CalcolaPosAsta() - 0.5 * math.pi
    if alpha > 0:
        motionProxy.moveTo(0, 0, 5 / 180.0 * math.pi)
    else:
        motionProxy.moveTo(0, 0, -5 / 180.0 * math.pi)

    correzionePos()
    time.sleep(1)
    colpo()
