#!/usr/bin/env python
# -*- coding:utf-8 -*-
from naoqi import ALProxy
import almath
import time
import math
import motion
import sys
import thread


def StiffnessOn(proxy):  # corpo in posizione rigida
    proxy.stiffnessInterpolation("Body", 1, 1)


def Grab():
    while True:
        # RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
        headTouchedmidlleFlag = memoryProxy.getData("MiddleTactilTouched")
        if headTouchedmidlleFlag == 1.0:
            print "right hand touched"
            tts.say("Dammi una mazza da golf")
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4)
            time.sleep(5)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.1)
            time.sleep(3)
            # motionPrx.angleInterpolationWithSpeed(PositionJointNamesR,golfPositionJointAnglesR2,0.4)
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4)  # era commentato
            motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4)  # era commentato
            break


# def Grab():
#     while True:
#         RighthandTouchedFlag = memoryProxy.getData("HandRightRightTouched")
#         if RighthandTouchedFlag == 1.0:
#             print "right hand touched"
#             # tts.say("Dammi una mazza da golf")
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR5, 0.4);
#             time.sleep(4)
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR6, 0.4);
#             time.sleep(3)
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR2, 0.4);
#             motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR1, 0.4);
#             break

def posizionamento():
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("HeadYaw")  # movimento dell'asse Z
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LAnklePitch")  # Asse Z caviglia
    times.append([1, 2, 3, 4])
    keys.append([-0.349794, -0.349794, -0.349794, -0.349794])

    names.append("LAnkleRoll")  # Asse X caviglia
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LElbowRoll")  # Asse Z del gomito
    times.append([1, 2, 3, 4])
    keys.append([-0.321141, -0.321141, -1.1, -1.1])

    names.append("LElbowYaw")  # Asse X
    times.append([1, 2, 3, 4])
    keys.append([-1.37757, -1.37757, -1.466076, -1.466076])

    names.append("LHand")  # Palmo sinistro
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.9800, 0.9800, 0.9800, 0.9800, 0.1800])

    names.append("LHipPitch")  # Asse Y della gamba
    times.append([1, 2, 3, 4])
    keys.append([-0.450955, -0.450955, -0.450955, -0.450955])

    names.append("LHipRoll")  # Asse X della gamba
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LHipYawPitch")  # 啥关节
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("LKneePitch")  # Asse Y del ginocchio
    times.append([1, 2, 3, 4])
    keys.append([0.699462, 0.699462, 0.699462, 0.699462])
    names.append("LShoulderPitch")  # Asse Y del ginocchio
    times.append([1, 2, 3, 4, 5.2])
    ##------------------------------------------------------------
    keys.append([1.53885, 1.43885, 1.3, 1.3, 1.3])

    names.append("LShoulderRoll")  # Asse Z della spalla
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.268407, 0.268407, -0.04014, -0.04014, -0.04014])

    names.append("LWristYaw")  # Asse X del polso

    times.append([1, 2, 3, 4])
    keys.append([-0.016916, -0.016916, -1.632374, -1.632374])

    names.append("RAnklePitch")  # Asse Y Caviglia
    times.append([1, 2, 3, 4])
    keys.append([-0.354312, -0.354312, -0.354312, -0.354312])

    names.append("RAnkleRoll")  # Asse X Caviglia
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RElbowRoll")  # Asse Z del gomito
    times.append([1, 2, 3, 4])
    keys.append([0.958791, 0.958791, 0.958791, 0.958791])

    names.append("RElbowYaw")  # Asse X del gomito
    times.append([1, 2, 3, 4])
    keys.append([1.466076, 1.466076, 1.466076, 1.466076])

    names.append("RHand")
    times.append([1, 2, 3, 4])
    keys.append([0.0900, 0.0900, 0.0900, 0.0900])

    names.append("RHipPitch")  # Asse Y della gamba
    times.append([1, 2, 3, 4])
    keys.append([-0.451038, -0.451038, -0.451038, -0.451038])

    names.append("RHipRoll")  # Asse X della gamba
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RHipYawPitch")
    times.append([1, 2, 3, 4])
    keys.append([0, 0, 0, 0])

    names.append("RKneePitch")  # Asse Y del ginocchio
    times.append([1, 2, 3, 4])
    keys.append([0.699545, 0.699545, 0.699545, 0.699545])

    names.append("RShoulderPitch")  # Asse Y della spalla
    times.append([0.5, 1, 2, 3, 4, 5.2])
    # keys.append([1.03856, 1.03856, 1.03856, 1.03856, 1.03856])
    keys.append([0.9, 1.03856, 1.03856, 1.03856, 1.03856, 1.03856])

    names.append("RShoulderRoll")  # Asse Z della spalla
    times.append([1, 2, 3, 4, 5.2])
    keys.append([0.04014, 0.04014, 0.04014, 0.04014, 0.04014])

    names.append("RWristYaw")  # Asse X del polso
    times.append([1, 2, 3, 4])
    keys.append([1.632374, 1.632374, 1.632374, 1.632374])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)


def LShoulderpitchAmend():  # correzione della spalla sinistra
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  # aggiuge alla lista la spalla sinistra
    times.append([1])  # tempo
    keys.append([1.03856])  # angolazione
    names.append("LHand")  # aggiunge alla lista la mano sinistra
    times.append([1, 2, 3, 4])
    keys.append([0.0200, 0.0200, 0.0200, 0.0200])
    motionPrx.setMoveArmsEnabled(False, False)  # movimento braccia disabilitato
    motionPrx.angleInterpolation(names, keys, times, True)


def ShoulderpitchAmend2():
    names = list()
    keys = list()
    times = list()
    names.append("LShoulderPitch")  # avvicina a se la mazza(ruota la spalla sinistra)
    times.append([0.5, 1])
    keys.append([1.43856, 1.88495559])
    names.append("RShoulderPitch")  # avvicina a se la mazza(ruota la spalla destra)
    times.append([0.5, 1])
    keys.append([1.43856, 1.88495559])
    names.append("LElbowRoll")  # ruota il gomito sinistro inbasso
    times.append([0.5, 1])
    keys.append([-1.23490659, -1.51843645])
    names.append("RElbowRoll")  # ruota il gomito destra in basso
    times.append([0.5, 1])
    keys.append([1.23490659, 1.51843645])
    motionPrx.setMoveArmsEnabled(False, False)
    motionPrx.angleInterpolation(names, keys, times, True)


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
        # Successivamente, per la seconda volta, il robot cammina in una posizione a 20 cm dalla palla rossa, con un programma di 40 cm, perché il robot fa troppi errori nella camminata reale per calciare la palla
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
    print dx
    return dx
    # print("dx="+dx)


def AdjustPosition2():
    global _passoRobot2
    h = 0.478
    # Parametri di camminata del robot

    # motionPrx.angleInterpolationWithSpeed("HeadPitch", 0.0, 0.3)
    motionPrx.setMoveArmsEnabled(False, False)
    # Aggiungere un paragrafo sulla procedura
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


def colpoInverso():
    motionPrx.angleInterpolationWithSpeed(PositionJointNamesR, golfPositionJointAnglesR7, 0.2)
    time.sleep(1)


PORT = 9559
# robotIP = "192.168.43.35"  #ip del robot
robotIP = "127.0.0.1"
_numRicercaPalle = 0
stop1 = 0
# 0代表正常,非零则没有识别
landmarkFlag = 0
redballFlag = 0
# 步态和击球姿式
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
                              0.038397243543875255, 0.6]  # 松杆参数
GgolfPositionJointAnglesR6 = [1.4835298641951802, 0.061086523819801536, 1.4172073526193958, 1.6022122533307945,
                              0.038397243543875255, 0.04]  # 抓杆参数

memoryProxy = ALProxy("ALMemory", robotIP, PORT)  # memory  object
motionPrx = ALProxy("ALMotion", robotIP, PORT)  # motion  object
tts = ALProxy("ALTextToSpeech", robotIP, PORT)
postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

# Lifestop = ALProxy("ALAutonomousLife", robotIP, PORT)
# Lifestop.setState("disabled")

redballProxy = ALProxy("ALRedBallDetection", robotIP, PORT)
camProxy = ALProxy("ALVideoDevice", robotIP, PORT)
# landmarkProxy = ALProxy("ALLandMarkDetection", robotIP, PORT)
# ------------------------------------------------------------------------------------------------------------#
StiffnessOn(motionPrx)
postureProxy.goToPosture("StandInit", 1.0)
alpha = -math.pi / 2  # valore iniziale
colpo()
time.sleep(2)
while True:
    allballData = firstricercaredball()  # Posizionamento palla rossa Restituisce informazioni sulla posizione della palla rossa
    if (allballData[2] == 0):
        break
CalculateRobotToRedball1(allballData)
AdjustPosition2()
colpoInverso()
colpo()
