import os
from scipy.optimize import lsq_linear
import socket
import time
import numpy as np
from numpy import random
import math
import csv
import telnetlib

# THIS IS TO ESTABLISH COMMUNICATION WITH UNITY - YOU DO NOT NEED THIS
# =============================================================================
host, port = '127.0.0.1', 65432
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))
# Message for indicating if we need continous or not continuous
first_msg = int(input(" 1 for continuous, 2 for not continuous: "))
if first_msg == 1:
    sock.sendall("continuous".encode('utf-8'))
else:
    sock.sendall("not".encode('utf-8'))
# =============================================================================

scale = 1  # Scale the light intensity by this factor
rgbSpot = [255, 255, 0]
rgbEdge = [0, 255, 255]

# This function is to transform the multis from the unity orientation to the physical troffer orientation
def transformMultisUnityToTroffer(listMultis):
    # Scale the multis to 0-4095 and round to integers
    listIntMultis = [round((4095*i*scale), 0) for i in listMultis]
    array = np.array(listIntMultis)
    # Splitting the array into two parts
    edgeMultis = array[:4]
    print(edgeMultis)
    pixelMultis = array[4:]
    pixelMatrix = pixelMultis.reshape((7, 7))
    pixelMatrix = np.flip(pixelMatrix, axis=0)  # Flip the pixel matrix upside down
    print(pixelMatrix)
    mergedMultis = np.concatenate((edgeMultis, pixelMatrix.flatten()), axis=0)
    mergedMultisList = mergedMultis.tolist()  # Flatten the matrix to a list
    return mergedMultisList

# This function is to rotate the multis for the troffer when the troffer is rotated
# The listMultis is the list of multis for the troffer and the numRot90 is the angle of clock-wise rotation divided by 90
def rotateMultisForTroffer(listMultis, numRot90):
    array = np.array(listMultis)
    # Splitting the array into two parts
    edgeMultis = array[:4]
    pixelMultis = array[4:]
    rotatedEdgeMultis = np.rot90(edgeMultis.reshape((2, 2)), numRot90)
    rotatedPixelMultis = np.rot90(pixelMultis.reshape((7, 7)), numRot90)
    mergedMultis = np.concatenate((rotatedEdgeMultis.flatten(), rotatedPixelMultis.flatten()), axis=0)
    rotatedMultis = mergedMultis.tolist()  # Flatten the matrix to a list
    return rotatedMultis

def sendMultisToArduino(listMultis1, listMultis2):
    print(listMultis1)
    print(listMultis2)
    # Telnet server IP address and port number
    HOST = ["192.168.0.80", "192.168.0.81", "192.168.0.61", "192.168.0.57"]
    PORT = 23  # Telnet server port number

    # Connect to Telnet servers
    # tn1Right = telnetlib.Telnet(HOST[0], PORT)
    # tn1Left = telnetlib.Telnet(HOST[1], PORT)
    tn2Right = telnetlib.Telnet(HOST[2], PORT)
    tn2Left = telnetlib.Telnet(HOST[3], PORT)

    smoother = 200  # To control how smooth the transition is

    array1 = listMultis1
    array2 = listMultis2

    # Transform multis to troffer orientation
    array1 = transformMultisUnityToTroffer(array1)
    array2 = transformMultisUnityToTroffer(array2)

    # Rotated troffer multis
    # array1 = rotateMultisForTroffer(array1, 1)

    array2.insert(0, rgbEdge[2])
    array2.insert(0, rgbEdge[1])
    array2.insert(0, rgbEdge[0])

    array2.insert(0, rgbSpot[2])
    array2.insert(0, rgbSpot[1])
    array2.insert(0, rgbSpot[0])

    array1.insert(0, smoother)
    array2.insert(0, smoother)
    
    # Send the array to Arduino as a string
    # tn1Right.write(str(array1).encode('utf-8'))
    # tn1Left.write(str(array1).encode('utf-8'))
    tn2Right.write(str(array2).encode('utf-8'))
    tn2Left.write(str(array2).encode('utf-8'))

    # Closing all the connections
    # tn1Right.close()
    # tn1Left.close()
    tn2Right.close()
    tn2Left.close()

# =============================================================================

# DEFINE FUNCTIONS

# AWAKE --------------------------------------------------------------------

# PREP GAZE MATH -----------------------------------------------------------

# READS 53 LIGHT VECTORS AT ORIGIN (FOR GAZE/GLARE MATH)
# INPUT: csv file path
# OUTPUT: dictionary of 53 light vectors at unity origin
def read_store_inDict_LightVectors_atOrigin(readLightVectorsPath):

    openFilePath = readLightVectorsPath
    lineCounter = 0
    dictLightVectors53 = {}
    # listLightVectorValues = []
    with open(openFilePath) as fp1:
        Lines = fp1.readlines()
        for line in Lines:
            lineCounter += 1
            if (lineCounter > 1):
                stringsList = line.split(",")
                coords = []
                for i, item in enumerate(stringsList):
                    if (i == 0):
                        dictKey = item
                    if (i == 1):
                        x = float(item)
                    if (i == 2):
                        y = float(item)
                    if (i == 3):
                        z = float(item)
                # Accounts for Unity Coords Conversion!
                coords = np.array([-x, z, -y])
                dictLightVectors53[dictKey] = coords

    return dictLightVectors53

# EXPORTS 53 LIGHT VECTORS AS TXT (PROBABLY WON'T NEED IT TO COMPUTE MULTIPLIERS)
# INPUT: csv file path
# OUTPUT: dictionary of 53 light vectors at unity origin


def export_Light_Vectors(dictLightVectors53, exportLightVectorsPath):

    Lines = []
    # print(f'The shape of the lv is {listLightVectors53[25].shape}')
    for key, lv in dictLightVectors53.items():
        line = str(lv[0]) + " " + str(lv[1]) + " " + str(lv[2]) + "\n"
        Lines.append(line)

    with open(exportLightVectorsPath, 'w') as fp:
        fp.writelines(Lines)

    return

# COMPUTES RESTRICTED UPPER BOUNDS FOR BVLS SOLVER (TO ACCOUNT FOR GAZE/GLARE MATH)
# INPUTS:
# listGazeVectors: occupant gaze vectors
# dictLightVectors53: dictionary of 53 light vectors
# # Define Angle Threshold, Max, Min Values
# th1,th2,gthMax,gthMin # glare math threshold values
# th1,th2,gthMax,gthMin = 120,165,1,0 # these values are assigned by default at START section of code
# listArrayOccupantCoords = occupant coordinates
# listTrofferOrigins = list of 8 troffer origin coordinates - unity coordinates
# x1,x2,gmax # threshold values
# x1,x2,gmax = 1,2,1 # these values are assigned by default at START section of code
# OUTPUTS:
# listMinAngleGlareMultis: list of bounds only based on angle calculations (you won't need that)
# listMinGlareMultis: list of bounds based on all glare math parameters
# dictGlareMultisTroffersAll: dictionary of restricted upper bounds for all troffers (use that)


def angleAndDistance_gaze_math(listGazeVectors, dictLightVectors53, th1, th2, gthMax, gthMin, listArrayOccupantCoords, listTrofferOrigins, x1, x2, gmax):

    print(
        f'Gaze Vector 1 is {listGazeVectors[0]} and type is {type(listGazeVectors[0])}')
    print(
        f'Occupant Coords 1 is {listArrayOccupantCoords[0]} and type is {type(listArrayOccupantCoords[0])}')
    print(
        f'Troffer Origin 1 is {listTrofferOrigins[0]} and type is {type(listTrofferOrigins[0])}')

    # BASED ON ANGLE
    listANGLEOccupantsAllGlareMultis = []
    listOccupantsAllGlareMultis = []

    for i, gv in enumerate(listGazeVectors):

        angleMultisOccupantX = []
        angleBtwList = []

        for key, lv in dictLightVectors53.items():
            # gvNorm = gv / np.linalg.norm(gv)
            # lvNorm = lv / np.linalg.norm(lv)
            # calculate angle between two vectors
            angleBtw = np.rad2deg(np.arccos(np.dot(gv, lv)))
            angleBtwList.append(angleBtw)

            # check if angle is acute
            if (angleBtw > 180):
                angleBtw = 360 - angleBtw

            # check if ANGLE and DISTANCE cause glare
            if (angleBtw < th1):
                gth = 1
                angleMultisOccupantX.append(gth)
            elif (angleBtw > th2):
                gth = 0
                angleMultisOccupantX.append(gth)
            else:
                # linear behavior
                gth = ((th2 - angleBtw) * (gthMax - gthMin) /
                       (th2 - th1)) + gthMin
                # cosinusoidal behavior
                # gth = (np.cos(np.radians(math.pi - ((th2 - angleBtw) * math.pi / (th2 - th1))) * ((gthMax - gthMin) / 2))) + ((gthMax + gthMin) / 2)
                angleMultisOccupantX.append(gth)
        print(f'Angles Between Are: {angleBtwList}')

        # BASED ON ANGLE AND DISTANCE

        # plane-line intersection math
        #   l: line vector - light vector
        #   l0: point on line - troffer origin
        #   n: plane normal vector - gaze vector
        #   p0: point on plane - occupant coords

        glareMultisOccupantX = []
        intPoints = []

        n = gv  # input no1
        p0 = listArrayOccupantCoords[i]  # input no2
        tr = 0
        pixel = 0
        for tr in range(8):  # number of troffers
            l0 = listTrofferOrigins[tr]  # input no3
            for key, lv in dictLightVectors53.items():  # number of elements per troffer
                a = pixel % 53
                pixel += 1
                if (angleMultisOccupantX[a] == 1):
                    g = 1
                    glareMultisOccupantX.append(g)
                else:
                    l = lv  # input no4
                    if (np.dot(l, n) == 0):
                        if (np.dot((p0-l0), n) == 0):
                            print("Light Vector Line lies on Gaze Vector Plane")
                        else:
                            print(
                                "Light Vector Line and Gaze Vector Plane do not Intersect")
                        g = None
                        glareMultisOccupantX.append(g)
                    else:
                        d = np.dot((p0-l0), n)/np.dot(l, n)
                        p = l0 + l*d
                        intPoints.append(p[1])
                        # check if intersection point is above troffer
                        if (p[1] > listTrofferOrigins[0][1]):  # if above
                            g = 1
                            glareMultisOccupantX.append(g)
                        else:  # if below
                            # print(f'Light Vector Line Intersects Gaze Vector Plane on a Single Point P{p}')
                            # measure distance btw p and occupant eyes
                            dx = math.dist(p, listArrayOccupantCoords[i])
                            # calculate g based on dx
                            gth = angleMultisOccupantX[a]
                            if (dx > 0 and dx < x1):
                                g = gth
                            elif (dx > x2):
                                g = 1
                            elif (dx >= x1 and dx <= x2):
                                # linear behavior
                                g = ((dx - x1) * (gmax - gth) / (x2 - x1)) + gth
                                # cosinusoidal behavior
                                # g = (np.cos(np.radians(math.pi - ((dx - x1) * math.pi / (x2 - x1))) * ((gmax - gth) / 2))) + ((gmax + gth) / 2)
                            if (g != 0):
                                glareMultisOccupantX.append(g)
                            else:
                                glareMultisOccupantX.append(0.0001)

        listANGLEOccupantsAllGlareMultis.append(angleMultisOccupantX)
        listOccupantsAllGlareMultis.append(glareMultisOccupantX)
        # print(f'Angle Glare Multis Per Occupant: {angleMultisOccupantX}')
        # print(f'Glare Multis Per Occupant: {glareMultisOccupantX}')
        print(f'Coords of Int Point: {intPoints}')

    # Compare ANGLE glare multis per occupant
    # Keep min ANGLE glare multis per lighting element
    listMinAngleGlareMultis = []
    for i in range(len(listANGLEOccupantsAllGlareMultis[0])):
        elementwise = []
        for l in listANGLEOccupantsAllGlareMultis:
            elementwise.append(l[i])
        listMinAngleGlareMultis.append(min(elementwise))

    print(f'Angle Glare Multis length is: {len(listMinAngleGlareMultis)}')

    # Compare glare multis per occupant
    # Keep min glare multis per lighting element
    listMinGlareMultis = []
    for i in range(len(listOccupantsAllGlareMultis[0])):
        elementwise = []
        for l in listOccupantsAllGlareMultis:
            elementwise.append(l[i])
        listMinGlareMultis.append(min(elementwise))

    # Reconstruct Glare Multis from Vector(424) to Dict(dict(53))
    c = 0
    dictGlareMultisTroffersAll = {}
    for i in range(8):  # number of troffers
        k = "Troffer " + str(i)
        dictGlareMultisTrofferX = {}
        for key, lv in dictLightVectors53.items():  # number of elements per troffer
            dictGlareMultisTrofferX[key] = listMinGlareMultis[c]
            c += 1
        dictGlareMultisTroffersAll[k] = dictGlareMultisTrofferX

    print(f'The number of minDistAngleMultis is: {len(listMinGlareMultis)}')

    return listMinAngleGlareMultis, listMinGlareMultis, dictGlareMultisTroffersAll


# PREP AUTOMATIONS FOR ECO PRO PLUS (ENERGY EFFICIENCY) (BUBBLE ALGORITHM) --------------------
# Accounts for Unity Coords Conversion!
# READS SAMPLE POINTS FROM CSV
# INPUT: CSV FILE PATH
# OUTPUTS:
# dictSamplePointCoords: dictionary of coords for all sample points
# dictSamplePointFeatureID: dictionary of feature IDs for all sample points
def read_store_sample_points_asDict(samplePointsFilePath):

    openFilePath = samplePointsFilePath
    lineCounter = 0
    dictSamplePointCoords = {}
    dictSamplePointFeatureID = {}
    with open(openFilePath) as fp1:
        Lines = fp1.readlines()
        for line in Lines:
            lineCounter += 1
            if (lineCounter > 1):
                stringsList = line.split(",")
                coords = []
                for i, item in enumerate(stringsList):
                    if (i == 0):
                        dictKey = item
                    if (i == 1):
                        x = float(item)
                    if (i == 2):
                        y = float(item)
                    if (i == 3):
                        z = float(item)
                    if (i == 4):
                        feature = item.strip()
                coords = [-x, z, -y]  # Accounts for Unity Coords Conversion!
                dictSamplePointCoords[dictKey] = coords
                dictSamplePointFeatureID[dictKey] = feature

    return dictSamplePointCoords, dictSamplePointFeatureID

# COMPUTES A TARGET LUX VECTOR THAT ACCOUNTS FOR OCCUPANTS - BUBBLE ALGORITHM
# INPUTS:
# dictSPcoords: dictionary with all sample point coordinates
# tlvOG: target lux vector of general room activity (not energy saving TLV - first (eco) column from csv)
# tlvES: energy saving target lux vector of general room activity (energy saving TLV - second (eco++) or third (eco+) column of csv)
# listOccupantCoords: list of all occupant coordinates
# thresholdsR1R2: bubble math threshold values
# thresholdsR1R2 = [0.5,2] # default values for (R1,R2) as assigned at START section of code
# dimmerValue: defaults to a value of 1 at START section of code
# OUTPUT: arrayHumanAwareTLVs: TARGET LUX VECTOR THAT ACCOUNTS FOR OCCUPANTS


def humanAware_TLVs(dictSPcoords, tlvOG, tlvES, listOccupantCoords, thresholdsR1R2, dimmerValue):

    arrayHumanAwareTLVs = np.empty([1215,])

    if (len(listOccupantCoords) == 0):  # no person in the room
        arrayHumanAwareTLVs = tlvES
        print('No people present. HumanAwareness assigned tlvES values everywhere.')

    elif (len(listOccupantCoords) == 1):  # one person in the room
        # foreach sample point ------------------------------------------------
        for i in range(arrayHumanAwareTLVs.shape[0]):
            spKey = "PT_" + str(i).zfill(4)
            spCoords = dictSPcoords[spKey]
            # determine occupant distance from sample point
            minSPdistance = math.dist(spCoords, listOccupantCoords[0])
            # determine target ytp for sample point
            if (minSPdistance < thresholdsR1R2[0]):  # mind < R1
                ytp = tlvOG[i] * dimmerValue
            elif (minSPdistance > thresholdsR1R2[1]):  # mind > R2
                ytp = tlvES[i] * dimmerValue
            # mind > R1 and mind < R2
            elif (minSPdistance >= thresholdsR1R2[0] and minSPdistance <= thresholdsR1R2[1]):
                # linear equation
                ytp = ((thresholdsR1R2[1] - minSPdistance) * (tlvOG[i] - tlvES[i]) / (
                    thresholdsR1R2[1] - thresholdsR1R2[0]) + tlvES[i]) * dimmerValue
                # cosinusoidal equation
                # ytp = (math.cos(math.pi - (thresholdsR1R2[1] - minSPdistance) * math.pi / (thresholdsR1R2[1] - thresholdsR1R2[0])) * (tlvOG[i] - tlvES[i]) / 2 + (tlvOG[i] + tlvES[i]) / 2) * dimmerValue
            arrayHumanAwareTLVs[i] = ytp
        print('HumanAwareness calculated for one person.')
        print(f'ytps {arrayHumanAwareTLVs[:]}')

    elif (len(listOccupantCoords) > 1):  # more than one people in the room
        # foreach sample point ------------------------------------------------
        for i in range(arrayHumanAwareTLVs.shape[0]):
            spKey = "PT_" + str(i).zfill(4)
            spCoords = dictSPcoords[spKey]
            # determine closest occupant's distance from sample point
            listSPOdistances = []
            for oCoords in listOccupantCoords:
                d = math.dist(spCoords, oCoords)
                listSPOdistances.append(d)
            listSPOdistances.sort()
            minSPdistance = listSPOdistances[0]
            # determine target ytp for sample point && color for Unity visualization of each sample point
            if (minSPdistance < thresholdsR1R2[0]):  # mind < R1
                ytp = tlvOG[i] * dimmerValue
            elif (minSPdistance > thresholdsR1R2[1]):  # mind > R2
                ytp = tlvES[i] * dimmerValue
            # mind > R1 and mind < R2
            elif (minSPdistance >= thresholdsR1R2[0] and minSPdistance <= thresholdsR1R2[1]):
                # linear equation
                ytp = ((thresholdsR1R2[1] - minSPdistance) * (tlvOG[i] - tlvES[i]) / (
                    thresholdsR1R2[1] - thresholdsR1R2[0]) + tlvES[i]) * dimmerValue
                # cosinusoidal equation
                # ytp = (math.cos(math.pi - (thresholdsR1R2[1] - minSPdistance) * math.pi / (thresholdsR1R2[1] - thresholdsR1R2[0])) * (tlvOG[i] - tlvES[i]) / 2 + (tlvOG[i] + tlvES[i]) / 2) * dimmerValue
            arrayHumanAwareTLVs[i] = ytp
        print('HumanAwareness calculated for multiple people.')
        print(f'ytps {arrayHumanAwareTLVs[:]}')

    return arrayHumanAwareTLVs

# INVERSE BUBBLE TLVs

def humanAware_TLVs_InverseBubble(dictSPcoords, tlvOG, listOccupantCoords, thresholdsR1R2, dimmerValue):

    arrayHumanAwareTLVs = np.empty([1215,])
    nearZeroValue = 0.005

    if (len(listOccupantCoords) == 0):  # no person in the room
        arrayHumanAwareTLVs = tlvES
        print('No people present. HumanAwareness assigned tlvES values everywhere.')

    elif (len(listOccupantCoords) == 1):  # one person in the room
        # foreach sample point ------------------------------------------------
        for i in range(arrayHumanAwareTLVs.shape[0]):
            spKey = "PT_" + str(i).zfill(4)
            spCoords = dictSPcoords[spKey]
            # determine occupant distance from sample point
            minSPdistance = math.dist(spCoords, listOccupantCoords[0])
            # determine target ytp for sample point
            if (minSPdistance < thresholdsR1R2[0]):  # mind < R1
                ytp = nearZeroValue * dimmerValue
            elif (minSPdistance > thresholdsR1R2[1]):  # mind > R2
                ytp = tlvOG[i] * dimmerValue
            # mind > R1 and mind < R2
            elif (minSPdistance >= thresholdsR1R2[0] and minSPdistance <= thresholdsR1R2[1]):
                # linear equation
                ytp = ((thresholdsR1R2[1] - minSPdistance) * (nearZeroValue - tlvOG[i]) / (thresholdsR1R2[1] - thresholdsR1R2[0]) + tlvOG[i]) * dimmerValue
                # cosinusoidal equation
                # ytp = (math.cos(math.pi - (thresholdsR1R2[1] - minSPdistance) * math.pi / (thresholdsR1R2[1] - thresholdsR1R2[0])) * (tlvOG[i] - tlvES[i]) / 2 + (tlvOG[i] + tlvES[i]) / 2) * dimmerValue
            arrayHumanAwareTLVs[i] = ytp
        print('HumanAwareness calculated for one person.')
        print(f'ytps {arrayHumanAwareTLVs[:]}')

    elif (len(listOccupantCoords) > 1):  # more than one people in the room
        # foreach sample point ------------------------------------------------
        for i in range(arrayHumanAwareTLVs.shape[0]):
            spKey = "PT_" + str(i).zfill(4)
            spCoords = dictSPcoords[spKey]
            # determine closest occupant's distance from sample point
            listSPOdistances = []
            for oCoords in listOccupantCoords:
                d = math.dist(spCoords, oCoords)
                listSPOdistances.append(d)
            listSPOdistances.sort()
            minSPdistance = listSPOdistances[0]
            # determine target ytp for sample point && color for Unity visualization of each sample point
            if (minSPdistance < thresholdsR1R2[0]):  # mind < R1
                ytp = nearZeroValue * dimmerValue
            elif (minSPdistance > thresholdsR1R2[1]):  # mind > R2
                ytp = tlvOG[i] * dimmerValue
            # mind > R1 and mind < R2
            elif (minSPdistance >= thresholdsR1R2[0] and minSPdistance <= thresholdsR1R2[1]):
                # linear equation
                ytp = ((thresholdsR1R2[1] - minSPdistance) * (nearZeroValue - tlvOG[i]) / (thresholdsR1R2[1] - thresholdsR1R2[0]) + tlvOG[i]) * dimmerValue
                # cosinusoidal equation
                # ytp = (math.cos(math.pi - (thresholdsR1R2[1] - minSPdistance) * math.pi / (thresholdsR1R2[1] - thresholdsR1R2[0])) * (tlvOG[i] - tlvES[i]) / 2 + (tlvOG[i] + tlvES[i]) / 2) * dimmerValue
            arrayHumanAwareTLVs[i] = ytp
        print('HumanAwareness calculated for multiple people.')
        print(f'ytps {arrayHumanAwareTLVs[:]}')

    return arrayHumanAwareTLVs


# PREP BVLS SOLVER ---------------------------------------------------------

# READS CONTRIBUTION MATRIX FROM CSV
# INPUT: csv file path
# OUTPUT: arrMatrix: contribution matrix as numpy array


def read_store_contribution_matrix_asArray(contrMatrixFilePath):

    arrMatrix = np.genfromtxt(
        contrMatrixFilePath, dtype=float, delimiter=",", skip_header=1)[:, 1:]
    print(f'Matrix Shape is: {arrMatrix.shape}')

    return arrMatrix

# READS TARGET LUX VALUES FOR EACH GENERAL ACTIVITY FROM FOLDER WITH CSVs
# INPUT: folder file path
# OUTPUT: dictActVectors: dictionary containing a three column numpy array with target lux values for each activity - eco|eco++|eco+


def read_store_activities_asDictTLVs_3modes(activitiesFolderFilePath):

    # dictActVectors<ActID,3LuxVectorsArray>
    dictActVectors = {}
    activityFiles = os.listdir(activitiesFolderFilePath)
    for i, file in enumerate(activityFiles):
        actFilePath = activitiesFolderFilePath + file
        dictActVectors[file[:len(file)-4]] = np.genfromtxt(actFilePath,
                                                           dtype=float, delimiter=",", skip_header=1)[:, 1:]
    # print(dictActVectors.keys())

    return dictActVectors


def multis_transform_fromOutput_toDictionary(outputtedVector):

    dictMT1 = {}
    dictMT2 = {}
    dictMT3 = {}
    dictMT4 = {}
    dictMT5 = {}
    dictMT6 = {}
    dictMT7 = {}
    dictMT8 = {}

    names = ["b", "l", "r", "u"]

    # group 424 Multis per Troffer
    for id in range(1, 425):
        q, r = divmod(id, 53)
        if (r == 0):
            q -= 1
            r = 53
        # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')

        if (q+1 == 1):
            # TR1
            if (r < 5):
                keyMultiBat = "MT1_" + names[r-1]
                # bat multis for troffer
                dictMT1[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT1_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT1[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 2):
            # TR2
            if (r < 5):
                keyMultiBat = "MT2_" + names[r-1]
                # bat multis for troffer
                dictMT2[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT2_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT2[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 3):
            # TR3
            if (r < 5):
                keyMultiBat = "MT3_" + names[r-1]
                # bat multis for troffer
                dictMT3[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT3_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT3[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 4):
            # TR4
            if (r < 5):
                keyMultiBat = "MT4_" + names[r-1]
                # bat multis for troffer
                dictMT4[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT4_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT4[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 5):
            # TR5
            if (r < 5):
                keyMultiBat = "MT5_" + names[r-1]
                # bat multis for troffer
                dictMT5[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT5_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT5[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 6):
            # TR6
            if (r < 5):
                keyMultiBat = "MT6_" + names[r-1]
                # bat multis for troffer
                dictMT6[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT6_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT6[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 7):
            # TR7
            if (r < 5):
                keyMultiBat = "MT7_" + names[r-1]
                # bat multis for troffer
                dictMT7[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT7_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT7[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

        if (q+1 == 8):
            # TR8
            if (r < 5):
                keyMultiBat = "MT8_" + names[r-1]
                # bat multis for troffer
                dictMT8[keyMultiBat] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} and is its {r} element')
            if (r > 4):
                newID = r - 4
                quo, rem = divmod(newID, 7)
                if (rem == 0):
                    quo -= 1
                    rem = 7
                x = quo + 1  # row
                y = rem  # column
                keyMultiPixel = "MT8_" + str(x) + str(y)
                # pixel multis for troffer
                dictMT8[keyMultiPixel] = outputtedVector[id-1]
                # print(f'Light {id} belongs to troffer {q+1} in row {x} and column {y}')

    return dictMT1, dictMT2, dictMT3, dictMT4, dictMT5, dictMT6, dictMT7, dictMT8

# TRANSFORMS MULTIPLIERS FROM BVLS OUTPUT TO DICTIONARY
# INPUTS:
# outputtedVector: vector outputted from bvls
# listTrofferOrigins: list of troffer origin coordinates
# OUTPUT: master dictionary containing a dictionary with 53 multipliers per troffer

# SCULPT --------------------------------------------------------------------

# BVLS SOLVER
# INPUTS:
# contrMatrix: contribution matrix
# dictSPcoords: dictionary with sample point coordinates
# swapWith: key that is used to pass the general activity - passsed from kinect - can either be:
# (swapWith == "setForget BVLS") or (swapWith == "table BVLS") or (swapWith == "west BVLS") or (swapWith == "blackboard BVLS")
# dictActTLV: dictionary containing a three column numpy array with target lux values for each activity - eco|eco++|eco+
# ecoMode: active eco mode (possible options: eco|eco+|eco++)
# listOccupantCoords: list of all occupant coordinates
# thresholdsR1R2: bubble math threshold values
# thresholdsR1R2 = [0.5,2] # default values for (R1,R2) as assigned at START section of code
# dimmerValue: defaults to a value of 1 at START section of code
# listDistAngleMultis: glare restricted upper bounds
# OUTPUTS: eight dictionaries with SCULPTED multipliers
# dictMxBVLS[0], dictMxBVLS[1], dictMxBVLS[2], dictMxBVLS[3], dictMxBVLS[4], dictMxBVLS[5], dictMxBVLS[6], dictMxBVLS[7]


def solver_BVLS(contrMatrix, dictSPcoords, swapWith, dictActTLV, ecoMode, listOccupantCoords, thresholdsR1R2, dimmerValue, listDistAngleMultis, listTrofferOrigins, inverseMode):

    # BVLS optimization
    lb = 0.0
    # ub = np.ones(contrMatrix.shape[1],)
    ub = np.array(listDistAngleMultis)
    # for Light Scenes where TLVs are read from disk

    actKey = "Scene " + swapWith
    if (ecoMode == "eco"):
        targetVector = dictActTLV[actKey][:, 0]
        if (inverseMode == "True"):
            targetVector = humanAware_TLVs_InverseBubble(dictSPcoords, dictActTLV[actKey][:, 0], listOccupantCoords, thresholdsR1R2, dimmerValue)
    elif (ecoMode == "ecoPlus"):
        targetVector = humanAware_TLVs(dictSPcoords, dictActTLV[actKey][:, 0], dictActTLV[actKey][:, 2], listOccupantCoords, thresholdsR1R2, dimmerValue)
    elif (ecoMode == "ecoPlusPlus"):
        targetVector = humanAware_TLVs(dictSPcoords, dictActTLV[actKey][:, 0], dictActTLV[actKey][:, 1], listOccupantCoords, thresholdsR1R2, dimmerValue)
            

    # --------------- solver ---------------
    # print(f'Lower Bounds are: {lb}')
    # print(f'Upper Bounds are: {ub}')
    beforeBVLS = time.time()
    res = lsq_linear(contrMatrix, targetVector, bounds=(lb, ub), method='trf', tol=1e-10, max_iter=400, verbose=0)
    scalars = res.x

    afterBVLS = time.time()

    print(f'Optimization took {afterBVLS - beforeBVLS} seconds')

    # plot rounded scalars
    # roundedScalars = []
    # for s in scalars:
    #     roundedS = round(s, 3)
    #     roundedScalars.append(roundedS)
    # print(f'The multis are (from BVLS): {roundedScalars}')
    # print(f'The multis shape is (from BVLS): {scalars.shape}')

    # -------- solver additional ifo --------
    # constraints = res.active_mask
    # notOnBoundsOld = constraints[np.where(constraints == 0)].size
    # notOnBounds = constraints.size - \
    #     np.count_nonzero(constraints)  # the zeros are NotOnBounds
    # print(
    #     f'There are {notOnBounds} Multis that are NOT on Bounds. Old ones {notOnBoundsOld}')

    # dictionaries with BVLS multiplieres per troffer
    # dictM1, dictM2, dictM3, dictM4, dictM5, dictM6, dictM7, dictM8
    dictMxBVLS = multis_transform_fromOutput_toDictionary(scalars)
    # print(dictMxBVLS[0].values())

    return dictMxBVLS[0], dictMxBVLS[1], dictMxBVLS[2], dictMxBVLS[3], dictMxBVLS[4], dictMxBVLS[5], dictMxBVLS[6], dictMxBVLS[7]

# EXTRAS -------------------------------------------------------

# CREATES DICTIONARIES OF UNIFORM MULTIPLIERS
# INPUT: fillValue: value assigned to all multipliers
# OUTPUT: a dictionary per troffer


def define_multiplier_dictionaries(fillValue):

    # MULTIS for EACH troffer
    names = ["u", "r", "b", "l"]

    dictMT1 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT1_" + str(x) + str(y)
            dictMT1[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT1_" + n
        dictMT1[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT2 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT2_" + str(x) + str(y)
            dictMT2[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT2_" + n
        dictMT2[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT3 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT3_" + str(x) + str(y)
            dictMT3[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT3_" + n
        dictMT3[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT4 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT4_" + str(x) + str(y)
            dictMT4[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT4_" + n
        dictMT4[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT5 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT5_" + str(x) + str(y)
            dictMT5[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT5_" + n
        dictMT5[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT6 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT6_" + str(x) + str(y)
            dictMT6[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT6_" + n
        dictMT6[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT7 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT7_" + str(x) + str(y)
            dictMT7[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT7_" + n
        dictMT7[keyMultiBat] = fillValue  # bat multis for troffer

    dictMT8 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT8_" + str(x) + str(y)
            dictMT8[keyMultiPixel] = fillValue  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT8_" + n
        dictMT8[keyMultiBat] = fillValue  # bat multis for troffer

    return dictMT1, dictMT2, dictMT3, dictMT4, dictMT5, dictMT6, dictMT7, dictMT8

# CREATES DICTIONARIES OF RANDOM MULTIPLIERS
# INPUT: no input
# OUTPUT: a dictionary per troffer


def define_RANDOM_multiplier_dictionaries():

    # MULTIS for EACH troffer
    names = ["u", "r", "b", "l"]

    dictMT1 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT1_" + str(x) + str(y)
            dictMT1[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT1_" + n
        dictMT1[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT2 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT2_" + str(x) + str(y)
            dictMT2[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT2_" + n
        dictMT2[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT3 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT3_" + str(x) + str(y)
            dictMT3[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT3_" + n
        dictMT3[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT4 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT4_" + str(x) + str(y)
            dictMT4[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT4_" + n
        dictMT4[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT5 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT5_" + str(x) + str(y)
            dictMT5[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT5_" + n
        dictMT5[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT6 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT6_" + str(x) + str(y)
            dictMT6[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT6_" + n
        dictMT6[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT7 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT7_" + str(x) + str(y)
            dictMT7[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT7_" + n
        dictMT7[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    dictMT8 = {}
    for x in range(1, 8):
        for y in range(1, 8):
            keyMultiPixel = "MT8_" + str(x) + str(y)
            dictMT8[keyMultiPixel] = random.uniform(
                0.0, 1.0)  # pixel multis for troffer
    for n in names:
        keyMultiBat = "MT8_" + n
        dictMT8[keyMultiBat] = random.uniform(
            0.0, 1.0)  # bat multis for troffer

    return dictMT1, dictMT2, dictMT3, dictMT4, dictMT5, dictMT6, dictMT7, dictMT8

# PER TROFFER GROUP -------------------------------------------------------

# SUMS MULTIPLIERS FOR GIVEVN ACTIVITY
# INPUT: dictionary of multipliers
# OUTPUTS:
# sum8Troffers: sum of multipliers of all 8 troffers
# multisCounter: counter that checks all multipliers are accounted for


def calculate_multisSum_basedonActivity(dictActMultis):

    sum8Troffers = 0
    multisCounter = 0
    for i in range(1, 9):
        # print(f'Troffer {i} multis are:')
        # print(dictActMultis[i-1].values())
        sumTroffer = sum(dictActMultis[i-1].values())
        sum8Troffers += sumTroffer
        multisCounter += len(dictActMultis[i-1].values())
    # print(f'Sum of troffers {sum8Troffers}')
    # print(f'Count of troffers {multisCounter}')

    return sum8Troffers, multisCounter

def calculate_multisSum_basedonActivity_NEW_selectTroffers(dictActMultis, trofferIDs):

    # FOR EXAMPLE: trofferIDs = [7,8]

    sumXTroffers = 0
    multisCounter = 0
    for tr in trofferIDs:
        # print(f'Troffer {tr} multis are:')
        # print(dictActMultis[tr-1].values())
        sumTroffer = sum(dictActMultis[tr-1].values())
        sumXTroffers += sumTroffer
        multisCounter += len(dictActMultis[tr-1].values())
    # print(f'Sum of troffers {sumXTroffers}')
    # print(f'Count of troffers {multisCounter}')

    return sumXTroffers, multisCounter

# DEPLOYS SOLVER AND CALCULATES ENERGY SAVINGS
# INPUTS:
# swapWith: key that is used to pass the general activity - passsed from kinect - can either be:
# (swapWith == "setForget BVLS") or (swapWith == "table BVLS") or (swapWith == "west BVLS") or (swapWith == "blackboard BVLS")
# mtx: contribution matrix
# dictSPcoords: dictionary with sample point coordinates
# dictActTLV: dictionary containing a three column numpy array with target lux values for each activity - eco|eco++|eco+
# ecoMode: active eco mode (possible options: eco|eco+|eco++)
# listOccupantCoords: list of all occupant coordinates
# thresholdsR1R2: bubble math threshold values
# thresholdsR1R2 = [0.5,2] # default values for (R1,R2) as assigned at START section of code
# listDistAngleMultis: glare restricted upper bounds
# OUTPUTS:
# dictAllTrMultis: dictionary with sculpted multipliers for all troffers
# energySavings: energy savings of sculpted lighting configuration


def solver_BVLS_EnergySavings(swapWith, mtx, dictSPcoords, dictActLV, ecoMode, listOccupantCoords, thresholdsR1R2, listDistAngleMultis):

    start = time.time()

    if ((swapWith == "random") or (swapWith == "full") or (swapWith == "86%") or (swapWith == "dim") or (swapWith == "zero") or
            (swapWith == "setForget BVLS") or (swapWith == "table BVLS") or (swapWith == "west BVLS") or (swapWith == "blackboard BVLS")):

        if (swapWith == "random"):
            dictMultis = define_RANDOM_multiplier_dictionaries()
        elif (swapWith == "full"):
            dictMultis = define_multiplier_dictionaries(1)
        elif (swapWith == "86%"):
            dictMultis = define_multiplier_dictionaries(0.86)
        elif (swapWith == "dim"):
            dictMultis = define_multiplier_dictionaries(0.5)
        elif (swapWith == "zero"):
            dictMultis = define_multiplier_dictionaries(0.05)
        elif ((swapWith == "setForget BVLS") or (swapWith == "table BVLS") or (swapWith == "west BVLS") or (swapWith == "blackboard BVLS")):
            dictMultis = solver_BVLS(mtx, dictSPcoords, swapWith, dictActLV, ecoMode, listOccupantCoords, thresholdsR1R2, dimmerValue, listDistAngleMultis, listTrofferOrigins, inverseMode)

        # calculate ES from MULTIS
        activityMultisSum, multisCounter = calculate_multisSum_basedonActivity(dictMultis)
        energySavings = 100 - (activityMultisSum/benchmark1MultisSum*100)
        print(f'Activity {swapWith} multisSum is {"{:.2f}".format(activityMultisSum)}. The max multisSum could be Bench1 {benchmark1MultisSum}. The energySavings are: {"{:.2f}".format(energySavings)}.')
        print("------------------------")
        print("------------------------")
        # calculate ES from MULTIS | NEW | FOR SELECT TROFFERS
        activityMultisSumNEWselectTr, multisCounterNEWselectTr = calculate_multisSum_basedonActivity_NEW_selectTroffers(dictMultis, trofferIDs)
        energySavingsNEWselectTr = 100 - (activityMultisSumNEWselectTr/benchmark1MultisSumNEWselectTr*100)
        print(f'Activity {swapWith} multisSumNEW is {"{:.2f}".format(activityMultisSumNEWselectTr)}. The max multisSumNEW could be Bench1NEW {benchmark1MultisSumNEWselectTr}. The energySavingsNEW are: {"{:.2f}".format(energySavingsNEWselectTr)}.')
        print("------------------------")
        print("------------------------")
        multisSculpted = dictMultis

    end = time.time()
    duration = end-start
    return multisSculpted, energySavings, energySavingsNEWselectTr


# START -----------------------------------------------------------------

# intensityBoosterFactor
intensityBoosterFactor = 1

# for energy efficient TLVs
listOccupantCoords = [(-2.75116, 1.817935, -0.8036823),
                      (-4.405, 1.817935, -6.698), (-1.74, 1.817935, -9.625)]
thresholdsR1R2 = [0.5, 2]  # (R1,R2)

# LIST of Troffer Origins - COORDINATES ARE FOR LESA
listTrofferOrigins = [(-2.727, 4.040, -2.203), (-2.727, 4.040, -4.280), (-2.727, 4.040, -6.353), (-2.727, 4.040, -9.436), (-2.727, 4.040, -11.515),
                      (-2.727, 4.040, -13.595), (-6.150, 4.040, -2.203), (-6.150, 4.040, -4.280)]
trofferNumber = len(listTrofferOrigins)
multisNumber = len(listTrofferOrigins) * 53

# FILE PATHS
iesOGFilePath = 'Sculpting/AuxiliaryFiles/Original_3deg_IES_forPython/'
precalculatedIESReadFilePath = 'Sculpting/AuxiliaryFiles/bvls_Sculpted_Activities_perTroffer/'
assignedTroffersIESPth = 'Sculpting/AuxiliaryFiles/newIES/python replace AB test/'
iesReadFilePath = 'Sculpting/AuxiliaryFiles/newIES/python read/'
exportColorInfoLVsPath = 'Sculpting/AuxiliaryFiles/LVs color info/ColorInfoLVs.txt'
exportColorInfoSPsPath = 'Sculpting/AuxiliaryFiles/SPs color info/ColorInfoSPs.txt'
combUNSCfilePath = 'Sculpting/AuxiliaryFiles/combUNSCiesFile.ies'
contrMatrixFilePath = 'Sculpting/AuxiliaryFiles/Matrix.csv'
activitiesFolderFilePath = 'Sculpting/AuxiliaryFiles/Activities Folder cossie/'
samplePointsFilePath = 'Sculpting/AuxiliaryFiles/SensorPoints.csv'
readLightVectorsPath = 'Sculpting/AuxiliaryFiles/IES_Directions_Update-20220323.csv'
exportLightVectorsPath = 'Sculpting/AuxiliaryFiles/LightVectors53.txt'
exportAllTrofferMutisPath = 'Sculpting/AuxiliaryFiles/AllTrofferMultis.txt'
# exportColorInfoLVsPath = 'C:/Users/Admin/Desktop/SCULPT Project/SCULPT semester 6/7 Viz Gaze Math Unity/ColorInfoLVs.txt'

# ------------------------------------------------------------------------

# DICTIONARIES WITH MULTIPLIERS PER TROFFER
dictMTx = define_RANDOM_multiplier_dictionaries()
# print(dictMTx[1], dictMTx[3], dictMTx[5], dictMTx[7])

# ------------------------------------------------------------------------

# CONTRIBUTION MATRIX
mtx = read_store_contribution_matrix_asArray(contrMatrixFilePath)

# load ACTIVITIES AND LUX VALUES per SAMPLE POINT
dictActLV = read_store_activities_asDictTLVs_3modes(activitiesFolderFilePath)

# DEFINE dictMxBVLS FOR BVLS MULTIS
dictMxBVLS = {}

# ------------------------------------------------------------------------

# INVERSE BUBBLE FLAG
inverseMode = "False"

# ------------------------------------------------------------------------

# ECO PRO (energy efficiency)

# load dictionaries with spIDs, spCoords, and SPfeatureIDs per SAMPLE POINT
dictSPcoords, dictSPfeatureID = read_store_sample_points_asDict(
    samplePointsFilePath)

# read SetForget TLVs (OG and ES)
tlvOG = dictActLV["Scene setForget BVLS"][:, 0]
tlvES = dictActLV["Scene setForget BVLS"][:, 1]
dimmerValue = 1

# create SetForget TLVs (HumanAware)
arrayHumanAwareTLVs = humanAware_TLVs(
    dictSPcoords, tlvOG, tlvES, listOccupantCoords, thresholdsR1R2, dimmerValue)
# this is probably NOT required here - TO BE DELETED
# export_info_toColor_SamplePoints(dictSPcoords, arrayHumanAwareTLVs, listColorVector, exportColorInfoSPsPath)

# EE CALCS & Benchmark 1
ecoMode = "eco"
dictMultis1 = solver_BVLS(mtx, dictSPcoords, "setForget BVLS", dictActLV, ecoMode, listOccupantCoords, thresholdsR1R2, dimmerValue, 1.0, exportColorInfoSPsPath, inverseMode)
benchmark1MultisSum, multisCounter = calculate_multisSum_basedonActivity(dictMultis1)
print(f'Bench1 sumMultis is {benchmark1MultisSum}')

# EE CALCS & Benchmark 1 | NEW | FOR SELECT TROFFERS
trofferIDs = [7,8]
benchmark1MultisSumNEWselectTr, multisCounterNEWselectTr = calculate_multisSum_basedonActivity_NEW_selectTroffers(dictMultis1, trofferIDs)
print(f'Bench1 NEW sumMultis is {benchmark1MultisSumNEWselectTr}')

print("------------------------")
print("------------------------")

# ------------------------------------------------------------------------

# GAZE MATH

# List with Manual Troffer Origins
listManualTrofferOrigins = [(-2.224, 3.084, -1.24), (-3.451, 3.084, -1.24), (-1.608, 3.084, -3.686), (-4.067, 3.084, -3.686),
                            (-1.608, 3.084, -6.766), (-4.067, 3.084, -6.766), (-2.224, 3.084, -9.23), (-3.451, 3.084, -9.23)]
# Read dict of 53 Light Vectors from HKS & export file for UNITY
dictLVs53 = read_store_inDict_LightVectors_atOrigin(readLightVectorsPath)
export_Light_Vectors(dictLVs53, exportLightVectorsPath)
# dictDistrLVs = distribute_LightVectors_atTrofferOrigins(dictLVs53, listManualTrofferOrigins)
# Light Vector Intensity Multis based on Angle
#       ++ computed later ON
# Define Angle Threshold, Max, Min Values
th1, th2, gthMax, gthMin = 120, 165, 1, 0
# Define Distance Threshold, Max Values
x1, x2, gmax = 1, 2, 1

print("------------------------")
print("------------------------")


glareMode = "False"
listGazeVectors = []
# -------------------------------------------------------------------------------------------------------------
# CHECK FOR GLARE ELIMINATION REQUIREMENTS
# -------------------------------------------------------------------------------------------------------------
if (glareMode == "False"):
    listDistAngleMultis = np.ones(424).tolist()
elif (glareMode == "True"):
    # CALCULATE distanceAngleMultis for each Light Vector !!
    listAngleMultis, listDistAngleMultis, dictDistAngleMultis = angleAndDistance_gaze_math(
        listGazeVectors, dictLVs53, th1, th2, gthMax, gthMin, listOccupantCoords, listTrofferOrigins, x1, x2, gmax)
    # print(f'Distance and Angle Multis Dict {dictDistAngleMultis}')
    print(f'Angle Multis List {listAngleMultis}')
    print(f'Distance and Angle Multis List {listDistAngleMultis}')
    # EXPORT info to visualize COLORED LVs in UNITY
# -------------------------------------------------------------------------------------------------------------


# for i in range(1,9):
#     with open('MultisFor ' + swapWith + ' Troffer' + str(i) + '.csv', 'w') as f:  # You will need 'wb' mode in Python 2.x
#         w = csv.DictWriter(f, multisSculpted[i-1].keys())
#         w.writeheader()
#         w.writerow(multisSculpted[i-1])


# INITIATE while LOOP
counter = 0
print(f"Counter value is: {counter}")
canSendToUnityBool = False
# sock.sendall("waiting".encode("UTF-8")) # Converting String to Byte and Sending it to C#
messageUnity = "null"

# CREATE AN update LOOP
while True:
    # time.sleep(5) # take 5 more
    # Receiving Byte Data from C# and Converting it to String
    msgReceived = sock.recv(1024).decode("UTF-8")
    print("------------------------")
    print("------------------------")
    print(f"Received Msg from UNITY is:\n{msgReceived}")

    # CHECK MESSAGE
    if (msgReceived != "null"):

        startReceivingTime = time.time()

        # DECOMPOSE Message Received from UNITY
        listMessages = msgReceived.split("$")

        if (listMessages[0][0:4] == "swap"):
            act, mode, r1r2, oPos, gVecs, dimVal, glMode, invMode = 0, 1, 2, 3, 4, 5, 6, 7
            activity = listMessages[act]
            print(f'Acivity is: {activity}')
            if (len(listMessages) > 1):
                ecoMode = listMessages[mode]
                print(f'EcoMode is: {ecoMode}')
            if (len(listMessages) > 2):
                strthr1r2 = listMessages[r1r2].split(" ")
                thresholdsR1R2 = [float(strthr1r2[0]), float(strthr1r2[1])]
                print(
                    f'Threshold1 is {thresholdsR1R2[0]} and Threshold2 is {thresholdsR1R2[1]}')
            if (len(listMessages) > 3):
                print(f'OG String is {listMessages[oPos]}.')
                listStrOccLoc = listMessages[oPos].replace(
                    "(", "").replace(" ", "").replace(")", " ").rstrip().split(" ")
                listOccupantCoords = []
                listArrayOccupantCoords = []
                for st in listStrOccLoc:
                    strValues = st.split(",")
                    floatValues = [float(x) for x in strValues]
                    tupleWithVals = tuple(floatValues)
                    arrayWithVals = np.array(floatValues)
                    listOccupantCoords.append(tupleWithVals)
                    listArrayOccupantCoords.append(arrayWithVals)
                print(f'Occupand Coords are {listOccupantCoords}.')
            if (len(listMessages) > 4):
                print(f'OG Gaze Vectors String is {listMessages[gVecs]}.')
                listStrGazeVectors = listMessages[gVecs].replace(
                    " ", "").replace(")", ") ").rstrip().split(" ")
                listGazeVectors = []
                for st in listStrGazeVectors:
                    strValues = st.replace("(", "").replace(")", "").split(",")
                    floatValues = [float(x) for x in strValues]
                    tupleWithVals = tuple(floatValues)
                    arrayWithVals = np.array(floatValues)
                    # list with np arrays
                    listGazeVectors.append(arrayWithVals)
                print(f'Occupand Gaze Vectors are {listGazeVectors}.')
            if (len(listMessages) > 5):
                dimmerValue = float(listMessages[dimVal])
                print(f'The Value of the Dimmer Slider is {dimmerValue}.')
            if (len(listMessages) > 6):
                glareMode = listMessages[glMode]
                print(f'The Glare Mode is {glareMode}.')
                # -------------------------------------------------------------------------------------------------------------
                # -------------------------------------------------------------------------------------------------------------
                # CHECK FOR GLARE ELIMINATION REQUIREMENTS
                # -------------------------------------------------------------------------------------------------------------
                # -------------------------------------------------------------------------------------------------------------
                if (glareMode == "False"):
                    listDistAngleMultis = np.ones(424).tolist()
                elif (glareMode == "True"):
                    # CALCULATE distanceAngleMultis for each Light Vector !!
                    listAngleMultis, listDistAngleMultis, dictDistAngleMultis = angleAndDistance_gaze_math(
                        listGazeVectors, dictLVs53, th1, th2, gthMax, gthMin, listArrayOccupantCoords, listTrofferOrigins, x1, x2, gmax)
                    # print(f'Distance and Angle Multis Dict {dictDistAngleMultis}')
                    print(f'Angle Multis List {listAngleMultis}')
                    print(
                        f'Distance and Angle Multis List {listDistAngleMultis}')
                    # EXPORT info to visualize COLORED LVs in UNITY
                # -------------------------------------------------------------------------------------------------------------
                # -------------------------------------------------------------------------------------------------------------
            if (len(listMessages) > 7):
                inverseMode = listMessages[invMode]
                print(f'The Inverse Bubble Mode is {inverseMode}.')
        
        
        if (
           (activity == "swap ies profiles to SetForget BVLS") or
           (activity == "swap ies profiles to Table BVLS") or
           (activity == "swap ies profiles to West BVLS") or
           (activity == "swap ies profiles to Blackboard BVLS")):

            if (activity == "swap ies profiles to SetForget BVLS"):
                swapWith = "setForget BVLS"
            elif (activity == "swap ies profiles to Table BVLS"):
                swapWith = "table BVLS"
            elif (activity == "swap ies profiles to West BVLS"):
                swapWith = "west BVLS"
            elif (activity == "swap ies profiles to Blackboard BVLS"):
                swapWith = "blackboard BVLS"

            activity = "null"

            startSwap = time.time()

            # CALL SWAPPING FUNCTION: for all troffers of group
            multisSculpted, energySavings, energySavingsNEWselectTr = solver_BVLS_EnergySavings(
                swapWith, mtx, dictSPcoords, dictActLV, ecoMode, listOccupantCoords, thresholdsR1R2, listDistAngleMultis)

            endSwap = time.time()

            print(f"Time to Swap is: {endSwap - startSwap}")

            # print(multisSculpted[7].keys()) 

            counter += 1
            print(f"Counter value is: {counter}")
            messageUnity = "BLEND" + "$" + swapWith + \
                "$" + str("{:.2f}".format(energySavings))
            canSendToUnityBool = False

            # print("------------------------")
            # print(multisSculpted[7])
            # print("------------------------")
            # arr = np.array(list(multisSculpted[7].values()))
            # print(arr)
            # print("------------------------")
            multis7 = np.array(list(multisSculpted[6].values()))
            multis8 = np.array(list(multisSculpted[7].values()))
            sendMultisToArduino(multis7, multis8)

            # # SEND MESSAGE TO UNITY
            # if (canSendToUnityBool):
            #     sock.sendall(messageUnity.encode("UTF-8")) # Converting String to Byte and Sending it to C#
            #     canSendToUnityBool = False
            #     messageUnity = "null"
        endReceivingTime = time.time()
        sock.sendall('send next'.encode())
        endReceivingEncodedTime = time.time()

        print(f"Time before sending is: {endReceivingTime - startReceivingTime}")
        print(f"Time after sending is: {endReceivingEncodedTime - startReceivingTime}")
