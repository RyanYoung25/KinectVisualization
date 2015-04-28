#! /usr/bin/env python

import numpy as np
import scipy
import math 
import json
import sys

'''
Calculate the theta between two vectors a, and b. 
'''
def calculateTheta(a, b):
    dotProd = np.dot(a, b)        #a dot b
    aNorm = np.sqrt(np.dot(a, a))  # ||a||
    bNorm = np.sqrt(np.dot(b, b))  # ||b||
    #theta = arccos(a dot b / (||a||*||b||))
    val =  dotProd/(aNorm*bNorm)
    #I think there is a float issue
    if val > 1:
        val = 1
    elif val < -1:
        val = -1
    return math.acos(val)


'''
Takes a list of xyz positions for two points, A and
B and generates the vector from A to B
'''
def generateVector(Apos, Bpos):
    #B minus A
    xComp = Bpos[0] - Apos[0]
    yComp = Bpos[1] - Apos[1]
    zComp = Bpos[2] - Apos[2]
    return np.array([xComp, yComp, zComp])


'''
Return a numpy vector of the coordinates for
the joint struture passed to to
'''
def getPointFromJoint(joint):
    posList = []
    posList.append(joint['pos']['x'])
    posList.append(joint['pos']['y'])
    posList.append(joint['pos']['z'])
    return np.array(posList)


'''
Math for calculating the Shoulder Yaw from
Matthew Wiese Star Project ITERO inspired
'''
def calculateYaw(EW, ES, joint):
        #Create the z component vector of the elbow
        zElbVec = np.array([2, 2, joint[2]])
        #Create the red, elbow wrist elbow shoulder cross product vector
        elbowOrient = np.cross(EW, ES)
        #Create the Blue vector by crossing the elbowShoulder and zVector
        blue = np.cross(ES, zElbVec)
        #Find the green vector by crossing the blue vector and the elbow to shoulder
        green = np.cross(ES, blue)
        #Find the angle between the green vector and the orientation
        yaw = calculateTheta(elbowOrient, green)

        return yaw


def reduceRadians(angle):
    #For negative numbers use negative 2pi
    scaler =  math.pi
    if(angle < 0.0):
        scaler = -1 * scaler

    #Mod division to get the remaining amount of rotation. Should normalize
    #things a little bit. Hopefully
    newAngle = (angle % scaler)
    return newAngle


def getPitch(vector):
    #acos(sqrt(1/(1+(vecta[0]/vecta[2]))**2))
    zComps = vector[0]/vector[2]
    zComps = zComps**2
    zComps = 1 + zComps
    beta = math.sqrt(1/zComps)
    beta = math.acos(beta)
    return beta


def getRoll(vector):
    #asin(vect[1])
    alpha = math.asin(vector[1])
    return alpha


def normalizeVector(vec):
    vec = vec / np.linalg.norm(vec)
    return vec


def calculateRPY(bodyMatrix, armMatrix):

    #Symbolic computation done in matlab
    '''

    [ cos(b)*cos(y) + sin(a)*sin(b)*sin(y), cos(a)*sin(y), cos(b)*sin(a)*sin(y) - sin(b)*cos(y)]
    [ sin(a)*sin(b)*cos(y) - cos(b)*sin(y), cos(a)*cos(y), sin(b)*sin(y) + cos(b)*sin(a)*cos(y)]
    [                        cos(a)*sin(b),       -sin(a),                        cos(a)*cos(b)]

    '''

    #Create the rotation matrix
    rot = np.transpose(bodyMatrix)*armMatrix
    #Find alpha, roll
    negSinAlpha = rot[2,1]
    alpha = math.asin(-1 * negSinAlpha)
    #Find beta, pitch
    #Cos(a)*cos(b) = rot[2,2]
    beta = math.acos(rot[2,2]/math.cos(alpha))
    #Find gamma, yaw
    #cos(a)*cos(gamma) = rot[1,1]
    gamma = math.acos(rot[1,1]/math.cos(alpha))


    #Return roll, pitch, yaw
    guess = np.array([alpha, beta, gamma])

    x = solveNonLinear(rot, guess)
    return x


def solveNonLinear(rot, guess):
    #The function of alpah beta and gamma
    def F(vals):
        a = vals[0]
        b = vals[1]
        y = vals[2]
        #The rotation matrix formulas
        r = [math.cos(b)*math.cos(y) + math.sin(a)*math.sin(b)*math.sin(y),
             math.cos(a)*math.sin(y),
             math.cos(b)*math.sin(a)*math.sin(y) - math.sin(b)*math.cos(y),
             math.sin(a)*math.sin(b)*math.cos(y) - math.cos(b)*math.sin(y),
             math.cos(a)*math.cos(y),
             math.sin(b)*math.sin(y) + math.cos(b)*math.sin(a)*math.cos(y),
             math.cos(a)*math.sin(b),
             -1*math.sin(a),
             math.cos(a)*math.cos(b)]
             
        #The difference between the arrays, a 1x9 array
        diff = np.array(r) - np.ravel(rot)

        return diff

    sol = scipy.optimize.leastsq(F, guess)
    return sol[0]


def FancygenerateAngles(jsonString):
    dict = json.loads(jsonString)
    #Get the joint list
    jointList = dict["Joints"]

    #Create coordianate vectors for each joint point
    #Indices from above
    LH = getPointFromJoint(jointList[12])
    LW = getPointFromJoint(jointList[11])
    LE = getPointFromJoint(jointList[10])
    LS = getPointFromJoint(jointList[9])
    NK = getPointFromJoint(jointList[1])
    HD = getPointFromJoint(jointList[0])
    RH = getPointFromJoint(jointList[6])
    RS = getPointFromJoint(jointList[3])
    RE = getPointFromJoint(jointList[4])
    RW = getPointFromJoint(jointList[5])
    TS = getPointFromJoint(jointList[2])

    #Create vectors for the elbows
    REBA = generateVector(RE, RW)
    REBB = generateVector(RE, RS)
    LEBA = generateVector(LE, LW)
    LEBB = generateVector(LE, LS)

    #Create vectors for the roll
    
    RSRA = generateVector(RS, RH)
    RSRB = generateVector(RS, RE)
    LSRA = generateVector(LS, LH)
    LSRB = generateVector(LS, LE)

    #Create Fully body cooridinate system
    zAxis = generateVector(TS, NK)
    zAxis = normalizeVector(zAxis)
    yAxis = generateVector(RS, LS)
    yAxis = normalizeVector(yAxis)
    xAxis = np.cross(zAxis, yAxis)
    xAxis = normalizeVector(xAxis)
    yAxis = np.cross(zAxis, xAxis)
    yAxis = normalizeVector(yAxis)

    bodyMatrix = np.matrix([xAxis, yAxis, zAxis])

    #Create Left Arm coordinate system
    leftZAxis = normalizeVector(generateVector(LW, LS))
    leftYAxis = normalizeVector(np.cross(LEBA, LEBB))
    leftXAxis = normalizeVector(np.cross(leftYAxis, leftZAxis))

    leftMatrix = np.matrix([leftXAxis, leftYAxis, leftZAxis])

    #Create right Arm coordinate system
    rightZAxis = normalizeVector(generateVector(RW, RS))
    rightYAxis = normalizeVector(np.cross(REBA, REBB))
    rightXAxis = normalizeVector(np.cross(rightYAxis, rightZAxis))

    rightMatrix = np.matrix([rightXAxis, rightYAxis, rightZAxis])

    #Matrix magic
   
    #Get right angles
    rightAngles = calculateRPY(bodyMatrix, rightMatrix)
    RSR = -1 * calculateTheta(RSRA, RSRB) #rightAngles[0]

    #Sholder roll is horizontalish, pitch should be 0
    if RSR > -1.6 and RSR < -1.4:
        RSP = 0.0
    else:
        RSP = -1 * rightAngles[1]

    RSY = -1 * calculateYaw(REBA, REBB, RE)# 0 #-1 * rightAngles[2] 
    REP = calculateTheta(REBA, REBB)

    #Get left angles
    leftAngles = calculateRPY(bodyMatrix, leftMatrix)
    LSR = calculateTheta(LSRA, LSRB)#leftAngles[0]

    #Sholder roll is horizontalish, pitch should be 0
    if LSR < 1.6 and LSR > 1.4:
        LSP = 0.0
    else:
        LSP = -1 * leftAngles[1]

    LSY = -1 * calculateYaw(LEBA, LEBB, LE)# 0 # leftAngles[2] 
    LEP = calculateTheta(LEBA, LEBB)

    REP = reduceRadians(REP)
    LEP = reduceRadians(LEP)
    RSY = reduceRadians(RSY)
    LSY = reduceRadians(LSY)
    RSR = reduceRadians(RSR)
    LSR = reduceRadians(LSR)
    RSP = reduceRadians(RSP)
    LSP = reduceRadians(LSP)

    #Return the angles
    angles = np.array([REP, LEP, RSY, LSY, RSR, LSR, RSP, LSP])

    return angles





#Simple ITERO. This works. Don't touch it!!!! 
def generateAngles(jsonString):
    #Make the jsonString into a dictionary to get the data
    dict = json.loads(jsonString)
    '''
    Order for joint angles: [REB, LEB, RSY, LSY, RSR, LSR, RSP, LSP]
    To calculate each angle, take the angle between the vectors formed
    by the specified points*:

    REB- RE-RW, RE-RS
    LEB- LE-LW, LE-LS
    RSY*-
    LSY*-
    RSR- RS-NK, RS-RE
    LSR- LS-NK, LS-LE
    RSP**- cross(RSR), NK-HD
    LSP**- cross(LSR), NK-HD

    *The shoulder yaws require a bit more complexity. First we must
    make a constant vector in the z direction and take the cross product of the two vectors
    specified. Then we need to cross that vector with the constant vector and get the angle between
    that vector and the vector from the cross product of the orginal vectors

    JOINTS = [
       0 'head',
       1 'neck',
       2 'torso',
       3 'left_shoulder',
       4 'left_elbow',
       5 'left_hand',
       6 'left_hip',
       7 'left_knee',
       8 'left_foot',
       9 'right_shoulder',
       10 'right_elbow',
       11 'right_hand',
       12 'right_hip',
       13 'right_knee',
       14 'right_foot'
        ]
    '''
    #Get the joint list
    jointList = dict["Joints"]

    #Create coordianate vectors for each joint point
    #Indices from above
    RH = getPointFromJoint(jointList[12])
    RW = getPointFromJoint(jointList[11])
    RE = getPointFromJoint(jointList[10])
    RS = getPointFromJoint(jointList[9])
    NK = getPointFromJoint(jointList[1])
    HD = getPointFromJoint(jointList[0])
    LH = getPointFromJoint(jointList[6])
    LS = getPointFromJoint(jointList[3])
    LE = getPointFromJoint(jointList[4])
    LW = getPointFromJoint(jointList[5])

    #Calculate Rich's angles
    #RS, RE
    vectorA = generateVector(RE, RS)
    vectorB = generateVector(LE, LS)


    #Generate the vectors that we will need to calculate the angles
    REBA = generateVector(RE, RW)
    REBB = generateVector(RE, RS)
    LEBA = generateVector(LE, LW)
    LEBB = generateVector(LE, LS)

    RSYA = generateVector(RE, RW)
    RSYB = generateVector(RE, RS)
    LSYA = generateVector(LE, LW)
    LSYB = generateVector(LE, LS)

    RSRA = generateVector(RS, RH)
    RSRB = generateVector(RS, RE)
    LSRA = generateVector(LS, LH)
    LSRB = generateVector(LS, LE)

    RSPA = np.cross(RSRA, RSRB)
    RSPB = generateVector(NK, HD)
    LSPA = np.cross(LSRA, LSRB)
    LSPB = generateVector(NK, HD)

    #Generate the angles
    REB = calculateTheta(REBA, REBB)
    LEB = calculateTheta(LEBA, LEBB)
    RSY = calculateYaw(RSYA, RSYB, RE)
    LSY = calculateYaw(LSYA, LSYB, LE)
    RSR = -1 * calculateTheta(RSRA, RSRB)  #getRoll(vectorA)
    LSR = calculateTheta(LSRA, LSRB)  #getRoll(vectorB)
    RSP = -1 * calculateTheta(RSPA, RSPB)  #getPitch(vectorA)
    LSP = -1 * calculateTheta(LSPA, LSPB)  #getPitch(vectorB)

    #Reduce the angles! Might not be needed after all. I thought there was a bug
    # but it was somewhere else.
    REB = reduceRadians(REB)
    LEB = reduceRadians(LEB)
    RSY = reduceRadians(RSY)
    LSY = reduceRadians(LSY)
    RSR = reduceRadians(RSR)
    LSR = reduceRadians(LSR)
    RSP = 0#reduceRadians(RSP)
    LSP = 0#reduceRadians(LSP)


    angles = np.array([REB, LEB, RSY, LSY, RSR, LSR, RSP, LSP])
    #print len(angles)
    return angles


if __name__ == '__main__':
    #If ran alone try to convert a file containing json strings
    # of the appropriate data to a file of angle arrays
    filename = "Positions.log"
    output = "JointAngles.txt"

    if len(sys.argv) == 2:
        filename = sys.argv[1]
    elif len(sys.argv) == 3:
        output = sys.argv[2]
    elif len(sys.argv) > 3:
        print "Usage: ./AngleCalculator.py <Input File> <Output File>"

    #Open the input file
    fIn = open(filename, 'r')
    #Open the output file
    fOut = open(output, 'w')
    #Iterate through line by line of the input
    #creating a row vector for each line in the output
    for line in fIn:
        vector = generateAngles(line)
        fOut.write(str(vector) + "\n")
