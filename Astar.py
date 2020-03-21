from node import Node
from collections import deque
from queue import PriorityQueue
import heapq
import numpy as np
import math


######################################
#          Workspace
######################################
def isValidWorkspace(pt, r, radiusClearance):  
    x, y = pt

    # ------------------------------------------------------------------------------
    #                              Circle 1 pts
    # ------------------------------------------------------------------------------
    ptInCircle1 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(2 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 2 pts
    # ------------------------------------------------------------------------------
    ptInCircle2 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(8 / r)) ** 2 - ((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 3 pts
    # ------------------------------------------------------------------------------
    ptInCircle3 = (x - math.floor(5 / r)) ** 2 + (y - math.floor(5 / r)) ** 2.0 - ((1.0 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 4 pts
    # ------------------------------------------------------------------------------
    ptInCircle4 = (x - math.floor(3 / r)) ** 2 + (y - math.floor(8 / r)) ** 2.0 - ((1.0 + radiusClearance) / r) ** 2 <= 0


    # --------------------------------------------------------------------------------
    #                             Rectangle pts
    # --------------------------------------------------------------------------------
    X = np.float32([2.25, 3.75, 3.75, 2.25]) / r
    Y = np.float32([1.25, 1.25, 1.75, 1.75]) / r
    ptInRectangle = y >= Y[0] - radiusClearance / r                        and \
                    0 >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                    y <= Y[2]+ radiusClearance / r                         and \
                    0 >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 

    # --------------------------------------------------------------------------------
    #                             Square 1 pts
    # --------------------------------------------------------------------------------
    X = np.float32([0.2, 1.8, 1.8, 0.2])/r
    Y = np.float32([4.2, 4.2, 5.8, 5.8])/r
    ptInSquare1 = y  >= Y[0] - radiusClearance / r                        and \
                  0  >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                  y  <= Y[2]+ radiusClearance / r                         and \
                  0  >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 


    # --------------------------------------------------------------------------------
    #                             Square 2 pts
    # --------------------------------------------------------------------------------
    X = np.float32([8.2, 9.8, 9.8, 8.2])/r
    Y = np.float32([4.2, 4.2, 5.8, 5.8])/r
    ptInSquare2 = y  >= Y[0] - radiusClearance / r                        and \
                  0  >= (Y[2] - Y[1]) * (x - X[1]) - radiusClearance / r  and \
                  y  <= Y[2]+ radiusClearance / r                         and \
                  0  >= (Y[0] - Y[3]) * (x - X[3]) - radiusClearance / r 

    if ptInCircle1 or ptInCircle2 or ptInCircle3 or ptInCircle4 or ptInRectangle or ptInSquare1 or ptInSquare2:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, r, radiusClearance):
    col = float(10 / r)
    row = float(10 / r)

    if newState[0] < 0.0 or newState[0] > col or newState[1] < 0.0 or newState[1] > row:
        return False
    return isValidWorkspace(newState[0:2], r, radiusClearance)


# prints solution path
def printPath(node):
    l = []
    current = node
    while (current):
        l.append(current.state)
        current = current.parent
    return l


# Normalizing angle and step size 
def normalize(coor,threshDistance ,threshAngle):
    x, y,t = coor
    x = round(x / threshDistance) * threshDistance
    y = round(y / threshDistance) * threshDistance
    t = round(t / threshAngle) * threshAngle
    return [x, y, t]


# CalcrobotParams[0]ating the Euclidean distance
def distance(startPosition, goalPosition):
    sx, sy,_ = startPosition
    gx, gy,_ = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)

# generates optimal path for robot
def generatePath(q, startEndCoor, nodesExplored,robotParams,dt,radiusClearance,threshDistance = 0.1,threshAngle = 5):

    # normalize goal and start positions
    sx, sy, st = normalize(startEndCoor[0],threshDistance,threshAngle)
    gx, gy, gt = normalize(startEndCoor[1],threshDistance,threshAngle)

    # Initializing root node
    key = str(sx) + str(sy) + str(st)
    root = Node(np.float32([sx, sy, st]), 0.0, 0.0, None)
    nodesExplored[key] = root

    count = 1
    heapq.heappush(q, (root.cost, count, root))

    while (len(q) > 0):
        _, _, currentNode = heapq.heappop(q)

        if (distance(currentNode.state, [gx,gy,gt]) <= 0.3):
            sol = printPath(currentNode)
            return [True, sol]


        for actions in range(8):
            x, y, t = currentNode.state
            
            # Defining actions based on constraints
            if actions == 0:
                newPosX, newPosY, newOrientation = constraints(x, y, t, 0, robotParams[0],robotParams, dt)
            elif actions == 1:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[0], 0,robotParams,dt)
            elif actions == 2:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[0], robotParams[0],robotParams,dt)
            elif actions == 3:
                newPosX, newPosY, newOrientation = constraints(x, y, t, 0, robotParams[1],robotParams,dt)
            elif actions == 4:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[1], 0, robotParams,dt)
            elif actions == 5:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[1], robotParams[1], robotParams,dt)
            elif actions == 6:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[0], robotParams[1], robotParams,dt)
            elif actions == 7:
                newPosX, newPosY, newOrientation = constraints(x, y, t, robotParams[1], robotParams[0], robotParams,dt)
           
 
            newState = np.array(normalize([newPosX,newPosY,newOrientation],threshDistance,threshAngle))            
            s = str(newState[0]) + str(newState[1]) + str(newState[2])

            if (s not in nodesExplored):
                if (isSafe(newState, 1, radiusClearance)):
                    newCostToCome = currentNode.costToCome + threshDistance
                    newCost = newCostToCome + distance(newState, [gx, gy,gt])

                    newNode = Node(newState, newCost, newCostToCome, currentNode)
                    nodesExplored[s] = newNode

                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if (nodesExplored[s].cost > currentNode.costToCome + threshDistance+ distance(newState, [gx, gy,gt])):
                    nodesExplored[s].costToCome = currentNode.costToCome + threshDistance
                    nodesExplored[s].cost = nodesExplored[s].costToCome + distance(newState, [gx, gy,gt])
                    nodesExplored[s].parent = currentNode

    return [False, None]


# Functions which defines actions considering the constraints
def constraints(X0, Y0,Theta0,UL,UR,robotParams,dt):
    r = robotParams[2]        # Radius of the wheel  
    L = robotParams[3]        # Distance between the wheels  

    dx = r/2 * (UL + UR) * math.cos(math.radians(Theta0)) * dt
    dy = r/2 * (UL + UR) * math.sin(math.radians(Theta0)) * dt

    dtheta = (r / L) * (UR - UL) * dt
    Xn = X0 + dx
    Yn = Y0 + dy
    Thetan = (Theta0 +  dtheta)%360
    return Xn, Yn, Thetan


if __name__ == "__main__":
    pass
    # startOrientation = 360 - 15
    # clearance = 0.1
    # q = []
    # ul = 2
    # ur = 2
    # s1 = 5+(-4)
    # s2 = 5-(4)
    # g1 = 5+(-5)
    # g2 = 5-(5)
    # nodesExplored = {}
    # res = 1

    # startPosition = np.float32((np.float32([s1,s2]))/res)
    # goalPosition = np.float32((np.float32([g1,g2]))/res)

    # generatePath(q,startPosition,startOrientation,goalPosition,nodesExplored,ul, ur,clearance+0.038) 





