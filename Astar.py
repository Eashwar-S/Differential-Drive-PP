from node import Node
from collections import deque
from queue import PriorityQueue
import heapq
import numpy as np
import math


######################################
#          Workspace
######################################


def isValidWorkspace(pt, r=1, radiusClearance=0):  # To be modified
    x, y = pt

    # ------------------------------------------------------------------------------
    #                              Circle 1 pts
    # ------------------------------------------------------------------------------
    ptInCircle1 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(2 / r)
                                                  ) ** 2 - math.floor((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 2 pts
    # ------------------------------------------------------------------------------
    ptInCircle2 = (x - math.floor(7 / r)) ** 2 + (y - math.floor(8 / r)
                                                  ) ** 2 - math.floor((1 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 3 pts
    # ------------------------------------------------------------------------------
    ptInCircle3 = (x - math.floor(5 / r)) ** 2 + (y - math.floor(5 / r)
                                                  ) ** 2 - math.floor((2 + radiusClearance) / r) ** 2 <= 0

    # ------------------------------------------------------------------------------
    #                              Circle 4 pts
    # ------------------------------------------------------------------------------
    ptInCircle4 = (x - math.floor(3 / r)) ** 2 + (y - math.floor(8 / r)
                                                  ) ** 2 - math.floor((1 + radiusClearance) / r) ** 2 <= 0

    # # --------------------------------------------------------------------------------
    # #                             Rectangle pts
    # # --------------------------------------------------------------------------------
    # X = np.array([2.25, 3.75, 3.75, 2.25]) / r
    # Y = np.array([1.25, 1.25, 1.75, 1.75]) / r
    # ptInRectangle = (y - Y[0]) <= ((Y[1] - Y[0]) / (X[1] - X[0])) * (x - X[0]) - \
    #               radiusClearance / r * (1 + math.sqrt(((Y[1] - Y[0]) / (X[1] - X[0])) ** 2)) and \
    #               0 >= (Y[2] - Y[1]) * (x - X[1]) + \
    #               radiusClearance / r * (1 + math.sqrt((Y[2] - Y[1]) ** 2)) and \
    #               (y - Y[2]) >= ((Y[3] - Y[2]) / (X[3] - X[2])) * (x - X[2]) + \
    #               radiusClearance / r * (1 + math.sqrt(((Y[3] - Y[2]) / (X[3] - X[2])) ** 2)) and \
    #               0 <= (Y[0] - Y[3]) * (x - X[3]) - \
    #               radiusClearance / r * (1 + math.sqrt((Y[0] - Y[3]) ** 2))
    
    # --------------------------------------------------------------------------------
    #                             Rectangle pts
    # --------------------------------------------------------------------------------
    X = np.array([2.25, 3.75, 3.75, 2.25]) / r
    Y = np.array([1.25, 1.25, 1.75, 1.75]) / r
    ptInRectangle = (y - Y[0]) >= ((Y[1] - Y[0]) / (X[1] - X[0])) * (x - X[0]) - \
                  radiusClearance / r  and \
                  0 <= (Y[2] - Y[1]) * (x - X[1]) + \
                  radiusClearance / r  and \
                  (y - Y[2]) <= ((Y[3] - Y[2]) / (X[3] - X[2])) * (x - X[2]) + \
                  radiusClearance / r  and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - \
                  radiusClearance / r 

    # --------------------------------------------------------------------------------
    #                             Square 1 pts
    # --------------------------------------------------------------------------------
    X = np.array([0.2, 1.8, 1.8, 0.2])/r
    Y = np.array([4.2, 4.2, 5.8, 5.8])/r
    ptInSquare1 = (y - Y[0]) >= ((Y[1] - Y[0]) / (X[1] - X[0])) * (x - X[0]) - \
                  radiusClearance / r  and \
                  0 <= (Y[2] - Y[1]) * (x - X[1]) + \
                  radiusClearance / r and \
                  (y - Y[2]) <= ((Y[3] - Y[2]) / (X[3] - X[2])) * (x - X[2]) + \
                  radiusClearance / r  and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - \
                  radiusClearance / r 

    # --------------------------------------------------------------------------------
    #                             Square 2 pts
    # --------------------------------------------------------------------------------
    X = np.array([8.2, 9.8, 9.8, 8.2])/r
    Y = np.array([4.2, 4.2, 5.8, 5.8])/r
    ptInSquare2 = (y - Y[0]) >= ((Y[1] - Y[0]) / (X[1] - X[0])) * (x - X[0]) - \
                  radiusClearance / r and \
                  0 <= (Y[2] - Y[1]) * (x - X[1]) + \
                  radiusClearance / r and \
                  (y - Y[2]) <= ((Y[3] - Y[2]) / (X[3] - X[2])) * (x - X[2]) + \
                  radiusClearance / r  and \
                  0 >= (Y[0] - Y[3]) * (x - X[3]) - \
                  radiusClearance / r

    if ptInCircle1 or ptInCircle2 or ptInCircle3 or ptInCircle4 or ptInRectangle or ptInSquare1 or ptInSquare2:
        return False
    return True


# checks whether next action is near an obstacle or ill defined
def isSafe(newState, r=1, radiusClearance=0):
    col = (10 / r)
    row = (10 / r)

    if newState[0] < 0 or newState[0] > col or newState[1] < 0 or newState[1] > row:
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


def normalize(startPosition, startOrientation):
    x, y = startPosition
    t = startOrientation
    # x = round(x / threshDistance) * threshDistance
    # y = round(y / threshDistance) * threshDistance
    # t = round(t / threshAngle) * threshAngle
    return [x, y, t]


# Calculating the Euclidean distance
def distance(startPosition, goalPosition):
    sx, sy = startPosition
    gx, gy = goalPosition
    return math.sqrt((gx - sx) ** 2 + (gy - sy) ** 2)


# generates optimal path for robot
def generatePath(q, startPosition, startOrientation, goalPosition, nodesExplored, ul, ur, radiusClearance=0):
    # normalize goal and start positions
    sx, sy, st = normalize(startPosition, startOrientation)
    gx, gy, gt = normalize(goalPosition, 0)

    # Initializing root node
    key = str(sx) + str(sy) + str(st)
    root = Node(np.array([sx, sy, st]), 0.0, 0.0, None)
    nodesExplored[key] = root

    count = 1
    heapq.heappush(q, (root.cost, count, root))

    while (len(q) > 0):
        _, _, currentNode = heapq.heappop(q)

        if (distance(currentNode.state[0:2], goalPosition) <= 0.3):
            sol = printPath(currentNode)
            return [True, sol]

        for actions in range(8):
            x, y, t = currentNode.state
            # newOrientation = math.radians((threshAngle*theta + t)%360)
            # newPosX = threshDistance*math.cos(newOrientation) + x
            # newPosY = threshDistance*math.sin(newOrientation) + y
            # newState = np.array(normalize([newPosX,newPosY],newOrientation,threshDistance,threshAngle))
            if actions == 0:
                newPosX, newPosY, newOrientation = constraints(x, y, t, 0, ul)
            elif actions == 1:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ul, 0)
            elif actions == 2:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ul, ul)
            elif actions == 3:
                newPosX, newPosY, newOrientation = constraints(x, y, t, 0, ur)
            elif actions == 4:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ur, 0)
            elif actions == 5:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ur, ur)
            elif actions == 6:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ul, ur)
            else:
                newPosX, newPosY, newOrientation = constraints(x, y, t, ur, ul)

            newState = np.array(
                normalize([newPosX, newPosY], newOrientation))
            s = str(newState[0]) + str(newState[1]) + str(newState[2])
            threshDistance = math.sqrt((newPosX - x)**2 + (newPosY - y)**2)
            if (s not in nodesExplored):
                if (isSafe(newState, 1, radiusClearance)):
                    newCostToCome = currentNode.costToCome + threshDistance
                    newCost = newCostToCome + \
                              distance([newPosX, newPosY], [gx, gy])

                    newNode = Node(newState, newCost, newCostToCome, currentNode)
                    nodesExplored[s] = newNode

                    heapq.heappush(q, (newNode.cost, count, newNode))
                    count += 1
            else:
                if (nodesExplored[s].cost > currentNode.costToCome + threshDistance + distance([newPosX, newPosY],
                                                                                               [gx, gy])):
                    nodesExplored[s].costToCome = currentNode.costToCome + \
                                                  threshDistance
                    nodesExplored[s].cost = nodesExplored[s].costToCome + \
                                            distance([newPosX, newPosY], [gx, gy])
                    nodesExplored[s].parent = currentNode

    return [False, None]


def constraints(X0, Y0, Theta0, UL, UR):
    t = 0
    r = 0.0038  # Radius of the wheel is 0.038 meters
    L = 0.009175  # Distance between the wheels is 0.3175 meters
    dt = 0.1
    X1 = 0
    Y1 = 0
    dtheta = 0
    Theta0 = 3.14 * Theta0 / 180
    Theta1 = Theta0
    while t < 1:
        t = t + dt
        X0 = X0 + X1
        Y0 = Y0 + Y1
        dx = r * (UL + UR) * math.cos(Theta1) * dt
        dy = r * (UL + UR) * math.sin(Theta1) * dt
        dtheta = (r / L) * (UR - UL) * dt
        X1 = X1 + dx
        Y1 = Y1 + dy
        Theta1 = Theta1 + 0.5 * dtheta
        Xn = X0 + X1
        Yn = Y0 + Y1
        Thetan = 180 * (Theta1) / 3.14
    return Xn, Yn, Thetan


if __name__ == "__main__":
    nodesExplored = {}
    # q = deque()
    # q = PriorityQueue()
    q = []
    startPosition = np.array([295, 195])
    startOrientation = 0
    goalPosition = np.array([5, 5])
    print(generatePath(q, startPosition, startOrientation,
                       goalPosition, nodesExplored, 5, 30))
