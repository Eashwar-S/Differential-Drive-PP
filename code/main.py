import pygame
import numpy as np
from Astar import *
import time


def triangleCoordinates(start, end, triangleSize = 5):
    
    rotation = (math.atan2(start[1] - end[1], end[0] - start[0])) + math.pi/2
    # print(math.atan2(start[1] - end[1], end[0] - start[0]))
    rad = math.pi/180

    coordinateList = np.array([[end[0],end[1]],
                              [end[0] + triangleSize * math.sin(rotation - 165*rad), end[1] + triangleSize * math.cos(rotation - 165*rad)],
                              [end[0] + triangleSize * math.sin(rotation + 165*rad), end[1] + triangleSize * math.cos(rotation + 165*rad)]])

    return coordinateList

def writeSolutionToFile(solution):
    # Writing all explored nodes in text file.
    sol = np.array(solution)

    #converting from image coordinates to world cooridnates
    sol[:,0] = -5 + sol[:,0]
    sol[:,1] = 5 - sol[:,1]
    sol[:,2] = 360 - sol[:,2]
    
    #Solution is stored as goal to start so we reverse it
    for i in range(6):
        sol[:,i] = -1*sol[:,i][::-1]
    np.savetxt('solution.txt',sol, delimiter=',')


def writeParametersForGazebo(dt,is1,is2,iorientation):
    #input from world coordinates to gazebo coordinates 
    s1 = -1*is1
    s2 = -1*is2
    orientation = math.radians(iorientation) + math.pi #gazebo works in radians

    params = np.array([dt,s1,s2,orientation]) 
    np.savetxt('gazebo_params.txt',params,delimiter=',')

###################################################
#                  Parameters 
###################################################
#------------------------------
#  Getting user Inputs
#------------------------------
# clearance = 0
# print('Robot considered is Turtlebot 2:')
# print("Enter cleareance")
# clearance = float(input())

# print('Enter start location s1 between -5 and 5')
# s1 = 5 + float(input())
# print('Enter start location s2 between -5 and 5')
# s2 = 5 - float(input())
# print('Enter the angle of the robot in degrees')
# startOrientation = 360 - float(input())

# print('Enter goal location g1 between -5 and 5')
# g1 = 5 + float(input())
# print('Enter goal location g2 between -5 and 5')
# g2 = 5 - float(input())

# print('Enter left wheel rotational velocity')
# ul = float(input())
# print('Enter right wheel rotational velocity')
# ur = float(ijput())

iul = 20
iur = 20
is1 = -4  #-4   
is2 = -4  #-3   
ig1 = 4   #0    
ig2 = 2.5 #-3   
istartOrientation = 0

#---------------------------------
# Inputs From World Coordinates 
# To Pygame Coordinates
#---------------------------------
startOrientation = 360 - istartOrientation
ul = iul
ur = iur
s1 = 5+(is1)
s2 = 5-(is2)
g1 = 5+(ig1)
g2 = 5-(ig2)

#---------------------------
#  Precision Parameters
#---------------------------
threshDistance = 0.1
clearance = 0.3
# threshAngle, dt = setPrecisionParameters(ul, ur)
dt = 0.8
threshAngle = 5

#---------------------------
#  Robot parameters
#---------------------------
wheelDist = 0.2116 # 0.3175/6 * 4
wheelRadius = 0.038
robotParams = [ul,ur,wheelRadius,wheelDist]
robotRadius = 0.177

#-------------------------------
#  Parameters needed by gazebo
#-------------------------------
#dt - affects publishing rate
#   - dt must be of resolution 0.1 
#   - restricting frequency to 10Hz in gazebo
#   - 1/frequency*dt must be a whole number 

#is1,is2,iorientation- initial pose for robot
writeParametersForGazebo(dt,is1,is2,istartOrientation)

#----------------------------
#  Display parameters
#----------------------------
pygame.init()

res = 1.0  #resolution of grid 
scale = 80 #scale of grid

white = (255,255,255)
black = (0,0,0)
red = (255,0,0)
green = (0,255,0)
blue = (0,0,255)
yellow = (255,255,0)

size_x = 10
size_y = 10
gameDisplay = pygame.display.set_mode((size_x*scale,size_y*scale))

#----------------------------
# Start and goal coordinates
#----------------------------
startCoor = np.float32((np.float32([s1,s2,startOrientation]))/res)
goalCoor = np.float32((np.float32([g1,g2,0]))/res)

startEndCoor = [startCoor,goalCoor]


############################################################
#                 Display Obstacles
############################################################
circlePts1 = [7,2,1]
circlePts2 = [7,8,1]
circlePts3 = [5,5,1]
circlePts4 = [3,8,1]


pygame.draw.circle(gameDisplay, red, (circlePts1[0]*scale,circlePts1[1]*scale), circlePts1[2]*scale)
pygame.draw.circle(gameDisplay, red, (circlePts2[0]*scale,circlePts2[1]*scale), circlePts2[2]*scale)
pygame.draw.circle(gameDisplay, red, (circlePts3[0]*scale,circlePts3[1]*scale), circlePts3[2]*scale)
pygame.draw.circle(gameDisplay, red, (circlePts4[0]*scale,circlePts4[1]*scale), circlePts4[2]*scale)
pygame.draw.rect(gameDisplay,red,[scale*2.25,scale*1.25,scale*1.5,scale*1.5])
pygame.draw.rect(gameDisplay,red,[scale*0.25,scale*4.25,scale*1.5,scale*1.5])
pygame.draw.rect(gameDisplay,red,[scale*8.25,scale*4.25,scale*1.5,scale*1.5])


############################################################
#          Draw Explored Nodes and solution path
############################################################
nodesExplored = {}
q = []

if(not isSafe(startCoor,res,clearance + robotRadius) or not isSafe(goalCoor,res,clearance + robotRadius)):
    pygame.draw.rect(gameDisplay,blue,(startCoor[0]*res*scale,startCoor[1]*res*scale, \
                                 res*2,res*2))

    pygame.draw.circle(gameDisplay,blue,(int(goalCoor[0]*res*scale),int(goalCoor[1]*res*scale)), \
                                  math.floor(0.3*res*scale))

    pygame.draw.rect(gameDisplay,white,(goalCoor[0]*res*scale,goalCoor[1]*res*scale, \
                                 res*2,res*2))
    basicfont = pygame.font.SysFont(None, 48)
    text = basicfont.render('Start or goal position must be in a valid workspace', True, (255, 0, 0), (255, 255, 255))
    textrect = text.get_rect()
    textrect.centerx = gameDisplay.get_rect().centerx
    textrect.centery = gameDisplay.get_rect().centery
 
    gameDisplay.blit(text, textrect)
    pygame.display.update()
    pygame.time.delay(2000)

else:
    startTime = time.time()  # Start time of simulation
    print('Exploring nodes...')
    success,solution = generatePath(q,startEndCoor,nodesExplored,robotParams,dt,clearance+robotRadius,threshDistance,threshAngle)
    endTime= time.time()
    
    #############################################
    #      Drawing 
    #############################################
    if(success):
        print('Optimal path found')
        print("Total time taken for exploring nodes "+ str(endTime-startTime) +" seconds.")
        writeSolutionToFile(solution)

        draw = True
        while draw:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    quit()
             
            #draw nodesExplored
            for s in nodesExplored:
                if(nodesExplored[s].parent):
                    pt = nodesExplored[s].state[0:2]
                    ptParent = nodesExplored[s].parent.state[0:2]
                    x,y = pt*scale*res
                    x2,y2 = ptParent*scale*res

                    # draw explored nodes
                    pygame.draw.line(gameDisplay,white,(x2,y2),(x,y),1)
                    # pygame.draw.circle(gameDisplay,green,(int(x),int(y)),4)
                    triangle = triangleCoordinates([x2,y2],[x,y],5)
                    pygame.draw.polygon(gameDisplay, green,[tuple(triangle[0]),tuple(triangle[1]),tuple(triangle[2])])

                #draw start and goal locations
                pygame.draw.rect(gameDisplay,blue,(startCoor[0]*res*scale,startCoor[1]*res*scale, \
                                 res*2,res*2))

                pygame.draw.circle(gameDisplay,blue,(int(goalCoor[0]*res*scale),int(goalCoor[1]*res*scale)), \
                                  math.floor(0.3*res*scale))

                pygame.draw.rect(gameDisplay,white,(goalCoor[0]*res*scale,goalCoor[1]*res*scale, \
                                 res*2,res*2))
                pygame.display.update()
           
            # draw solution path
            for i in range(len(solution)-2,-1,-1):
                pt = solution[i][0:2]
                pt1 = solution[i+1][0:2]
                xt,yt = pt[0]*scale*res,pt[1]*scale*res
                x, y = pt1[0]*scale*res,pt1[1]*scale*res
                pygame.draw.line(gameDisplay,yellow,(xt,yt),(x,y),3)
                pygame.draw.circle(gameDisplay,red,(int(x),int(y)),4)
                pygame.display.update()
            pygame.time.delay(4000)
            draw = False

    else:
        print('Path not possible')
        basicfont = pygame.font.SysFont(None, 48)
        text = basicfont.render('Path can\'t be generated', True, (255, 0, 0), (255, 255, 255))
        textrect = text.get_rect()
        textrect.centerx = gameDisplay.get_rect().centerx
        textrect.centery = gameDisplay.get_rect().centery
     
        gameDisplay.blit(text, textrect)
        pygame.display.update()
        pygame.time.delay(2000)

pygame.quit()

