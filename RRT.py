import cv2
import numpy as np
import random
import argparse


class Nodes: #Stores RRT Graph
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.xParent = []
        self.yParent = []
global coordinates
global xGoal 
global yGoal
#RGB definitions         
BLUE = (255,0,0)
GREEN = (0,255,0)
RED = (0,0,255)
BLACK = (0,0,0)
# returns the angle between 2 points
def angle_two_points(x1,y1,x2,y2):
    return np.arctan2(y2-y1,x2-x1)

# returns the distance between 2 points  
def distance_two_points(x1,y1,x2,y2):  
    return np.sqrt(((x2-x1)**2)+((y2-y1)**2))

# generate random point
def rand_point(h,l):
    return (random.randint(0, l),random.randint(0, l))

# Convert a pixel value to real world (x,y) points
def PixelToRealWorld(xPixel,yPixel): 
    # (0.4,-0.2) corresponds to coordinates(4,5)
    # (0.2,-0.2) corresponds to coordinates(6,7)
    # (0.2,0.2) corresponds to coordinates(8,9)

    xChangeInPixel = coordinates[6] - xPixel
    yChangeInPixel = yPixel - coordinates[7]
    
    # Define the workspace pixel range
    xPixelRange = abs(coordinates[4] -coordinates[6])
    #print("The x pixel range is : " + str(xPixelRange) )
    yPixelRange = abs(coordinates[7] - coordinates[9])

    xScaling= (0.4-0.2)/(xPixelRange)
    yScaling = (0.2 + 0.2) / (yPixelRange)
    xRealWorld = 0.2 + xScaling  * xChangeInPixel
    yRealWorld = -0.2 + yScaling * yChangeInPixel

    #print("The x,y value of your point is : " + str(xRealWorld) + " , " + str(yRealWorld))
    return xRealWorld , yRealWorld

    # Convert a pixel value to real world (x,y) points
def pixelArrayConversion(xPixelArray,yPixelArray): 
    
    xGoal = []
    yGoal = []

    for i in range(len(xPixelArray)):   
        xRealWorld , yRealWorld = PixelToRealWorld(xPixelArray[i],yPixelArray[i])
        xGoal.append(xRealWorld)
        yGoal.append(yRealWorld)
    
    print("\n")
    print("The KINOVOA x-values are: " + str(xGoal))
    np.savetxt('goalx.txt', xGoal,  delimiter=',')
    print("\n")

    print("\n")
    print("The KINOVOA y-values are: " + str(yGoal))
    np.savetxt('goaly.txt', yGoal, delimiter=',')
    print("\n")

# return the current point index
def closestNode(x,y):
    distance_list=[]
    for i in range(len(nodeList)):
        distance_list.append(distance_two_points(x,y,nodeList[i].x,nodeList[i].y))
    return distance_list.index(min(distance_list))

# check collision
def collision(x1,y1,x2,y2):
    color=[]
    x = list(np.arange(x1,x2,(x2-x1)/50))
    y = list(((y2-y1)/(x2-x1))*(x-x1) + y1) # y = mx + b
    xPixelRange = abs(coordinates[4] - coordinates[6])
    yPixelRange = abs(coordinates[7] - coordinates[9])

    for i in range(len(x)):
        pvalue_x = int(x[i])
        pvalue_y = int(y[i])
        color.append(img[pvalue_y,pvalue_x]) # append value of pixel to array
        #if (pvalue_x < xPixelRange or pvalue_x > xPixelRange) and (pvalue_y < yPixelRange or pvalue_y > yPixelRange):
            #return True  # collision
    if (0 in color):
        return True #collision
    else:
        return False #no collision

# check the  collision with obstacle and boarder
def check_collision(x1,y1,x2,y2):
    theta = angle_two_points(x2,y2,x1,y1) 
    x = x2 + step_size*np.cos(theta)
    y = y2 + step_size*np.sin(theta)
    columns,rows = img.shape
    if (y < 0 or y > columns or x < 0 or x > rows):
        nodefound= False
    else:
        if not collision(x,y,goal[0],goal[1]):
            goalfound = True
        else:
            goalfound= False
        if not collision(x,y,x2,y2):
            nodefound= True
        else:
            nodefound= False

    return(x,y,goalfound,nodefound)


def drawCircle(event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img2,(x,y),5,BLUE,-1)
        coordinates.append(x)
        coordinates.append(y)

def RRT(img, img2, start, goal):

    h,l= img.shape

    nodeList[0] = Nodes(start[0],start[1])
    nodeList[0].xParent.append(start[0])
    nodeList[0].yParent.append(start[1])

    # display start and goal
    cv2.circle(img2, (start[0],start[1]), 5,GREEN, thickness=5, lineType=4)
    cv2.circle(img2, (goal[0],goal[1]), 5,RED, thickness=5, lineType=4)

    i=1
    foundpath = False
    while foundpath == False:
        randx,randy = rand_point(h,l)

        closestIndex = closestNode(randx,randy)
        nearest_x = nodeList[closestIndex].x
        nearest_y = nodeList[closestIndex].y

        curr_x,curr_y,goalfound,nodefound= check_collision(randx,randy,nearest_x,nearest_y)

        coord_tx_ty = int(curr_x),int(curr_y)
        coord_node_closestIndex = int(nodeList[closestIndex].x),int(nodeList[closestIndex].y)
        #evalute node distance from goal point
        node_diff = tuple(np.subtract(coord_tx_ty,goal))
        if (goalfound and nodefound and (node_diff[0] < step_size and node_diff[1] < step_size)) == True:
            nodeList.append(i)
            nodeList[i] = Nodes(curr_x,curr_y)
            nodeList[i].xParent = nodeList[closestIndex].xParent.copy()
            nodeList[i].xParent.append(curr_x)
            nodeList[i].yParent = nodeList[closestIndex].yParent.copy()
            nodeList[i].yParent.append(curr_y)
            cv2.circle(img2, coord_tx_ty, 2,RED,thickness=3, lineType=8)
            cv2.line(img2, coord_tx_ty, coord_node_closestIndex, GREEN, thickness=1, lineType=4)
            cv2.line(img2, coord_tx_ty, (goal[0],goal[1]), BLUE, thickness=2, lineType=4)
            cv2.imwrite("outputImage.jpg",img2)

            nodeList[i].xParent.append(coordinates[2])
            nodeList[i].yParent.append(coordinates[3])

            print("\n")
            print("The x value are: " + str(nodeList[i].xParent))
            print("\n")
            print("The y values are: " + str(nodeList[i].yParent))
            print("\n")

            pixelArrayConversion(nodeList[i].xParent, nodeList[i].yParent)


            break

        elif nodefound :
            nodeList.append(i)
            nodeList[i] = Nodes(curr_x,curr_y)
            nodeList[i].xParent = nodeList[closestIndex].xParent.copy()
            nodeList[i].yParent = nodeList[closestIndex].yParent.copy()
            nodeList[i].xParent.append(curr_x)
            nodeList[i].yParent.append(curr_y)
            i=i+1
            cv2.circle(img2, coord_tx_ty, 2,RED,thickness=3, lineType=8)
            cv2.line(img2, coord_tx_ty, coord_node_closestIndex, GREEN, thickness=1, lineType=8)
            cv2.imshow("finalimage",img2)
            cv2.waitKey(1)
            continue
    #print final path
    for j in range(len(nodeList[i].xParent)-1):
        coord_node = int(nodeList[i].xParent[j]),int(nodeList[i].yParent[j])
        nxt_coord = int(nodeList[i].xParent[j+1]),int(nodeList[i].yParent[j+1])
        cv2.line(img2,coord_node,nxt_coord,BLUE, thickness=2, lineType=4)
    cv2.imwrite("outputImage.jpg",img2)




if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-selectPoint', action='store_true')
    param = parser.parse_args()

    img = cv2.imread("binarization.jpg",0)
    img2 = cv2.imread("binarization.jpg")
    step_size = 20
    nodeList = [0]
   
    coordinates=[]
    if param.selectPoint:
        print("Choose Start and goal Points and Choose Image Corners - Click Space when done")
        cv2.namedWindow('image')
        cv2.setMouseCallback('image',drawCircle)
        while(1):
            cv2.imshow('image',img2)
            k = cv2.waitKey(20)
            if k == 32:
                break
    PixelToRealWorld(590,90)
    #start = (0,1)
    #goal = (2,3)
    #topleft = (4,5)
    #top right = (6,7)
    #bottom left = (8,9)
    
    #set hidden border
    topleft_border_coord = (coordinates[4],coordinates[5])
    topright_border_coord = (coordinates[6],coordinates[7])
    bottomright_border_coord = (coordinates[8],coordinates[9])
    bottomleft_border_coord = (coordinates[4],coordinates[9])
    cv2.line(img, topleft_border_coord, topright_border_coord, BLACK, thickness=5, lineType=8)
    cv2.line(img, topright_border_coord, bottomright_border_coord, BLACK, thickness=5, lineType=8)
    cv2.line(img, bottomleft_border_coord, bottomright_border_coord, BLACK, thickness=5, lineType=8)
    cv2.line(img, topleft_border_coord, bottomleft_border_coord, BLACK, thickness=5, lineType=8)
    cv2.imwrite("test.jpg",img)
    img = cv2.imread("test.jpg",0)
    #img2 = cv2.imread("test.jpg")
    start=(coordinates[0],coordinates[1])
    goal=(coordinates[2],coordinates[3])

    RRT(img, img2, start, goal)
    
