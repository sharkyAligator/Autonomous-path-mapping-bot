import gym
import time
import pixelate_arena
import pybullet as p
import pybullet_data
import cv2
import os
import numpy as np
from cv2 import aruco
import math
from dis import dis
from tabnanny import check
import math


box_dist=0
inf=999999
d=set()
img=[]
antidose=[]
height,width=0,0
boxcenter=set()
graph={}
checkPosERR={}
Center=[]

def pri_path(child,path,k,startToEnd):
    if (path[child] == child):
        print(child)
        startToEnd.append(child)
        return startToEnd
    pri_path(path[child], path,k+1,startToEnd)
    n=chr(k)
    startToEnd.append(child)

def sortwght(node):
    return node[0]

    
                                   ##################################
                                   #############Dijisktra############
                                   ##################################
def dijisktra(str,end):
    startToEnd=[]
    dist_from_src={}
    visit={}
    path={}
    weight_node=set()

    for node in graph.keys():
        dist_from_src[node]=inf

    weight_node.add((0,str))  
    dist_from_src[str] = 0

    path[str] = str
        
    while(len(weight_node)!=0):
        weight_node = sorted(weight_node,key=sortwght,reverse=True)
        weight_node=set(weight_node)
        front=next(iter(weight_node))
        weight_node.discard(next(iter(weight_node)))
        parent_dist_from_src = front[0]
        parent_name = front[1]
        visit[parent_name] = 1

        for z in graph[front[1]]:
            if(z is None):
                continue

            child_wt = z[1]
            child_name = z[0]
            child_dist_from_src = child_wt + parent_dist_from_src
            if (child_dist_from_src < dist_from_src[child_name]):
                dist_from_src[child_name] = child_wt + parent_dist_from_src
                weight_node.add((child_dist_from_src, child_name))
                weight_node = sorted(weight_node,key=sortwght,reverse=True)
                weight_node=set(weight_node)
                path[child_name] = parent_name

    k=65
    pri_path(end, path,k,startToEnd)
    return startToEnd


def dist(a,b):
    x=(a[0]-b[0])*(a[0]-b[0])
    y=(a[1]-b[1])*(a[1]-b[1])
    return np.sqrt(x+y)


def p_right(pos):
    x=pos[0] + box_dist
    y=pos[1] 
    x=int(x)
    y=int(y)
    return getcolor(x,y)

    
def p_bottomLeft(pos):
    x=pos[0] - box_dist*np.cos(np.pi/3)
    y=pos[1] + box_dist*np.sin(np.pi/3)
    x=int(x)
    y=int(y)
    return getcolor(x,y)

def p_topRight(pos):
    x=pos[0] + box_dist*np.cos(np.pi/3)
    y=pos[1] - box_dist*np.sin(np.pi/3)
    x=int(x)
    y=int(y)
    return getcolor(x,y)
    


def p_topLeft(pos):
    x=pos[0] - box_dist*np.cos(np.pi/3)
    y=pos[1] - box_dist*np.sin(np.pi/3)
    x=int(x)
    y=int(y)
    return getcolor(x,y)
    
def p_left(pos):
    x=pos[0] - box_dist
    y=pos[1] 

    x=int(x)
    y=int(y)
    return getcolor(x,y)

def p_bottomRight(pos):
    x=pos[0] + box_dist*np.cos(np.pi/3)
    y=pos[1] + box_dist*np.sin(np.pi/3)
    x=int(x)
    y=int(y)
    return getcolor(x,y)

def error(co_ord):
    f=0
    for node in boxcenter:
        if(dist(co_ord,node) < 10):
            return node[0],node[1]

    return co_ord[0],co_ord[1]
    

def getcolor(cx,cy):

    if(cx>width or cy>height or cx<0 or cy<0):
        return None

    cx,cy=error((cx,cy))

    pixel_center = hsv[cy, cx]
    h=pixel_center[0]
    s=pixel_center[1]
    v=pixel_center[2]

    if( h>=1 and h<=163 and s<=220 ):
        return ((cx,cy),4)
    elif( h>=1 and s<=179 ):
        antidose.append((cx,cy))
        return ((cx,cy),1)
    elif( s>=200 and v<=120 ):
        return ((cx,cy),3)
    elif( h>=1 and h<=23 ):
        return ((cx,cy),1)
    elif( h>=62 and s>=231 ):
        return ((cx,cy),20)
    elif( h>=5 and h<=55 ):
        return ((cx,cy),2)
    elif( h<=21 and v>=198 ):
        return ((cx,cy),1)
    else:
        return None

def adjacentnode(point):
    adjacent = []
    adjacent.append(p_right(point))
    adjacent.append(p_topRight(point))
    adjacent.append(p_topLeft(point))
    adjacent.append(p_bottomRight(point))
    adjacent.append(p_bottomLeft(point))
    adjacent.append(p_left(point))
    graph[point]=adjacent
    

def mainFunction(frame):
    global box_dist
    global graph
    global graphKeyERR
    global checkPosERR
    global d
    global boxcenter
    global height,width
    global img
    global antidose
    global Center
    global hsv

    box_dist=0
    graph={}
    graphKeyERR=set()
    checkPosERR={}
    img=[]
    height=0
    width=0
    antidose=[]
    boxcenter=set()
    d=set()
    Center=[]

    img=frame.copy()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    height,width,_=frame.shape
    lower_blue = np.array([0, 0, 1])
    upper_blue = np.array([255, 255, 255])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    boxcenter=set()
    Center=[]
    for contour in contours:
        M= cv2.moments(contour)
        cx=int(M['m10']/M['m00'])
        cy=int(M['m01']/M['m00'])
        boxcenter.add((cx,cy))
        Center.append((cx,cy))
    box_dist = dist(Center[0],Center[1])
    boxcenter = sorted(boxcenter)
    for point in boxcenter:
        adjacentnode(point)


def getpath(src,end):
    return dijisktra(src,end)
    

if __name__ == '_main_':
    print("graph")

def detect_aruco(img):
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    corners = np.squeeze(corners)
    ids =np.squeeze(ids)
    return corners,ids

def p_orient():
    p.stepSimulation()             
    img = env.camera_feed()
    corners,ids = detect_aruco(img)
    x=0
    y=0
    for cord in corners:
        x=x+cord[0]
        y=y+cord[1]
    x=int(x/4)
    y=int(y/4)
    a=np.array(corners[0])
    b=np.array(corners[3])
    angle = np.arctan2(a[1]-b[1],a[0]-b[0])*(180/np.pi)
    return [x,y],angle

def orient(yaw_req):
    pos,current_yaw = p_orient()
    while abs(yaw_req - current_yaw) > 2.07:
        p.stepSimulation()  
        env.move_husky(9, -9,9, -9)
        pos,current_yaw = p_orient()
        p.stepSimulation()  
        env.move_husky(0, 0,0, 0)
    pos,current_yaw = p_orient()
    p.stepSimulation()  
    env.move_husky(0,0,0, 0)

def distance(a,b):
    x=(a[0]-b[0])*(a[0]-b[0])
    y=(a[1]-b[1])*(a[1]-b[1])
    return np.sqrt(x+y)


def dest(dest_x, dest_y):
    pos,yaw = p_orient()
    yaw_req = np.arctan2(dest_y - pos[1], dest_x - pos[0])*(180/np.pi)
    orient(yaw_req)

    while abs(dest_x - pos[0]) > 0.01 or abs(dest_y - pos[1]) > 0.01:
        p.stepSimulation()  
        env.move_husky(9, 9,9, 9)
        pos,yaw = p_orient()
        if distance(pos,(dest_x,dest_y))<10:
            p.stepSimulation()  
            env.move_husky(0,0,0, 0)
            break
    p.stepSimulation()  
    env.move_husky(0,0,0, 0)


               
env = gym.make("pixelate_arena-v0")   


env.remove_car()
img = env.camera_feed()
time.sleep(0.5)
env.respawn_car()

mainFunction(img)
f=0

j=0
points = [(299, 299),(388, 455)]

for j in range(len(points)):
    pos,yaw = p_orient()
    st=points[j]
    end=points[j+1]
    path = getpath(st,end)
    yaw_req = np.arctan2(end[1] - st[1], end[0] - st[0])*(180/np.pi)
    i=1
    while True:
        p.stepSimulation()  
        env.move_husky(0, 0, 0, 0) 
        if(i<len(path)):
            p.stepSimulation()  
            dest(path[i][0],path[i][1])
        p.stepSimulation()  
        env.move_husky(0, 0, 0, 0)     
        i=i+1
        if(i==len(path)): 
            break

    env.unlock_antidotes()
    time.sleep(3)

    
      

