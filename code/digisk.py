from dis import dis
from tabnanny import check
import cv2
import numpy as np
import math
import time

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

def sortwt(node):
    return node[0]

def dijisktra(src,end):
    startToEnd=[]
    dist_from_src={}
    visit={}
    path={}
    weight_node=set()

    for node in graph.keys():
        dist_from_src[node]=inf

    weight_node.add((0,src))  
    dist_from_src[src] = 0

    path[src] = src
        
    while(len(weight_node)!=0):
        weight_node = sorted(weight_node,key=sortwt,reverse=True)
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
                weight_node = sorted(weight_node,key=sortwt,reverse=True)
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