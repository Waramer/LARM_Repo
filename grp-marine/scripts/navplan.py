from copy import copy
import numpy as np
import math
import cv2

def dist(p1,p2):
    vec = [p1[0]-p2[0],p1[1]-p2[1]]
    dist = math.dist([0,0],vec)
    return dist

class Node:
    def __init__(self,pos,anc):
        self.pos = pos
        self.cost = 0
        self.heur = 0
        self.ancestor = anc
    
    def __str__(self):
        return "Node at [%.2f,%.2f], cost = %.2f, heur = %.2f"%(self.pos[0],self.pos[1],self.cost,self.heur)

    def setCost(self,oldNode):
        self.cost = oldNode.cost + dist(self.pos,oldNode.pos)
    
    def setHeur(self,goal):
        self.heur = dist(self.pos,goal.pos) + self.cost
    
    def egal(self,node):
        if self.pos[0]==node.pos[0] and self.pos[1]==node.pos[1]:
            return True
        else:
            return False


class NavigationPlanner:
    def __init__(self):
        self.map = []
        self.goal = Node([0,0],None)
        self.start = Node([0,0],None)
        self.navPlan = []
        self.openlist = []
        self.closelist = []

    def nodeCloseToObstacle(self,node,size):
        for i in range(-size,size+1):
            for j in range(-size,size+1):
                if self.map[node.pos[1]+j][node.pos[0]+i] < 150:
                    return True
        return False

    def printLists(self):
        print("CLOSELIST :")
        for node in self.closelist:
            print(str(node))
        print("OPENLIST :")
        for node in self.openlist:
            print(str(node))

    def updateMap(self,newMap):
        map2 = cv2.normalize(newMap,None,0,255,cv2.NORM_MINMAX,dtype=cv2.CV_16UC1)
        map2 = cv2.inRange(map2,np.array([200,200,200]),np.array([255,255,255]))
        map2 = cv2.erode(map2, None, iterations=1)
        map2 = cv2.medianBlur(map2,7)
        self.map = map2
    
    def updateStart(self,newStart):
        self.start = Node(newStart,None)

    def updateGoal(self,newGoal):
        self.goal = Node(newGoal,None)
    
    def displayMap(self):
        map3 = copy(self.map)
        for node in self.openlist:
            map3[node.pos[1]][node.pos[0]] = 245
        for node in self.closelist:
            map3[node.pos[1]][node.pos[0]] = 210
        for node in self.navPlan:
            map3[node.pos[1]][node.pos[0]] = 40
        print("map exported")
        cv2.imwrite("path.png",map3)

    def findLowerNodeIndex(self):
        lower = 0
        for node in range(len(self.openlist)):
            if self.openlist[node].heur < self.openlist[lower].heur :
                lower = node
        return lower

    def addNode(self,newNode,size):
        if self.nodeCloseToObstacle(newNode,size):
            return True
        for node in self.closelist:
            if newNode.egal(node) :
                if newNode.cost < node.cost :
                    self.closelist.remove(node)
                    self.closelist.append(newNode)
                return True
        for node in self.openlist:
            if newNode.egal(node) :
                if newNode.cost < node.cost :
                    self.openlist.remove(node)
                    self.openlist.append(newNode)
                return True
        self.openlist.append(newNode)
        return False

    def planNavigation(self,lastNode):
        self.navPlan = [self.goal]
        node = lastNode
        while not node.egal(self.start):
            self.navPlan.append(node)
            node = node.ancestor

    def addNeighbors(self,oldNode,size):
        for i in [-size,0,size]:
            for j in [-size,0,size]:
                node = Node([oldNode.pos[0]+i,oldNode.pos[1]+j],oldNode)
                node.setCost(oldNode)
                node.setHeur(self.goal)
                self.addNode(node,size)

    def aStar(self,size):
        self.start.setHeur(self.goal)
        self.openlist = [self.start]
        self.closelist = []
        self.navPlan = []
        self.displayMap()
        while len(self.openlist)!=0:
            lowerNode = self.openlist[self.findLowerNodeIndex()]
            self.openlist.remove(lowerNode)
            self.closelist.append(lowerNode)
            if dist(lowerNode.pos,self.goal.pos)<size :
                self.planNavigation(lowerNode)
                return self.navPlan
            self.addNeighbors(lowerNode,size)
        return False


# # Initialise
# my_start = [130,90]
# my_goal = [325,200]
# map = cv2.imread("map.png")
# height,width=np.shape(map)[0],np.shape(map)[1]

# # Map image treatment
# map2 = cv2.inRange(map,np.array([200,200,200]),np.array([255,255,255]))
# map2 = cv2.erode(map2, None, iterations=1)
# map2 = cv2.medianBlur(map2,7)

# # Use NavigationPlanner
# nav = NavigationPlanner()
# nav.goal = Node(my_goal,None)
# nav.start = Node(my_start,None)
# nav.map = map2
# path = nav.aStar(4)
# nav.displayMap()