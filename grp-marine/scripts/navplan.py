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
        """Méthode d'affichage de l'objet en string"""
        return "Node at [%.2f,%.2f], cost = %.2f, heur = %.2f"%(self.pos[0],self.pos[1],self.cost,self.heur)

    def setCost(self,oldNode):
        """Accesseur de l'attribut 'cost'."""
        self.cost = oldNode.cost + dist(self.pos,oldNode.pos)
    
    def setHeur(self,goal):
        """Accesseur de l'attribut 'heur'."""
        self.heur = dist(self.pos,goal.pos) + self.cost
    
    def egal(self,node):
        """Méthode pour gérer l'égalité entre les objets de type Node par leur position."""
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
        """Méthode qui renseigne si un point est trop proche d'un obstacle sur la carte.  
        
        La méthode prend en entrée :
        - 'node' : le noeud considéré
        - 'size' : la taille de la zone de danger"""
        for i in range(-size,size+1):
            for j in range(-size,size+1):
                if self.map[node.pos[1]+j][node.pos[0]+i] < 150:
                    return True
        return False

    def updateMap(self,newMap):
        """Accesseur de l'attribut 'map' avec un traitement d'image."""
        map2 = cv2.normalize(newMap,None,0,255,cv2.NORM_MINMAX,dtype=cv2.CV_16UC1)
        map2 = cv2.inRange(map2,np.array([200,200,200]),np.array([255,255,255]))
        map2 = cv2.erode(map2, None, iterations=1)
        map2 = cv2.medianBlur(map2,7)
        self.map = map2
    
    def updateStart(self,newStart):
        """Accesseur de l'attribut 'start'."""
        self.start = Node(newStart,None)

    def updateGoal(self,newGoal):
        """Accesseur de l'attribut 'goal'."""
        self.goal = Node(newGoal,None)
    
    def displayMap(self):
        """Méthode qui exporte la map représentant la réflexion faite par l'algorithme"""
        map3 = copy(self.map)
        for node in self.openlist:
            map3[node.pos[1]][node.pos[0]] = 245
        for node in self.closelist:
            map3[node.pos[1]][node.pos[0]] = 210
        for node in self.navPlan:
            map3[node.pos[1]][node.pos[0]] = 40
        cv2.imwrite("path.png",map3)

    def findLowerNodeIndex(self):
        """Méthode qui trouve l'indice du noeud avec le cout heuristique le plus bas."""
        lower = 0
        for node in range(len(self.openlist)):
            if self.openlist[node].heur < self.openlist[lower].heur :
                lower = node
        return lower

    def addNode(self,newNode,size):
        """Méthode qui permet d'ajouter ou non un noeud aux listes de l'algorithme en prenant en compte l'accessibilité du point sur la carte et les doublons. 

        La méthode prend en entrée :
        - 'newNode' : le noeud que l'on souhaite traiter
        - 'size' : la taille du kernel de la recherche"""
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
        """Méthode qui crée le chemin optimal après la recherche.  

        La méthode prend en entrée :
        - 'lastNode' : le dernier Noeud à partir du quel on remonte jusqu'au début"""
        self.navPlan = [self.goal]
        node = lastNode
        while not node.egal(self.start):
            self.navPlan.append(node)
            node = node.ancestor

    def addNeighbors(self,oldNode,size):
        """Méthode qui lance l'ajout des noeuds voisin.  

        La méthode prend en entrée :
        - 'oldNode' : le noeud originel
        - 'size' : une taille entière pour savoir combien de cases sont parcourues à chaque itération"""
        for i in [-size,0,size]:
            for j in [-size,0,size]:
                node = Node([oldNode.pos[0]+i,oldNode.pos[1]+j],oldNode)
                node.setCost(oldNode)
                node.setHeur(self.goal)
                self.addNode(node,size)

    def aStar(self,size):
        """Algorithme A*, crée le chemin optimal à partir de l'attribut 'map' et le rend dans l'attribut 'navplan'.  
        
        La méthode prend en entrée : 
        - 'size' : une taille entière pour savoir combien de cases sont parcourues à chaque itération"""
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

# # Map finale
# map = cv2.circle(map,my_start,3,(50,255,50),5)
# map = cv2.circle(map,my_goal,3,(50,50,255),5)
# for node in nav.navPlan:
#     map = cv2.circle(map,node.pos,1,(255,50,50),2)
# cv2.imshow("MAP",map)
# cv2.waitKey(0)