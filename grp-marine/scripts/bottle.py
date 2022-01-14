#!/usr/bin/env python3
import math
from turtle import distance, update
import numpy as np
from visualization_msgs.msg import Marker

class Bottle:

    distSameBottle = 0.3
    bottleID = 1

    def __init__(self,pos,color):
        self.color = color
        self.pos = pos
        self.reports = [pos]
        self.id = Bottle.bottleID
        Bottle.bottleID += 1

    def __str__(self):
        return "Bottle "+str(self.id)+" is "+self.color+" and at coordinates "+str(self.pos)

    def sameBottle(self,bottlepos,color):
        distance = math.sqrt(math.pow(self.pos[0]-bottlepos[0],2)+math.pow(self.pos[1]-bottlepos[1],2))
        if (color == self.color) and distance < Bottle.distSameBottle :
            return True
        else:
            return False

    def updatePos(self):
        pos = [0,0,0]
        count = 0
        for report in self.reports:
            count += 1
            pos[0] += report[0]
            pos[1] += report[1]
            pos[2] += report[2]
        pos[0] = pos[0]/count
        pos[1] = pos[1]/count
        pos[2] = pos[2]/count
        self.pos = pos

    def addReportAndUpdate(self,pos):
        if math.dist(self.pos,pos) < Bottle.distSameBottle :
            self.reports.append(pos)
        self.updatePos()