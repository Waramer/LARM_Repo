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
        """Méthode d'affichage de l'objet en string"""
        return "Bottle %1d is %s and at coordinates [%2.2f;%2.2f;%2.2f]"%(self.id,self.color,self.pos[0],self.pos[1],self.pos[2])

    def sameBottle(self,bottlepos,color):
        """Méthode qui détermine si une bouteille se supperpose à une autre avec une marge d'erreur représentée par 'distSameBottle' et en tenant compte de la courleur.
        
        La méthode prend en entrée :
        - 'bottlepos' : position de la deuxième boutielle
        - 'color' : couleur de la deuxième boutielle"""
        distance = math.sqrt(math.pow(self.pos[0]-bottlepos[0],2)+math.pow(self.pos[1]-bottlepos[1],2))
        if (color == self.color) and distance < Bottle.distSameBottle :
            return True
        else:
            return False

    def updatePos(self):
        """Méthode qui permet de mettre à jour la position de la bouteille en tenant compte de toutes les coordonnées où elle a été détecté pour en donner une moyenne."""
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
        """Méthode qui ajoute une donnée de position à de la bouteille et lance la mise à jour de sa position moyenne.

        La méthode prend en entrée :
        - 'pos' : la position où elle a été vue"""
        if math.dist(self.pos,pos) < Bottle.distSameBottle :
            self.reports.append(pos)
        self.updatePos()