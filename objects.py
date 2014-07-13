#New version of objects implementing vpyode

import numpy as np
import ode,odelib,vpyode
from visual import *
import Image
import random
from Texturesandcolours import *
import collections
from variables import *


def create_box(world, density, lx, ly, lz,position,rotation):
    """Create a box body and its corresponding geom."""
    # Create body
    body = vpyode.GDMFrameBody(world)
    element = vpyode.GDMElement()
    element.DefineBox(density, lx, ly, lz)
    body.AddGDMElement('Box', element)
    body.setPosition(position)
    body.setQuaternion((rotation,0,1,0))
    return body
    
class Arena(object):
    def __init__(self):
        # Create and draw arena and associated geoms
        self.floor = ode.GeomPlane(vpyode._bigSpace, (0,1,0), 0)
        self.arenafloor = box(pos=(0,0,0), size=(0.01,4,4), color=color.orange, material = tex2, axis=(0,1,0))
        self.wall1 = ode.GeomPlane(vpyode._bigSpace, (-1,0,0), -2)
        self.wall2 = ode.GeomPlane(vpyode._bigSpace, (1,0,0), -2)
        self.wall3 = ode.GeomPlane(vpyode._bigSpace, (0,0,-1), -2)
        self.wall4 = ode.GeomPlane(vpyode._bigSpace, (0,0,1), -2)
        self.wall1vis = box(pos=(-2,0,0), size=(0.01,1,4), color=color.orange)
        self.wall2vis = box(pos=(2,0,0), size=(0.01,1,4), color=color.orange)
        self.wall3vis = box(pos=(0,0,-2), size=(4,1,0.01), color=color.orange)
        self.wall4vis = box(pos=(0,0,2), size=(4,1,0.01), color=color.orange)

class Motor(object):
    def __init__(self, which_motor, speed = 0.0):
        self._speed = speed;
        self._motor_no = which_motor
        
    @property
    def speed(self):
        return self._speed
        
    @speed.setter
    def speed(self, value):
        global speed
        self._speed = value

    @speed.deleter
    def speed(self):
        del self._speed
    
class Robot(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.pos = vector(self.x,self.y,self.z)
        self.box = create_box(world,1000, 0.5,0.3,0.3, (self.x,self.y,self.z),0)
        self.motors = [Motor(0),Motor(1),Motor(2)]
        self.Bearingtuple = collections.namedtuple('Bearingtuple', 'x y z')
        self.Worldtuple = collections.namedtuple('Worldtuple', 'x y z')
        self.Markertuple = collections.namedtuple('Markertuple', 'distance code marker_type bearing world')
        self.totalmoment=0
    
    def update(self):
        #Calculates turning effect of each motor and uses them to make a turn
        averagespeed = float((self.motors[0].speed + self.motors[1].speed))/2
        moment0 = float(self.motors[0].speed)
        moment1 = float(-self.motors[1].speed)
        self.totalmoment = 3*(moment0 + moment1)
        self.box.setAngularVel((0,self.totalmoment,0))
        vel = odelib.rotateVector(self.box.getRotation(),(averagespeed,0,0))
        self.box.setLinearVel((vel[0],vel[1],vel[2]))
        self.box.UpdateDisplay()

class Token(object):
    def __init__(self):
        self.x = np.random.uniform((-WIDTH/2)+0.6,WIDTH/2-0.60)
        self.z = np.random.uniform((-LENGTH/2)+0.6,LENGTH/2-0.60)
        self.y = 0.07
        self.pos = vector(self.x,0.07,self.z)
        self.size = 0.1
        self.box = create_box(world, 1000, self.size,self.size,self.size, (self.x,self.y,self.z),np.random.uniform(0,6.28))

    def update(self):
        self.box.UpdateDisplay()
