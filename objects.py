#New version of objects implementing vpyode

import numpy as np
import ode,odelib,vpyode
from visual import *
import Image
import random
from Texturesandcolours import *
import collections
from variables import *
import time
import math


def create_box(world, density, lx, ly, lz,position,rotation, colour):
    """Create a box body and its corresponding geom."""
    # Create body
    body = vpyode.GDMFrameBody(world)
    element = vpyode.GDMElement()
    element.DefineBox(density, lx, ly, lz, colour)
    body.AddGDMElement('box', element)
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
    def __init__(self, which_motor, speed = 0.00):
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
        self.box = create_box(world,80, 0.5,0.3,0.3, (self.x,self.y,self.z),0, color.blue)
        self.makeCamera()
        self.motors = [Motor(0),Motor(1),Motor(2)]
        self.Bearingtuple = collections.namedtuple('Bearingtuple', 'x y z')
        self.Worldtuple = collections.namedtuple('Worldtuple', 'x y z')
        self.Markertuple = collections.namedtuple('Markertuple', 'distance code marker_type bearing world')
        self.totalmoment=0
    
    #Creates a box to act as camera
    def makeCamera(self):
        camera = vpyode.GDMElement()
        camera.DefineBox(1000,0.15,0.1,0.1,color.red,self.box.GetFeature("box").pos + (0.15,0.05,0))
        self.box.AddGDMElement("camera",camera)
    
    def roty_between(self, v1, v2):
        v1 = norm(v1)
        v2 = norm(v2)
        angle = atan2(v1.x,v1.z) - atan2(v2.x,v2.z)
        return angle

    def rotz_between(self, v1, v2):
        v1 = norm(v1)
        v2 = norm(v2)
        angle = atan2(v1.x,v1.y) - atan2(v2.x,v2.y)
        return angle
        
    def see(self):
        newlist = []
        personal_marker_list = []
        facelist = ["0","1","2","3","4","5"]
        robot_position = vector(self.box.getPosition())
        rel_robot_axis = norm(self.box.GetFeature("box").axis)
        world_robot_axis = odelib.rotateVector(self.box.getRotation(),rel_robot_axis)
        #check arena markers
        for m in arena_marker_list:
            robot_to_marker = m.pos - robot_position
            rot_y = self.roty_between(world_robot_axis,robot_to_marker)
            rot_z = self.rotz_between(world_robot_axis,robot_to_marker)
            if rot_y < 0.53 and rot_y > -0.53:
                if rot_z < 0.53 and rot_z > -0.53:
                    distance = mag(robot_to_marker)
                    marker = self.Markertuple(distance,m.code,m.marker_type,self.Bearingtuple(0,np.degrees(rot_y),np.degrees(rot_z)),self.Worldtuple(robot_to_marker.x,robot_to_marker.y,robot_to_marker.z))
                    personal_marker_list.append(marker)
                    
        #check token markers
        for t in token_list:
            for f in facelist:
                face = t.markers[int(f)]
                faceaxis = norm(odelib.rotateVector(t.box.getRotation(),face.axis))
                faceposition = vector(t.box.getPosition()) + faceaxis*0.05 #Replace 0.05 with marker size/2
                robot_to_marker = faceposition - robot_position
                rot_y = self.roty_between(world_robot_axis,robot_to_marker)
                rot_z = self.rotz_between(world_robot_axis,robot_to_marker)
                if diff_angle(faceaxis, world_robot_axis) < pi/2:
                    if rot_y < 0.53 and rot_y > -0.53:
                        if rot_z < 0.53 and rot_z > -0.53:
                            distance = mag(robot_to_marker)
                            marker = self.Markertuple(distance,face.code,face.marker_type,self.Bearingtuple(0,np.degrees(rot_y),np.degrees(rot_z)),self.Worldtuple(robot_to_marker.x,robot_to_marker.y,robot_to_marker.z))
                            personal_marker_list.append(marker)
        return personal_marker_list

    def update(self):
        #Calculates turning effect of each motor and uses them to make a turn
        averagespeed = float((self.motors[0].speed + self.motors[1].speed)/2)/100
        moment0 = float(self.motors[0].speed)/100
        moment1 = float(-self.motors[1].speed)/100
        self.totalmoment = 2*(moment0 + moment1)
        self.box.setAngularVel((0,self.totalmoment,0))
        vel = odelib.rotateVector(self.box.getRotation(),(averagespeed,0,0))
        self.box.setLinearVel((vel[0],vel[1],vel[2]))
        self.box.UpdateDisplay()


                

'''
Token class describes a "Token" (cardboard box) with markers attached on all faces
'''
class Token(object):
    def __init__(self,code):
        self.x = np.random.uniform(-((LENGTH/2)-0.06),((LENGTH/2)-0.06))
        self.z = np.random.uniform(-((WIDTH/2)-0.06),((WIDTH/2)-0.06))
        self.y = 0.052
        self.pos = vector(self.x,self.y,self.z)
        self.size = 0.1
        self.box = create_box(world, 100, self.size,self.size,self.size, (self.x,self.y,self.z),0, color.brown)
        self.markers = [Marker(code,self.box.GetFeature('box').pos.x-0.05,self.box.GetFeature('box').pos.y,self.box.GetFeature('box').pos.z,(-1,0,0),"TOKEN"),
                        Marker(code,self.box.GetFeature('box').pos.x+0.05,self.box.GetFeature('box').pos.y,self.box.GetFeature('box').pos.z,(1,0,0),"TOKEN"),
                        Marker(code,self.box.GetFeature('box').pos.x,self.box.GetFeature('box').pos.y,self.box.GetFeature('box').pos.z-0.05,(0,0,-1),"TOKEN"),
                        Marker(code,self.box.GetFeature('box').pos.x,self.box.GetFeature('box').pos.y,self.box.GetFeature('box').pos.z+0.05,(0,0,1),"TOKEN"),
                        Marker(code,self.box.GetFeature('box').pos.x,self.box.GetFeature('box').pos.y-0.05,self.box.GetFeature('box').pos.z,(0,-1,0),"TOKEN"),
                        Marker(code,self.box.GetFeature('box').pos.x,self.box.GetFeature('box').pos.y+0.05,self.box.GetFeature('box').pos.z,(0,1,0),"TOKEN")]
        self.box.AddDisplayObject("0",self.markers[0].marker)
        self.box.AddDisplayObject("1",self.markers[1].marker)
        self.box.AddDisplayObject("2",self.markers[2].marker)
        self.box.AddDisplayObject("3",self.markers[3].marker)
        self.box.AddDisplayObject("4",self.markers[4].marker)
        self.box.AddDisplayObject("5",self.markers[5].marker)
    def update(self):
        self.box.UpdateDisplay()

class Marker(object):
    def __init__(self,code,x,y,z,axis_decider,marker_type):
        self.x = x
        self.y = y
        self.z = z
        self.pos = vector(self.x,self.y,self.z)
        self.axis = vector(int(axis_decider[0]),int(axis_decider[1]),int(axis_decider[2]))
        self.marker_type = marker_type
        if self.marker_type == "TOKEN":
            self.size = 0.09
        elif self.marker_type == "ARENA":
            self.size = 0.4
        self.marker = box(pos=self.pos, size=(0.001,self.size,self.size), color=color.white,material=tex,axis = self.axis)
        self.angle = 0
        self.angle_rad = math.radians(self.angle)
        self.code = code


def populate_walls(Tokens_per_wallx,Tokens_per_wallz):
    spacingx = float(WIDTH)/(Tokens_per_wallx+1)
    spacingz = float(LENGTH)/(Tokens_per_wallz+1)
    #xwall1
    counter = 0
    xpos = -WIDTH/2
    ypos = HEIGHT*0.75
    zpos = LENGTH/2

    while counter <=Tokens_per_wallx:
        xposnew = xpos + (counter * spacingx)
        if counter > 0:
            box = Marker(counter,xposnew,ypos,zpos-0.01,(0,0,-1),"ARENA")
            arena_marker_list.append(box)
        counter +=1
        
    while counter <=Tokens_per_wallx+Tokens_per_wallz:
        zposnew = zpos - ((counter-Tokens_per_wallx) * spacingz)
        if counter > Tokens_per_wallx:
            box = Marker(counter,xpos+0.01,ypos,zposnew,(1,0,0),"ARENA")
            arena_marker_list.append(box)
        counter +=1
    
    while counter <=((Tokens_per_wallx*2)+Tokens_per_wallz):
        xposnew = xpos + ((counter-Tokens_per_wallx-Tokens_per_wallz) * spacingz)
        if counter > Tokens_per_wallx+Tokens_per_wallz:
            box = Marker(counter,xposnew,ypos,zpos-LENGTH+0.01,(0,0,1),"ARENA")
            arena_marker_list.append(box)
        counter +=1
    
    while counter <=(Tokens_per_wallx+Tokens_per_wallz)*2:
        zposnew = zpos - ((counter-Tokens_per_wallx-Tokens_per_wallz-Tokens_per_wallx) * spacingz)
        if counter > Tokens_per_wallx+Tokens_per_wallz+Tokens_per_wallx:
            box = Marker(counter,xpos+WIDTH-0.01,ypos,zposnew,(-1,0,0),"ARENA")
            arena_marker_list.append(box)
        counter +=1    
    
