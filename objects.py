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


class Camera(object):
    def __init__(self,x,y,z):
        self.cx = x
        self.cy = y
        self.cz = z
        self.size = 0.4
        self.pos = vector(self.cx,self.cy,self.cz)
        self.size = 0.2
        self.box = box(pos=self.pos, size=(self.size,self.size,self.size), color=color.brown,axis = (1,0,0))

    
class Robot(object):
    def __init__(self,x,y,z):
        self.x = x
        self.y = y
        self.z = z
        self.pos = vector(self.x,self.y,self.z)
        self.box = create_box(world,100000, 0.5,0.3,0.3, (self.x,self.y,self.z),0)
        self.motors = [Motor(0),Motor(1),Motor(2)]
        self.camera = Camera(self.x,self.y+0.5,self.z)
        self.Bearingtuple = collections.namedtuple('Bearingtuple', 'x y z')
        self.Worldtuple = collections.namedtuple('Worldtuple', 'x y z')
        self.Markertuple = collections.namedtuple('Markertuple', 'distance code marker_type bearing world')
        self.totalmoment=0

    def angle_diff(self,v1x,v1z,v2x,v2z):
        angle=atan2(v2z,v2x)-atan2(v1z,v1x)
        return angle


    def see(self):
        print "photo time"
        newlist = []
        personal_marker_list = []

        #checks faces are visible
        for m in marker_list:
            
            a = m.axis
            b = vector(m.pos.x,self.pos.y,m.pos.z)-self.pos
            if m.axis.y == 0.0:
                if diff_angle(a,b) > 1.6 and diff_angle(a,b)<=pi: #something was wrong with your version of my code.  #if (self.angle_diff(a.x,a.z,b.x,b.z)<=1.6) and (self.angle_diff(a.x,a.z,b.x,b.z) >= -1.6):
                    newlist.append(m)
            
        #print newlist



        #calculates angle to box
        for n in newlist:
            a = vector(n.pos.x,n.pos.y,n.pos.z)-self.pos
            b = self.box.GetFeature('box').pos
            c = -math.degrees(self.angle_diff(a.x,a.z,b.x,b.z))




            distance = round(mag(a),2)
            marker = self.Markertuple(distance,n.code,n.marker_type,self.Bearingtuple(2,c,2),self.Worldtuple(a.z,n.pos.y-self.box.GetFeature('box').pos.y,a.x))
            #pri
            


            #field of view stuff - this works
            if int(marker.bearing.y) <30 and int(marker.bearing.y) >-30:# and marker.distance>0.3+self.box.length/200: #if the robot gets too close it looses sight of the marker
                personal_marker_list.append(marker)

        return personal_marker_list


    
    def update(self):
        #Calculates turning effect of each motor and uses them to make a turn
        averagespeed = float(((self.motors[0].speed) + (self.motors[1].speed))/2)
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
        self.size = 0.2
        self.box = create_box(world, 0.00001, self.size,self.size,self.size, (self.x,self.y,self.z),np.random.uniform(0,6.28))

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
        self.marker = box(pos=self.pos, size=(0.01,self.size,self.size), color=color.white,material=tex,axis = self.axis)



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
            box = Marker(Tokens_per_wallx,xposnew,ypos,zpos-0.01,(0,0,-1),"ARENA")
            marker_list.append(box)
        counter +=1
    
    while counter <=Tokens_per_wallx+Tokens_per_wallz:
        zposnew = zpos - ((counter-Tokens_per_wallx) * spacingz)
        if counter > Tokens_per_wallx:
            box = Marker(Tokens_per_wallx+Tokens_per_wallz,xpos+0.01,ypos,zposnew,(1,0,0),"ARENA")
            marker_list.append(box)
        counter +=1
    
    while counter <=((Tokens_per_wallx*2)+Tokens_per_wallz):
        xposnew = xpos + ((counter-Tokens_per_wallx-Tokens_per_wallz) * spacingz)
        if counter > Tokens_per_wallx+Tokens_per_wallz:
            box = Marker(((Tokens_per_wallx*2)+Tokens_per_wallz),xposnew,ypos,zpos-LENGTH+0.01,(0,0,1),"ARENA")
            marker_list.append(box)
        counter +=1
    
    while counter <=(Tokens_per_wallx+Tokens_per_wallz)*2:
        zposnew = zpos - ((counter-Tokens_per_wallx-Tokens_per_wallz-Tokens_per_wallx) * spacingz)
        if counter > Tokens_per_wallx+Tokens_per_wallz+Tokens_per_wallx:
            box = Marker((Tokens_per_wallx+Tokens_per_wallz)*2,xpos+WIDTH-0.01,ypos,zposnew,(-1,0,0),"ARENA")
            marker_list.append(box)
        counter +=1
    
    
