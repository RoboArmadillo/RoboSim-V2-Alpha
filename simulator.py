from objects import *
from variables import *
import thread,time
import random
'''
#################
Usercode Function
#################
'''    

def SR_filter(listname,markertype):
    temporary_list=[]
    for l in listname:
        if l.marker_type== markertype:
            temporary_list.append(l)
    listname=temporary_list
    return listname


def distance_orderer(listname):
    listname=sorted(listname, key=lambda Marker:Marker.distance)
    return listname


def swarmcode(number):
    while True:
        robot_list[number].motors[0].speed = -50.0
        robot_list[number].motors[1].speed = 50.0



def usercode0():
    time.sleep(10) #This delay is to let the tokens settle down - need to fix the spawning
    while True:
        markers = R.see()
        markers = SR_filter(markers, 'TOKEN')

            
        if len(markers)>0:
            angle = markers[0].bearing.y
            print markers[0].marker_type
            print angle
            if angle >10 and angle <30:
                R.motors[0].speed = -40
                R.motors[1].speed = 40
            elif angle < -10 and angle > -30:
                R.motors[0].speed = 40
                R.motors[1].speed = -40
            elif angle <10 and angle >-10:
                R.motors[0].speed = 50
                R.motors[1].speed = 50
        else:
            R.motors[0].speed = -100
            R.motors[1].speed = 100

def usercode1():
    while True:
        markers = R.see()
        for m in markers:
            if m.marker_type != "token marker":
                markers.remove(m)

            
        if len(markers)>0:
            angle = markers[0].bearing.y
            if angle >10 and angle <30:
                R.motors[0].speed = -10
                R.motors[1].speed = 10
            elif angle < -10 and angle > -30:
                R.motors[0].speed = 20
                R.motors[1].speed = -20
            elif angle <10 and angle >-10:
                R.motors[0].speed = 30
                R.motors[1].speed = 30
        else:
            R.motors[0].speed = -10
            R.motors[1].speed = 10
            time.sleep(0.2)

def usercode2():
    while True:
        markers = R.see()
        for m in markers:
            if m.marker_type != "token marker":
                markers.remove(m)

            
        if len(markers)>0:
            angle = markers[0].bearing.y
            if angle >10 and angle <30:
                R.motors[0].speed = -10
                R.motors[1].speed = 10
            elif angle < -10 and angle > -30:
                R.motors[0].speed = 20
                R.motors[1].speed = -20
            elif angle <10 and angle >-10:
                R.motors[0].speed = 30
                R.motors[1].speed = 30
        else:
            R.motors[0].speed = -10
            R.motors[1].speed = 10
            time.sleep(0.2)

def usercode3():
    while True:
        markers = R.see()
        for m in markers:
            if m.marker_type != "token marker":
                markers.remove(m)

            
        if len(markers)>0:
            angle = markers[0].bearing.y
            if angle >10 and angle <30:
                R.motors[0].speed = -10
                R.motors[1].speed = 10
            elif angle < -10 and angle > -30:
                R.motors[0].speed = 20
                R.motors[1].speed = -20
            elif angle <10 and angle >-10:
                R.motors[0].speed = 30
                R.motors[1].speed = 30
        else:
            R.motors[0].speed = -10
            R.motors[1].speed = 10
            time.sleep(0.2)


a=Arena()
populate_walls(7,7)
for i in xrange(40,NUMBER_OF_TOKENS+40):
    token_list.append(Token(i))
R=Robot(0,0.15,0)
thread.start_new_thread(usercode0,())

while True:
    n = 2
    for i in range(n):
        # Simulation step
        world.step(dt/n)
    R.update() 
    for token in token_list:
        token.update()
    rate(RATE)
