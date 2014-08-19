from objects import *
from variables import *
import thread,time
import random
from visual.controls import *

for i in xrange(5):
    token_list.append(Token(i))

populate_walls(7,7)
a=Arena()
R=Robot(0,0.15,0)

def SR_filter(listname,markertype):
    temporary_list=[]
    for l in listname:
        if l.marker_type== markertype:
            temporary_list.append(l)
    listname=temporary_list
    return listname


def usercode0():
    while True:
        markers = R.see()
        markers = SR_filter(markers,'TOKEN')
        print len(markers)
        if len(markers) !=0:
            thing = markers[0]
            if thing.bearing.y < 3 and thing.bearing.y > -3:
                R.motors[0].speed = 50
                R.motors[1].speed = 50
            elif thing.bearing.y < -3:
                R.motors[0].speed = 50
                R.motors[1].speed = -50
            elif thing.bearing.y > 3:
                R.motors[0].speed = -50
                R.motors[1].speed = 50
        else:
            R.motors[0].speed = -50
            R.motors[1].speed = 50

                    
thread.start_new_thread(usercode0,())        
 
while True:
    n = 2
    for i in range(n):
        # Simulation step
        world.step(dt/n)
    R.update()
    m=R.see()
    for token in token_list:
        token.update()
    rate(RATE)