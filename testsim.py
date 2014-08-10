from objects import *
from variables import *
import thread,time
import random
from visual.controls import *

for i in xrange(1):
    token_list.append(Token(i))

populate_walls(7,7)
a=Arena()
R=Robot(0,0.15,0)


def usercode0():
    while True:
        m = R.see()
        for thing in m:
            print thing.marker_type
            if thing.marker_type == 'TOKEN':
                if thing.bearing.y < 3 and thing.bearing.y > -3:
                    R.motors[0].speed = 50
                    R.motors[1].speed = 50
                if thing.bearing.y < -3:
                    R.motors[0].speed = 50
                    R.motors[1].speed = -50
                if thing.bearing.y > 3:
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