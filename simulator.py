from objects import *
from variables import *
import thread,time

a=Arena()
R=Robot(0,0.15,0)

for i in xrange(NUMBER_OF_TOKENS):
    token_list.append(Token())

def usercode0():
    while True:
        R.motors[0].speed=0.8
        R.motors[1].speed=0.8
        time.sleep(2)
        R.motors[0].speed=0.5
        R.motors[1].speed=-0.5
        time.sleep(0.5)
        R.motors[0].speed=-0.3
        R.motors[1].speed=0.6
        time.sleep(1)
        R.motors[0].speed=-0.7
        R.motors[1].speed=-0.5
        time.sleep(2)
        
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