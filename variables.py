from visual import *
import ode,odelib,vpyode
RATE = 50
dt = 1.0/RATE
marker_list = []
token_list = []
robot_list = []


LENGTH = 4
WIDTH = 4
HEIGHT = 0.3
NUMBER_OF_TOKENS = 10





SWARM_MODE = False
SWARM_NUMBER = 0
world = vpyode.World()
world.setGravity((0,-9.81,0))
world.setERP(0.8)
world.setCFM(1E-5)
world._getScene().autoscale = False
world._getScene().center = (0,2.5,0)
world._getScene().range = 4
world._getScene().background = (0,0,0)


