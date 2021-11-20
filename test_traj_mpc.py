import numpy as np
from controller import Controller
from quad_model import Model
import matplotlib.pyplot as plt
from mpc_class import MPC_controller
from traj_planner import Traj_Planner



quad=Model()
quad2=Model()
controller=Controller()
mpc_controller=MPC_controller()

def to_dataset(x,u):
    x_data=np.zeros(13)
    for i in range(9):
        x_data[i]=x[i]
    for i in range(4):
        x_data[i+9]=u[i]
    return x_data

def conf_u(u):
    for i in range(3):
        u[i]=(u[i]-5)/10

    return u
def pose_err(x,xt):
    


traj=Traj_Planner()


Gate=np.array([[0,10,-2,0*np.pi/180],[5,8.66,-2,-30*np.pi/180],[8.66,5,-2,-60*np.pi/180],[10,0,-2,-90*np.pi/180],
               [8.66,-5,-2,-120*np.pi/180],[5,-8.66,-2,-150*np.pi/180],[0,-10,-2,-180*np.pi/180],
               [-5,-8.66,-2,-210*np.pi/180],[-8.66,-5,-2,-240*np.pi/180],[-10,0,-2,-270*np.pi/180],
               [-8.66,5,-2,-300*np.pi/180],[-5,8.66,-2,-330*np.pi/180],[0,10,-2,-360*np.pi/180]])

initial_pos=[-5,10,-2,0]
initial_vel=[0,0,0,0]

foward_vel=5

current_pose=initial_pos
current_vel=initial_vel

for i in range(len(Gate[:,0])):
    Target_pose=Gate[i,:]
    

