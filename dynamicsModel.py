# Dynamic Model
import numpy as np
import math as m
import IMU_Code
from IMU_Code import *
import time
import dataParser
from dataParser import *
import asyncio


### CONSTANTS
mass = 1.875 #kg

kdx = 0.24      #.18-.24
kdy = 0.24              # TO BE DISCOVERED
kdz = 0.24

thrust = 0.1 #100mm/s #(frame_data[1]-173)/(1810-173)

Ixx = 22036654.9/(10**9)
Iyy = 35990963.19/(10**9)
Izz = 15507494.32/(10**9)
#I = np.array([[Ixx, 0, 0], [0, Iyy, 0],[0, 0, Izz]])

direction = 0
x_dot = 0
y_dot = 0
z_dot = 0





async def get_T(accel_mods,des_state):
	frame_data = await get_frame()

	#Rotation
	phi = get_euler()[0]
	theta = get_euler()[1]
	psi = get_euler()[2]

	angle_pos = np.array([[phi],[theta],[psi]])


	phi_ddot = accel_mods[4]
	theta_ddot = accel_mods[5]
	psi_ddot = accel_mods[3]
	angle_accel = [phi_ddot,theta_ddot,psi_ddot]

"""
	phi_dot = (frame_data[3]-173)/(1810-173)-0.5     #roll
	theta_dot = (frame_data[2]-173)/(1810-173)-0.5    #pitch
	psi_dot = (frame_data[0]-173)/(1810-173)-0.5      #yaw
"""

	angle_vel = [des_state[1],des_state[2],des_state[0]]



	w = [0,0,0]
	r = [[1,0,-m.sin(theta)],[0,m.cos(phi),m.cos(theta)*m.sin(phi)],[0,-m.sin(phi),m.cos(theta)*m.cos(phi)]] #small rotation matrix/
	for x in range(0,3):
		w[x] = angle_vel[0]*r[x][0]+angle_vel[1]*r[x][1]+angle_vel[2]*r[x][2]
#	print("my w")
#	print(w)
	#w = np.multiply(r,angle_accel)
	w_dot = [0,0,0]
	for x in range(0,3):
		w_dot[x] = angle_accel[0]*r[x][0]+angle_accel[1]*r[x][1]+angle_accel[2]*r[x][2]

#	w_dot = np.multiply(r,angle_accel)
#	print("r")
#	print(r)
#	print("w")
#	print(w)
#	print("w_dot")
#	print(w_dot)
#	return [r,w]

#	print(r)
	Tphi = Ixx*w_dot[0] +(Iyy-Izz)*w[1]*w[2]
	Ttheta = Iyy*w_dot[1] +(Izz-Ixx)*w[0]*w[2]
	Tpsi = Izz*w_dot[2] +(Ixx-Iyy)*w[0]*w[1]
#	print("Tphi")
#	print(Tphi)
#	print(I)
#	print(I[0])
#	print(w_dot[0])
#	print(w[1])
#	print(Ttheta)
#	print(Tpsi)
#	print([Tphi,Ttheta,Tpsi])

	#TRANSLATION
#	thrust = 0.1 #100mm/s #(frame_data[1]-173)/(1810-173)


#	kdx = 0.24	#.18-.24
#	kdy = 0.24		# TO BE DISCOVERED
#	kdz = 0.24
#	print("here")
	y_dot = des_state[1]
	z_dot = des_state[2]
	x_dot = des_state[0]

	g = get_gravity()
	Tx = (accel_mods[0]+g[0])*mass-kdx*x_dot
	Ty = (accel_mods[1]+g[1])*mass-kdy*y_dot
	Tz = (accel_mods[2]+g[2])*mass-kdz*z_dot
#	print([Tx,Ty,Tz])
#	print(Tx[0])
#	print("before return")
	T = [Tx[0],Ty[0],Tz[0],Tphi,Ttheta,Tpsi]   #There is some weird formating that the [0] get around
	return T

#x = asyncio.run(get_T())



#prepare_for_get_T()
#task = loop.create_task(get_T())
#remaining_work_not_depends_on_get_T()
#loop.run_until_complete(task)

#print([x,y,z,a,b,c])
#print("end")
