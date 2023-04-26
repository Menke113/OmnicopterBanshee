# Dynamic Model
import numpy as np
import math as m
import IMU_Code
from IMU_Code import *
import time
import dataParser
from dataParser import *
import asyncio
"""
#Constant
Ixx = 22036654.9
Iyy = 35990963.19
Izz = 15507494.32
I = np.array([[Ixx, 0, 0], [0, Iyy, 0],[0, 0, Izz]])


#from IMU
phi = get_euler()[0]
theta = get_euler()[1]
psi = get_euler()[2]

angle_pos = np.array([[phi],[theta],[psi]])


#Calculate
time_old = -1
angular_velocity_old = np.array([[-1000000],[-1000000],[-1000000]])

def get_angular_accel(time_old, angular_velocity_old):
        if time_old == -1:
                time_old = np.transpose(get_gyro())
        if (angular_velocity_old <= -1000000).all():
                angular_velocity_old = np.transpose(get_gyro())

        time_new = time.time()
        angular_velocity_new = np.transpose(get_gyro())
        accel = np.subtract(angular_velocity_new,angular_velocity_old)/(time_new-time_old)

        angular_velocity_old = angular_velocity_new
        time_old = time_new
        phi_ddot = accel[0]
        theta_ddot = accel[1]
        psi_ddot = accel[2]

        angle_accel = np.array([[phi_ddot],[theta_ddot],[psi_ddot]])

        return [angle_accel, time_old, angular_velocity_old]

[a,t,o] = get_angular_accel(time_old, angular_velocity_old)
[a,t,o] = get_angular_accel(t, o)

print(a)
print(t)

phi_ddot = a[0]
theta_ddot = a[1]
psi_ddot = a[2]

angle_accel = a



#I #moments of inertia A #angular acceleration T #Translational

#from the transmiter

async def get_angle_vel():

        frame_data = await get_frame()
        phi_dot = frame_data[3]     #roll
        theta_dot = frame_data[2]   #pitch
        psi_dot = frame_data[0]     #yaw

        angle_vel = np.array([[phi_dot],[theta_dot],[psi_dot]])
        return angle_vel




r = np.array([[1,0,-m.sin(theta)],[0,m.cos(phi),m.cos(theta)*m.sin(phi)],[0,-m.sin(phi),m.cos(theta)*m.cos(phi)]]) #small rotation matrix
w = np.multiply(r,asyncio.run(get_angle_vel()))
w_dot = np.multiply(r,angle_accel)


#receiver input for angular velocities
Tphi = I[0]*w_dot[0] +(I[1]-I[2])*w[1]*w[2]
Ttheta = I[1]*w_dot[1] +(I[2]-I[0])*w[0]*w[2]
Tpsi = I[2]*w_dot[2] +(I[0]-I[1])*w[0]*w[1]



### TRANSLATION

trans_accel = get_acceleration()
mass = 1.875 #kg

kdx = 0.24      #.18-.24
kdy = 0.24              # TO BE DISCOVERED
kdz = 0.24

direction = 0
x_dot = 0
y_dot = 0
z_dot = 0


async def get_vel():

        frame_data = await get_frame()
        phi_dot = frame_data[3]     #roll
        theta_dot = frame_data[2]   #pitch
        psi_dot = frame_data[0]     #yaw

        vel = np.array([[phi_dot],[theta_dot],[psi_dot]])
        return vel



Tx = trans_accel[0]*mass-kdx*x_dot
Ty = trans_accel[1]*mass-kdy*y_dot
Tz = trans_accel[2]*mass-kdz*z_dot

"""
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


time_old = -1
angular_velocity_old = np.array([[-1000000],[-1000000],[-1000000]])


async def get_T():
	frame_data = await get_frame()
#	print(frame_data)
#	time.sleep(0.1)
	#Rotation
	phi = get_euler()[0]
	theta = get_euler()[1]
	psi = get_euler()[2]

	angle_pos = np.array([[phi],[theta],[psi]])

	time_old = -1
	angular_velocity_old = np.array([[-1000000],[-1000000],[-1000000]])

	def get_angular_accel(time_old, angular_velocity_old):
		if time_old == -1:
			time_old = np.transpose(get_gyro())
		if (angular_velocity_old <= -1000000).all():
			angular_velocity_old = np.transpose(get_gyro())

		time_new = time.time()
		angular_velocity_new = np.transpose(get_gyro())
		accel = np.subtract(angular_velocity_new,angular_velocity_old)/(time_new-time_old)

		angular_velocity_old = angular_velocity_new
		time_old = time_new
		phi_ddot = accel[0]
		theta_ddot = accel[1]
		psi_ddot = accel[2]

		angle_accel = [phi_ddot,theta_ddot,psi_ddot]

		return [angle_accel, time_old, angular_velocity_old]

	[a,t,o] = get_angular_accel(time_old, angular_velocity_old)
	[a,t,o] = get_angular_accel(t, o)

	phi_ddot = a[0]
	theta_ddot = a[1]
	psi_ddot = a[2]

	angle_accel = a
#	print("angle accel")
#	print(a)

#	while 1:
#		[a,t,o] = get_angular_accel(t, o)
#		time.sleep(1)
#		print(a)
	#[993, 997, 991]


	phi_dot = (frame_data[3]-173)/(1810-173)-0.5     #roll
	theta_dot = (frame_data[2]-173)/(1810-173)-0.5    #pitch
	psi_dot = (frame_data[0]-173)/(1810-173)-0.5      #yaw

	angle_vel = [phi_dot,theta_dot,psi_dot]
#	print("angleVel")
#	print(angle_vel)




	w = [0,0,0]
	r = [[1,0,-m.sin(theta)],[0,m.cos(phi),m.cos(theta)*m.sin(phi)],[0,-m.sin(phi),m.cos(theta)*m.cos(phi)]] #small rotation matrix/
	for x in range(0,3):
		w[x] = angle_vel[0]*r[x][0]+angle_vel[1]*r[x][1]+angle_vel[2]*r[x][2]
#	print("my w")
#	print(w)
	#w = np.multiply(r,angle_vel)
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
	# 5 is direction
	# 6 is which axis
	trans_accel = get_acceleration()
#	thrust = 0.1 #100mm/s #(frame_data[1]-173)/(1810-173)


	if frame_data[5] < 250:
		direction = 1
#		print("high")
	elif frame_data[5] > 1800:
		direction = -1
#		print("low")
	else:
		direction = 0
#		print("hover")


	if frame_data[6] < 250:
		# z axis
		z_dot = direction * thrust
#		print("z")
		x_dot = 0
		y_dot = 0
	elif frame_data[6] > 1800:
		# y axis
		y_dot = direction * thrust
#		print("y")
		x_dot = 0
		z_dot = 0
	else:
		# x axis
		x_dot = direction * thrust
#		print("x")
		y_dot = 0
		z_dot = 0

#	kdx = 0.24	#.18-.24
#	kdy = 0.24		# TO BE DISCOVERED
#	kdz = 0.24
#	print("here")
#	y_dot = 0
#	z_dot = 0
#	x_dot = 0
	Tx = trans_accel[0]*mass-kdx*x_dot
	Ty = trans_accel[1]*mass-kdy*y_dot
	Tz = trans_accel[2]*mass-kdz*z_dot
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
"""
max1 = 0
max2 = 0
max3 = 0
min1 = 0
min2 = 0
min3 = 0

while 1:
	time.sleep(0.1)
	x = asyncio.run(get_T())
	print(x)
	if x[3] > max1:
		max1 = x[3]
#	print(max1)
#	print(min1)
	if x[4] > max2:
		max2 = x[4]
#	print(max2)
#	print(min2)
	if x[5] > max3:
		max3 = x[5]
#	print(max3)
#	print(min3)
	if x[3] < min1:
		min1 = x[3]
	if x[4] < min2:
		min2 = x[4]
	if x[5] < min3:
		min3 = x[5]
"""

#print(z)
#print(a)
#print(b)
#print(c)
#print([x,y,z,a,b,c])
print("end")
