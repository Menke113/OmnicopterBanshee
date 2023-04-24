
import IMU_Code
from IMU_Code import *
import numpy as np
import time
t_start = time.time()
time_old = -1
angular_velocity_old = np.array([[-1000000],[-1000000],[-1000000]])





def get_angular_accel1(time_old, angular_velocity_old):
        if time_old == -1:
                time_old = np.transpose(get_gyro())
        if (angular_velocity_old <= -1000000).all():
                angular_velocity_old = np.transpose(get_gyro())

        time_new = time.time()
        angular_velocity_new = np.transpose(get_gyro())
        accel = np.subtract(angular_velocity_new,angular_velocity_old)/(time_new-time_old)

        angular_velocity_old = angular_velocity_new
        time_old = time_new
        return [accel, time_old, angular_velocity_old]

#[a,t,o] = get_angular_accel(time_old, angular_velocity_old)
#print(a)
#print(t)
#time.sleep(.2)
#count = 0
#while count<10:
 #       [a,t,o] = get_angular_accel(t,o)
  #      time.sleep(.01)
   #     print(a)
    #    #print(t)
     #   count += 1
#time.sleep(.2)
#[a,t,o] = get_angular_accel(t,o)

#print(a)
#print(t)

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

[a,t,o] = get_angular_accel(time_old, angular_velocity_old)
print(a)
print(t)
#time.sleep(.2)
count = 0
while count<100:
        [a,t,o] = get_angular_accel(t,o)
        time.sleep(.01)
        print(a)
        #print(t)
        count += 1
#time.sleep(.2)
[a,t,o] = get_angular_accel(t,o)

print(a)
print(t)

