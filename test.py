import IMU_Code
import time

print("Za Worudo!")

#eulerData = IMU_Code.eulerData

c =3
while(c<10):
	print(IMU_Code.sensor.quaternion)
	#IMU_Code.set_e_data()
	print(IMU_Code.sensor.euler)
#	print("Euler: {}".format(eulerData))
#	time.sleep(0.25) #only give reading every quarter second
	c = c + 1
print(IMU_Code.sensor.quaternion)
        #IMU_Code.set_e_data()
print(IMU_Code.sensor.euler)
print(IMU_Code.sensor.quaternion)
        #IMU_Code.set_e_data()
print(IMU_Code.sensor.euler)

