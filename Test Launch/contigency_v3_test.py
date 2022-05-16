import adafruit_mpl3115a2
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import adafruit_bno055
import Adafruit_BBIO.PWM as PWM
import time
import board
import os

from enum import Enum
class Vehicle_Status(Enum):
	ON_PAD = 0
	BOOST = 1
	GLIDE = 2
	APOGEE = 3
	DONE = 4


##################################
## launch day to-do
pad_pressure = 102250
launch_date = "05-07-2022"
##################################
# 3.44s -- burn phase
# 12.86s -- coast phase
time_bo = 4
time_apo = 13
time_end = 120

## constants
g0 = 9.8066
# /mnt/SD/ is the mount point of the SD card
savePath = '/mnt/SD/'
fileName = "DataLog_"+launch_date+".txt"
SD = False
if not SD:
	savePath = '/home/debian'

###################################
# threshold limits
boost_a_threshold = 2
boost_height_threshold = 20
glide_a_threshold = -0.9

# angle limits
MAX_ANGLE = 90
MIN_ANGLE = 0

# altimeter initialization count limit
count_limit = 50

# attitude fail safe limit
fail_angle_limit = 30
allow_angle_limit = 15

# contingency deployment array - test launch 1
# cda = [25, 0, 50, 0, 75, 0, 40, 70, 100, 0]
###################################

###################################
## bus address
# mpl3115a2 default i2c bus
MPL_BUS = 0x60
# imu default i2c bus
#LSM_BUS = 0x6a V3 uses BNO055
LSM_BUS = 0x28
###################################

# servo setup parameter
servoPin = "P8_13" # 
duty_min = 3
duty_max = 14.5

# constants
INIT_DEPLOYMENT = 0 # deployment in percentage
ALTI_DEPL_THRESHOLD = 0.5
RATE_ALTI_DEPL = 1/50


def init():

	global altimeter
	global imu
	global duty_span
	global file
	global i2c

	# create sensor object, communicating via default I2C bus
	i2c = board.I2C()

	## SD card logger setup
	# construct completed file location
	completedName = os.path.join(savePath, fileName)
	# create a new file
	file = open(completedName, "w")
	# write header on the logger
	SDwrite("Launch Date: " + launch_date)

	## altimeter setup
	# create and initialize sensor
	altimeter = altimeter_set(i2c)

	## IMU setup
	# create and initialize IMU
	# imu = LSM6DSOX(i2c, address=LSM_BUS)
	imu = imu_set(i2c)

	## servo setup
	duty_span = duty_max - duty_min
	# servo might have a different parity
	PWM.start(servoPin, (100 - duty_min), 60.0, 1)	

	status = Vehicle_Status.ON_PAD
	return status


def SDwrite(string):

	log_time = time.time() - start_time
	file.write("{0:0.3f}".format(log_time) + ": ")
	file.write(string + "\n")


def alti_initialize(init_start_time):

	count_altitude_read = 0
	average_altitude = 0

	while (count_altitude_read < count_limit):
		# on pad average altitude
		altitude = altimeter.altitude
		if count_altitude_read == 0 or count_altitude_read < count_limit:
		# (count_altitude_read < 100 and abs(altitude - average_altitude) < 0.8):

			count_altitude_read += 1
			average_altitude = (average_altitude * (count_altitude_read - 1)\
			 + altitude)/count_altitude_read

			print(average_altitude)

	SDwrite("pad altitude initialization complete - {0:0.3f}".format(average_altitude))
	print("pad altitude initialization complete - {0:0.3f}".format(average_altitude))
	alti_init_time = time.time() - init_start_time
	return average_altitude


def altimeter_set(i2c):

	global alti_Setup_fail

	try:
		altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=MPL_BUS)
		altimeter.sealevel_pressure = pad_pressure

		alti_Setup_fail = False
		return altimeter

	except Exception as err:
		SDwrite(str(err))
		print(str(err))

		alti_Setup_fail = True
		return None


def imu_set(i2c):

	global imu_Setup_fail

	try:
		imu = adafruit_bno055.BNO055_I2C(i2c, address=LSM_BUS)

		imu_Setup_fail = False
		return imu

	except Exception as err:
		SDwrite(str(err))
		print(str(err))

		imu_Setup_fail = True
		return None


def attitude_safe(euler):

	global atti_fail
	
	pitch = euler[1]
	yaw = euler[2]
	angle_2 = pitch ** 2 + yaw ** 2
	atti_fail = False

	if angle_2 > fail_angle_limit ** 2:
		SDwrite("attitude requirement failed")
		print("attitude requirement failed")

		atti_fail = True

def deploy_percentage_to_angle(percentage):

	return (MAX_ANGLE - MIN_ANGLE) / 100.0 * percentage + MIN_ANGLE


def servo_actuate(deployment):

	angle_f = float(deployment)
	duty = 100 - ((angle_f / 180) * duty_span + duty_min)
	PWM.set_duty_cycle(servoPin, duty)


def output(altitude, deployment, acceleration, gyro, euler_angle):

	if (not alti_Setup_fail and not alti_fail):

		SDwrite("	Al: {0:0.3f}".format(altitude)) # (meters)
		print("Altitude: {0:0.3f} meter".format(altitude))

	SDwrite("	SA: {0:0.2f}".format(deployment)) # (degrees)
	print("Servo Angle: {0:0.2f} degree".format(deployment))

	if (not imu_Setup_fail and not imu_fail):

		SDwrite("	Ac: %.2f %.2f %.2f" % (acceleration)) # X Y Z (m/s^2)
		SDwrite("	Gy: %.2f %.2f %.2f" % (gyro)) # X Y Z (radians/s)
		SDwrite("	EA: %.2f %.2f %.2f" % (euler_angle)) # latter two is pitch and yaw

		print("Acceleration: X: %.2f, Y: %.2f, Z: %.2f m/s^2" % (acceleration))
		print("Gyro: X: %.2f, Y: %.2f, Z: %.2f radians/s" % (gyro))
		print("Euler Angle: row: %.2f, pitch: %.2f, yaw: %.2f degree/s" % (euler_angle))

	print()


# customized based on the mission
def actuation_plan(status):

	global altimeter
	global imu

	deployment = deploy_percentage_to_angle(0)

	if (not imu_fail and not alti_fail and not atti_fail):

		## custmized based on missions
		if status is Vehicle_Status.GLIDE:

			deployment = deploy_percentage_to_angle(100)

		elif status is Vehicle_Status.APOGEE:

			deployment = deploy_percentage_to_angle(0)

	else:
		
		if alti_fail:
			altimeter = altimeter_set(i2c)
			SDwrite("altimeter reset attempt")
			print("altimeter reset attempt")

		if imu_fail:
			imu = imu_set(i2c)
			SDwrite("imu reset attempt")
			print("imu reset attempt")

	return deployment


def main():

	global start_time
	global imu_fail
	global alti_fail
	global altimeter
	global imu

	start_time = time.time()

	try:
		status = init()
	except Exception as err:
		SDwrite(str(err))

	## altimeter initial reading and altitude setup
	on_PAD_altitude = 0
	on_PAD_fail = False
	if not alti_Setup_fail:

		try:
			on_PAD_altitude = alti_initialize(time.time())

		except Exception as err:
			SDwrite(str(err))

			on_PAD_fail = True

	previous_altitude = 0
	altitude = 0
	alti_fail = False

	iter_post_apo = 0

	## imu setup
	acceleration = (0,0,0)
	gyro = (0,0,0)
	z_a = 0
	euler_angle = (0,0,0)
	imu_fail = False

	deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

	while (status is not Vehicle_Status.DONE):

		SDwrite(status.name)
		print(status)

		# ######################################

		# y = input("next status? y/n: ")
		# if y == 'y':
		# 	if status is Vehicle_Status.APOGEE:
		# 		return
		# 	elif status is Vehicle_Status.ON_PAD:
		# 		liftoff_time = time.time()
		# 		status = Vehicle_Status.BOOST
		# 	elif status is Vehicle_Status.BOOST:
		# 		status = Vehicle_Status.GLIDE
		# 	elif status is Vehicle_Status.GLIDE:
		# 		status = Vehicle_Status.APOGEE
				
		# ######################################

		if not imu_Setup_fail:

			try:
				acceleration = imu.acceleration
				z_a = acceleration[2]
				gyro = imu.gyro
				euler_angle = imu.euler

				imu_fail = False

			except Exception as err:
				SDwrite(str(err))
				print("imu read fail")
				print(str(err))

				imu_fail = True

		previous_altitude = altitude

		if not alti_Setup_fail:

			try:
				altitude = altimeter.altitude
				if not on_PAD_fail:
					on_PAD_altitude = altitude
					on_PAD_altitude = False

				alti_fail = False

			except Exception as err:
				SDwrite(str(err))
				print(str(err))

				alti_fail = True

		if (not alti_Setup_fail and not imu_Setup_fail):

			if status is Vehicle_Status.ON_PAD:

				if (z_a >= boost_a_threshold * g0 and altitude >= boost_height_threshold + on_PAD_altitude) or time.time() - start_time >= 60:
					liftoff_time = time.time()
					SDwrite("\n\nLift Off Mark at -- {0:0.3f}\n\n".format(liftoff_time))
					status = Vehicle_Status.BOOST
				
				# reset initial deployment
				deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

			elif status is Vehicle_Status.BOOST:

				if (z_a <= glide_a_threshold * g0 or time.time() - liftoff_time >= time_bo):
					boost_time = time.time()
					status = Vehicle_Status.GLIDE

			elif status is Vehicle_Status.GLIDE:

				if (altitude < previous_altitude or time.time() -liftoff_time >= time_bo + time_apo):
					status = Vehicle_Status.APOGEE

			elif status is Vehicle_Status.APOGEE:

				if (altitude <= 100 + on_PAD_altitude):
					status = Vehicle_Status.DONE

			attitude_safe(euler_angle)
			deployment = actuation_plan(status)
		
		else:

			if (time.time() - start_time >= 120):
				status = Vehicle_Status.DONE

			if alti_fail or alti_Setup_fail:
				altimeter = altimeter_set(i2c)
				SDwrite("altimeter reset attempt")
				print("altimeter reset attempt")

			if imu_fail or imu_Setup_fail:
				imu = imu_set(i2c)
				SDwrite("imu reset attempt")
				print("imu reset attempt")

			deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

		servo_actuate(deployment)
		output(altitude, deployment, acceleration, gyro, euler_angle)

		time.sleep(0.1)

	# end ...
	file.close()



if __name__ == "__main__":
	main()