import adafruit_mpl3115a2
from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
import Adafruit_BBIO.PWM as PWM
import time
import board

from enum import Enum
class Vehicle_Status(Enum):
	ON_PAD = 0
	BOOST = 1
	GLIDE = 2
	APOGEE = 3
	DONE = 4


##########################
## launch day to-do
pad_pressure = 102250
##########################
# 3.44s -- burn phase
# 12.86s -- coast phase
time_bo = 4
time_apo = 13

## constants
g0 = 9.8066
# threshold limits
boost_a_threshold = 4
glide_a_threshold = -0.9
# angle limits
MAX_ANGLE = 120
MIN_ANGLE = 60
# contingency deployment array
cda = [25, 0, 50, 0, 75, 0, 25, 40, 70, 100]

## bus address
# mpl3115a2 default i2c bus
MPL_BUS = 0x60
# imu default i2c bus
LSM_BUS = 0x6a

# servo setup parameter
servoPin = "P8_19" # 
duty_min = 3
duty_max = 14.5

INIT_DEPLOYMENT = 0 # deployment in percentage
ALTI_DEPL_THRESHOLD = 0.5
RATE_ALTI_DEPL = 1/50

def init():
	global altimeter
	global imu
	global duty_span

	# create sensor object, communicating via default I2C bus
	i2c = board.I2C()

	## altimeter setup
	# create and initialize sensor
	altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=MPL_BUS)
	altimeter.sealevel_pressure = pad_pressure

	## IMU setup
	# create and initialize IMU
	imu = LSM6DSOX(i2c, address=LSM_BUS)

	## servo setup
	duty_span = duty_max - duty_min
	# servo might have a different parity
	PWM.start(servoPin, (100 - duty_min), 60.0, 1)	

	status = Vehicle_Status.ON_PAD
	return status


def alti_initialize(init_start_time):

	count_altitude_read = 0
	average_altitude = 0
	while (count_altitude_read == 100)
		# on pad average altitude
		altitude = altimeter.altitude
		if count_altitude_read == 0 or count_altitude_read < 100:
		# (count_altitude_read < 100 and abs(altitude - average_altitude) < 0.8):

			count_altitude_read += 1
			average_altitude = (average_altitude * (count_altitude_read - 1)\
			 + altitude)/count_altitude_read

	print("pad altitude initialization complete - {0:0.3f}".format(average_altitude))
	alti_init_time = time.time() - init_start_time
	return average_altitude


def deploy_percentage_to_angle(percentage):
	return (MAX_ANGLE - MIN_ANGLE) / 100 * percentage + MIN_ANGLE


def main():
	start_time = time.time()
	status = init()

	## altimeter initial reading and altitude setup
	on_PAD_altitude = alti_initialize(time.time())

	previous_altitude = 0
	altitude = 0

	iter_post_apo = 0

	## acceleration setup
	acceleration = 0

	deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

	while (status is not Vehicle_Status.DONE):

		print(status)

		acceleration = imu.acceleration
		z_a = acceleration[2]
		previous_altitude = altitude
		altitude = altimeter.altitude

		if status is Vehicle_Status.ON_PAD:

			if (z_a >= boost_a_threshold * g0 and altitude >= 5):
				liftoff_time = time.time()
				status = Vehicle_Status.BOOST
				continue

			print("on pad idle - idle time: {0:0.3f}".format(time.time() - start_time))
			
			# set deployment
			deployment = deploy_percentage_to_angle(INIT_DEPLOYMENT)

		elif status is Vehicle_Status.BOOST:

			# maybe do something
			## calibrate the fliter maybe
			if (z_a <= glide_a_threshold * g0 or time.time() - liftoff_time >= time_bo):
				boost_time = time.time()
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.GLIDE:
			
			if (altitude < previous_altitude or time.time() -liftoff_time >= time_bo + time_apo):
				PWM.stop(servoPin)
				PWM.cleanup()
				status = Vehicle_Status.APOGEE

		elif status is Vehicle_Status.APOGEE:

			iter_post_apo += 1
			if iter_post_apo // 4 < 10:
				deployment = deploy_percentage_to_angle(cda[iter_post_apo // 4])

			else:
				status = Vehicle_Status.DONE



		angle_f = float(deployment)
		duty = 100 - ((angle_f / 180) * duty_span + duty_min)
		PWM.set_duty_cycle(servoPin, duty)


		print("Altitude: {0:0.3f} meter".format(altitude))
		print("Servo Angle: {0:0.3f} degree".format(deployment))
		print("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (acceleration))
		print("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (imu.gyro))
		print(on_PAD_altitude)
		print()
		time.sleep(0.1)

	# end ...



if __name__ == "__main__":
	main()