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


##########################
## launch day to-do
pad_pressure = 102250
##########################
# 3.44s -- burn phase
# 12.86s -- coast phase

## constants
g0 = 9.8066

## bus address
# mpl3115a2 default i2c bus
MPL_BUS = 0x60
# imu default i2c bus
LSM_BUS = 0x6a

# servo setup parameter
servoPin = "P8_19" # 
duty_min = 3
duty_max = 14.5

INIT_DEPLOYMENT = 60
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


def main():
	status = init()

	## altimeter initial reading and altitude setup
	on_PAD_altitude = altimeter.altitude
	average_altitude = 0
	count_altitude_read = 0
	start_altitude = 0
	previous_altitude = 0
	altitude = 0

	## acceleration setup
	acceleration = 0

	deployment = INIT_DEPLOYMENT

	while (status is not Vehicle_Status.APOGEE):

		print(status)

		acceleration = imu.acceleration
		z_a = acceleration[2]
		previous_altitude = altitude
		altitude = altimeter.altitude

		if status is Vehicle_Status.ON_PAD:
			if (z_a >= 5 and altitude >= 5):
				status = Vehicle_Status.BOOST
				continue
			
			# set deployment
			deployment = INIT_DEPLOYMENT
			# on pad average altitude
			if count_altitude_read == 0 or \
			(count_altitude_read < 100 and abs(altitude - average_altitude) < 0.8):

				count_altitude_read += 1
				average_altitude = (average_altitude * (count_altitude_read - 1)\
				 + altitude)/count_altitude_read

				on_PAD_altitude = average_altitude
				print(count_altitude_read)

				if (count_altitude_read == 100):
					print()
					print("average done")
					print()


			# program test
			if (altitude >= on_PAD_altitude + 1):
				start_altitude = altitude
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.BOOST:
			# do something
			if (z_a <= -0.9 * g0):
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.GLIDE:
			deployment = INIT_DEPLOYMENT + 40
			if (altitude < on_PAD_altitude - 1):
				PWM.stop(servoPin)
				PWM.cleanup()
				status = Vehicle_Status.APOGEE

			if (altitude <= on_PAD_altitude):
				status = Vehicle_Status.ON_PAD

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