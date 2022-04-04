import adafruit_mpl3115a2
import Adafruit_BBIO.PWM as PWM
import time
import board
from enum import Enum
class Vehicle_Status(Enum):
	ON_PAD = 0
	BOOST = 1
	GLIDE = 2
	APOGEE = 3

# mpl3115a2 default i2c bus
mpl_bus = 0x60

# servo setup parameter
servoPin = "P8_13" # 
duty_min = 3
duty_max = 14.5
servo_set = False

INIT_DEPLOYMENT = 15
ALTI_DEPL_THRESHOLD = 0.5
RATE_ALTI_DEPL = 1/50

def init():
	global altimeter
	global duty_span
	global servo_set

	# create sensor object, communicating via default I2C bus
	i2c = board.I2C()

	## altimeter setup
	# create and initialize sensor
	altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=mpl_bus)
	altimeter.sealevel_pressure = 102250

	## servo setup
	duty_span = duty_max - duty_min
	# servo might have a different parity
	PWM.start(servoPin, (100 - duty_min), 60.0, 1)
	servo_set = True

	status = Vehicle_Status.ON_PAD
	return status

def write_servo(deployment):
	global servo_set

	if not servo_set:
		PWM.start(servoPin, (100 - duty_min), 60.0, 1)
		servo_set = True

	angle_f = float(deployment)
	duty = 100 - ((angle_f / 180) * duty_span + duty_min)
	PWM.set_duty_cycle(servoPin, duty)

def main():
	status = init()
	init_altitude = altimeter.altitude
	start_altitude = 0
	previous_altitude = 0
	altitude = 0

	deployment = 0

	while (status is not Vehicle_Status.APOGEE):
		acceleration = 0 # ....
		previous_altitude = altitude
		altitude = altimeter.altitude

		if status is Vehicle_Status.ON_PAD:
			time.sleep(0.1)
			if (acceleration >= 5 and altitude >= 5):
				status = Vehicle_Status.BOOST

			# program test
			if (altitude - previous_altitude >= 0.5 or altitude - init_altitude >= 1):
				start_altitude = altitude - init_altitude
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.BOOST:
			time.sleep(0.1)
			# do something
			if (acceleration <= -0.9):
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.GLIDE:
			deployment = INIT_DEPLOYMENT + min(RATE_ALTI_DEPL * (altitude - start_altitude), 100)
			if (previous_altitude - altitude < 0):
				PWM.stop(servoPin)
				PWM.cleanup()
				status = Vehicle_Status.APOGEE

		write_servo(deployment)

		# match status:
		# 	case Vehicle_Status.ON_PAD:
		# 		time.sleep(0.1)
		# 		if (acceleration >= 5 && altitude >= 5):
		# 			status = Vehicle_Status.BOOST

		# 	case Vehicle_Status.BOOST:
		# 		time.sleep(0.1)
		# 		# do something
		# 		if (acceleration <= -0.9):
		# 			status = Vehicle_Status.GLIDE

		# 	case Vehicle_Status.GLIDE:
		# 		deployment = CONST_DEPLOYMENT
		# 		if (previous_altitude - altitude < 0):
		# 			status = Vehicle_Status.APOGEE

		print("Altitude: {0:0.3f} meter".format(altitude))
		print("Servo Angle: {0:0.3f} degree".format(deployment))
		print(status)
		time.sleep(0.1)

	# end ...



if __name__ == "__main__":
	main()