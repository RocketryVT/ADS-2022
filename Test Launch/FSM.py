from enum import Enum
import time

class Vehicle_Status(Enum):
	ON_PAD = 0
	BOOST = 1
	GLIDE = 2
	APOGEE = 3

CONST_DEPLOYMENT = 15

def init():
	status = Vehicle_Status.ON_PAD
	return status

def main():

	status = init()
	previous_altitude = 0
	altitude = 0

	while (status is not Vehicle_Status.APOGEE):
		acceleration = 0 # ....
		previous_altitude = altitude
		altitude = 0 # ....

		deployment = 0


		if status is Vehicle_Status.ON_PAD:
			time.sleep(0.1)
			if (acceleration >= 5 and altitude >= 5):
				status = Vehicle_Status.BOOST

		elif status is Vehicle_Status.BOOST:
			time.sleep(0.1)
			# do something
			if (acceleration <= -0.9):
				status = Vehicle_Status.GLIDE

		elif status is Vehicle_Status.GLIDE:
			deployment = CONST_DEPLOYMENT
			if (previous_altitude - altitude < 0):
				status = Vehicle_Status.APOGEE

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

		print(status)

	# end ...



if __name__ == "__main__":
	main()