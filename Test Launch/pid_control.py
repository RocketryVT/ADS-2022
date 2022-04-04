from simple_pid import PID
import math
import time
import matplotlib.pyplot as plt

VEL_MAP = [[100,80,60,50,40,30,15,10,0],[6000,7000,8000,8500.9000,9400,9750,9900,10000]]

g = 9.81
rho = 1.2
s = 0.01824
cd0 = 0.2
mass = 26.581

init_velocity = 150
init_altitude = 0

# simulation() is a simulation of rocket flight dynamics -- sensor data
def simulation(velocity, altitude, Cd, dt):

	# with aerodynamic drag
    acceleration = -g - 0.5 * rho/mass*s*Cd * velocity ** 2
    velocity_update = velocity + acceleration*dt
    altitude_update = altitude + velocity*dt + 0.5*acceleration*dt ** 2

    # print(acceleration, velocity_update, altitude_update)

    return acceleration, velocity_update, altitude_update


# designated velocity map -- velocity to height
def desired_velocity(altitude):

    desired_velo = (100 - 0.1 * altitude)*1.2 + 30
    # desired_velo = 1000/(10+math.exp(0.01*(altitude-100)))

    return desired_velo


velo = init_velocity
alti = init_altitude

pid = PID(5, 0.1, 0.1, setpoint=velo)
pid.output_limits = (-0.4, 1)

start_time = time.time()
last_time = start_time

# Keep track of values for plotting
setpoint, a, z, y, x,  = [], [], [], [], []

# while time.time() - start_time < 100 and velo >= 0:
while velo >= 0:
    current_time = time.time()
    dt = current_time - last_time

    cd = pid(velo)
    print (cd, velo)
    print ()
    [acceleration,velo,alti] = simulation(velo,alti,cd,dt)
   
    x += [current_time - start_time]
    y += [velo]
    z += [alti]
    a += [acceleration]
    setpoint += [pid.setpoint]

    if current_time - start_time > 0:
        pid.setpoint = desired_velocity(alti)

    last_time = current_time

print(alti)

plt.plot(x, y, label='measured')
plt.plot(x, setpoint, label='target')
plt.figure()
plt.plot(x, a , label='acceleration')
# plt.plot(x, z , label='altitude')
plt.xlabel('time')
plt.ylabel('velocity')
plt.legend()
plt.show()