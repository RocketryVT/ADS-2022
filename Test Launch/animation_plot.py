import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import adafruit_mpl3115a2

# Create figure for plotting
fig = plt.figure()
ax = fig.add_subplot(1, 1, 1)
xs = []
ys = []

# Initialize communication with TMP102
i2c = board.I2C()
alti = altimeter_set(i2c)

def altimeter_set(i2c):

    altimeter = adafruit_mpl3115a2.MPL3115A2(i2c, address=MPL_BUS)
    altimeter.sealevel_pressure = pad_pressure
    return altimeter

# This function is called periodically from FuncAnimation
def animate(i, xs, ys):

    # Read temperature (Celsius) from TMP102
    alti_c = round(alti.altitude, 2)

    # Add x and y to lists
    xs.append(dt.datetime.now().strftime('%H:%M:%S.%f'))
    ys.append(alti_c)

    # Limit x and y lists to 20 items
    xs = xs[-20:]
    ys = ys[-20:]

    # Draw x and y lists
    ax.clear()
    ax.plot(xs, ys)

    # Format plot
    plt.xticks(rotation=45, ha='right')
    plt.subplots_adjust(bottom=0.30)
    plt.title('TMP102 Temperature over Time')
    plt.ylabel('Temperature (deg C)')

# Set up plot to call animate() function periodically
ani = animation.FuncAnimation(fig, animate, fargs=(xs, ys), interval=1000)
plt.show()
