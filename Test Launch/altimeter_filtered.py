# SPDX-FileCopyrightText: 2019 Tony DiCola for Adafruit Industries
# SPDX-License-Identifier: MIT

# Simple demo of the MPL3115A2 sensor.
# Will read the pressure and temperature and print them out every second.
import time
import board
import adafruit_mpl3115a2
import numpy as np
from scipy.signal import butter, lfilter, freqz


def butter_lowpass(cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# Setting standard filter requirements.
order = 6
fs = 5.0       
cutoff = 1  

b, a = butter_lowpass(cutoff, fs, order)


# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA

# Initialize the MPL3115A2.
sensor = adafruit_mpl3115a2.MPL3115A2(i2c)
# Alternatively you can specify a different I2C address for the device:
# sensor = adafruit_mpl3115a2.MPL3115A2(i2c, address=0x10)

# You can configure the pressure at sealevel to get better altitude estimates.
# This value has to be looked up from your local weather forecast or meteorological
# reports.  It will change day by day and even hour by hour with weather
# changes.  Remember altitude estimation from barometric pressure is not exact!
# Set this to a value in pascals:
sensor.sealevel_pressure = 102250

# Main loop to read the sensor values and print them every second.
data = []
count = 0
while True:

    if count <= 200:
        altitude = sensor.altitude
        data.append(altitude)
        count += 1

    else:
        altitude = sensor.altitude
        data.pop(0)
        data.append(altitude)
        print(data)


        filtered = butter_lowpass_filter(data, cutoff, fs, order)
        print(filtered)

    time.sleep(0.01)