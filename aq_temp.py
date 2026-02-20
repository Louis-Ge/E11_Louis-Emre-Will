# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

"""
Example sketch to connect to PM2.5 sensor with either I2C or UART.
"""

import time

import board
import busio
from digitalio import DigitalInOut, Direction, Pull

from adafruit_pm25.i2c import PM25_I2C

import adafruit_bme680

import csv

fmt = "%H:%M:%S"

reset_pin = None
# If you have a GPIO, its not a bad idea to connect it to the RESET pin
# reset_pin = DigitalInOut(board.G0)
# reset_pin.direction = Direction.OUTPUT
# reset_pin.value = False


# For use with a computer running Windows:
# import serial
# uart = serial.Serial("COM30", baudrate=9600, timeout=1)

# For use with microcontroller board:
# (Connect the sensor TX pin to the board/computer RX pin)
# uart = busio.UART(board.TX, board.RX, baudrate=9600)

# For use with Raspberry Pi/Linux:
import serial
uart = serial.Serial("/dev/ttyS0", baudrate=9600, timeout=0.25)

# For use with USB-to-serial cable:
# import serial
# uart = serial.Serial("/dev/ttyUSB0", baudrate=9600, timeout=0.25)

# Connect to a PM2.5 sensor over UART
from adafruit_pm25.uart import PM25_UART
pm25 = PM25_UART(uart, reset_pin)

# Create library object, use 'slow' 100KHz frequency!
# i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
# Connect to a PM2.5 sensor over I2C
# pm25 = PM25_I2C(i2c, reset_pin)

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller
bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c, debug=False)

# change this to match the location's pressure (hPa) at sea level
bme680.sea_level_pressure = 1013.25

# You will usually have to add an offset to account for the temperature of
# the sensor. This is usually around 5 degrees but varies by use. Use a
# separate temperature sensor to calibrate this one.
temperature_offset = -5

print("Found PM2.5 sensor, reading data...")

f = open('data/aq_temp.csv', 'w', newline=None)
csvwriter = csv.writer(f, delimiter=',')
csvwriter.writerow(['Time','0.3','0.5','1.0','2.5','5.0','10','Temperature','Pressure','Humidity'])

usertime = int(input('How long would you like to take measurements?\n'))
t = 1

while t <= usertime:
    try:
        aqdata = pm25.read()
    except RuntimeError:
        print("Unable to read from sensor, retrying...")
        continue

    csvwriter.writerow([time.time(),aqdata["particles 03um"],aqdata["particles 05um"],aqdata["particles 10um"],aqdata["particles 25um"],aqdata["particles 50um"],aqdata["particles 100um"],bme680.temperature + temperature_offset,bme680.pressure,bme680.relative_humidity])

    print()
    print(f"\nTime: {time.strftime(fmt, time.localtime())}")
    print("Concentration Units (standard)")
    print("---------------------------------------")
    print("PM 1.0: %d\tPM2.5: %d\tPM10: %d"% (aqdata["pm10 standard"], aqdata["pm25 standard"], aqdata["pm100 standard"]))
    print("Concentration Units (environmental)")
    print("---------------------------------------")
    print("PM 1.0: %d\tPM2.5: %d\tPM10: %d"% (aqdata["pm10 env"], aqdata["pm25 env"], aqdata["pm100 env"]))
    print("---------------------------------------")
    print("Particles > 0.3um / 0.1L air:", aqdata["particles 03um"])
    print("Particles > 0.5um / 0.1L air:", aqdata["particles 05um"])
    print("Particles > 1.0um / 0.1L air:", aqdata["particles 10um"])
    print("Particles > 2.5um / 0.1L air:", aqdata["particles 25um"])
    print("Particles > 5.0um / 0.1L air:", aqdata["particles 50um"])
    print("Particles > 10 um / 0.1L air:", aqdata["particles 100um"])
    print("---------------------------------------")
    
    print(f"Temperature: {(bme680.temperature + temperature_offset):.1f} C, Gas: {bme680.gas} ohm, Humidity: {bme680.relative_humidity:0.1f} %, Pressure: {bme680.pressure:0.3f} hPa, Altitude = {bme680.altitude:0.2f} meters")

    t += 1
    time.sleep(1)

f.close()
