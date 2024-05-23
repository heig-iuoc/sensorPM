#!/usr/bin/python
#
# Copyright (c) 2020, Sensirion AG
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# # Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# # Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# # Neither the name of Sensirion AG nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# (c) Copyright 2021 Sensirion AG, Switzerland

# Example to use a Sensirion SCD40 or SCD41 with a Raspbery Pi
#
# Prerequisites:
#
# - open the command line tool
#
# - Enable the i2c interface on your Raspbery Pi
# using 'sudo raspi-config'
#
# - Install python3 and pip3 and some tools
# 'sudo apt-get install python3 python3-pip i2c-dev i2c-tools wget'
#
# - Install the smbus2 library
# 'pip3 install smbus2'
#
# - Check if the sensor is recognized on the i2c bus
# executing the command 'i2cdetect -y 1'
# the result should look like this:
#      0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
# 00:          -- -- -- -- -- -- -- -- -- -- -- -- --
# 10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# 60: -- -- 62 -- -- -- -- -- -- -- -- -- -- -- -- --
#
# - Retrieve this example file from github
# 'wget -L https://raw.githubusercontent.com/Sensirion/raspberrypi-snippets/main/LD20_I2C_minmal_example.py'
#
# - Run the example 'python3 SCD4x_I2C_PYTHON_minmal_example.py'

import time
import datetime
import julian
import numpy as np
import threading #pip install thread6
from smbus2 import SMBus, i2c_msg

from pyubx2 import UBXReader
import serial

import paho.mqtt.client as mqtt #pip install paho-mqtt
import json

class GNSSData:    
    def __init__(self):
    
        self.lon_deg = np.nan
        self.lat_deg = np.nan
        self.h_ell = np.nan
        self.calendarUTCTimestamp = None
        self.JDUTCTimestamp = None
        self.dataAlreadySent = False

class PMSensorData:    
    def __init__(self):
    
        self.PM1p0 = np.nan
        self.PM2p5 = np.nan
        self.PM4p0 = np.nan
        self.PM10p0 = np.nan
        self.humidity = np.nan
        self.temperature = np.nan
        self.VOC_index = np.nan
        self.NOx_index = np.nan
        self.calendarUTCTimestamp = None
        self.JDUTCTimestamp = None
        self.dataAlreadySent = False
        
def readGNSS(ubr):
    global GNSS_epoch
    iTOW_POS = 0
    iTOW_TIME = 0
    while True:
        new_data = False
        for rawData,parsedData in ubr:
#             print(parsedData)
            if parsedData.identity == "NAV-HPPOSLLH":
                lon_deg = parsedData.lon
                lat_deg = parsedData.lat
                h_ell = parsedData.height/10**3
                iTOW_POS = parsedData.iTOW
#                 print(lon_deg, lat_deg, h_ell)
            elif parsedData.identity == "NAV-TIMEUTC":
                iTOW_TIME = parsedData.iTOW
                year = parsedData.year
                month = parsedData.month
                day = parsedData.day
                hour = parsedData.hour
                minute = parsedData.min
                second = parsedData.sec
                timestampUTC = datetime.datetime(year, month, day, hour, minute, second)
                timestampJD = julian.to_jd(timestampUTC, fmt='jd')
            
            if iTOW_POS == iTOW_TIME and iTOW_POS != 0:
                GNSS_epoch = GNSSData()
                GNSS_epoch.lon_deg = lon_deg
                GNSS_epoch.lat_deg = lat_deg
                GNSS_epoch.h_ell = h_ell
                GNSS_epoch.calendarUTCTimestamp = timestampUTC
                GNSS_epoch.JDUTCTimestamp = timestampJD
#                 print(GNSS_epoch.JDUTCTimestamp)

    return

def readPMSensor(DEVICE_ADDR, bus):
    global PMSensor_epoch
    while True:
        try:
            msg = i2c_msg.write(DEVICE_ADDR, [0x03, 0xC4])
            bus.i2c_rdwr(msg)

            # wait 1 ms for data ready
            time.sleep(0.001)

            # read 12 bytes; each three bytes in as a sequence of MSB, LSB, CRC
            # co2, temperature, rel. humidity, status
            msg = i2c_msg.read(DEVICE_ADDR, 24)
            bus.i2c_rdwr(msg)
        
            timestampUTC = datetime.datetime.now(datetime.UTC)
            timestampJD = julian.to_jd(timestampUTC, fmt='jd')
            
            # merge byte 0 and byte 1 to integer
            # co2 is in ppm
            pm1p0 = (msg.buf[0][0] << 8 | msg.buf[1][0])/10
            pm2p5 = (msg.buf[3][0] << 8 | msg.buf[4][0])/10
            pm4p0 = (msg.buf[6][0] << 8 | msg.buf[7][0])/10
            pm10p0 = (msg.buf[9][0] << 8 | msg.buf[10][0])/10

            # merge byte 3 and byte 4 to integer
            temperature = msg.buf[15][0] << 8 | msg.buf[16][0]
            # calculate temperature  according to datasheet
            temperature /= 200

            # merge byte 6 and byte 7 to integer
            humidity = msg.buf[12][0] << 8 | msg.buf[13][0]
            # calculate relative humidity according to datasheet
            humidity /= 100

            voc = (msg.buf[18][0] << 8 | msg.buf[19][0]) / 10
            nox = (msg.buf[21][0] << 8 | msg.buf[22][0]) / 10
            
            PMSensor_epoch = PMSensorData()
            PMSensor_epoch.PM1p0 = pm1p0
            PMSensor_epoch.PM2p5 = pm2p5
            PMSensor_epoch.PM4p0 = pm4p0
            PMSensor_epoch.PM10p0 = pm10p0
            PMSensor_epoch.humidity = humidity
            PMSensor_epoch.temperature = temperature
            PMSensor_epoch.VOC_index = voc
            PMSensor_epoch.NOx_index = nox
            PMSensor_epoch.calendarUTCTimestamp = timestampUTC
            PMSensor_epoch.JDUTCTimestamp = timestampJD
            
            # wait 1 s for next measurement
            time.sleep(1)
        except:
            print("PM Sensor error")
    return



#GNSS
ser = serial.Serial(
    port='/dev/serial0',
    baudrate = 115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
    )

ubr = UBXReader(ser)
GNSS_epoch = GNSSData()




#PM Sensor
PMSensor_epoch = PMSensorData()

# I2C bus 1 on a Raspberry Pi 3B+
# SDA on GPIO2=Pin3 and SCL on GPIO3=Pin5
# sensor +3.3V at Pin1 and GND at Pin6
DEVICE_BUS = 1

# device address SCD4x
DEVICE_ADDR = 0x69

# init I2C
bus = SMBus(DEVICE_BUS)

# wait 1 s for sensor start up (> 1000 ms according to datasheet)
time.sleep(1)

# start scd measurement in periodic mode, will update every 2 s
msg = i2c_msg.write(DEVICE_ADDR, [0x00, 0x21])
bus.i2c_rdwr(msg)

# wait for first measurement to be finished
time.sleep(2)
new_data = False

flux_PMSensor = threading.Thread(target=readPMSensor, args=(DEVICE_ADDR,bus,))
flux_PMSensor.start()

flux_GNSS = threading.Thread(target=readGNSS, args=(ubr,))
flux_GNSS.start()

mqtt_broker = "iuoc-tb.heig-vd.ch"
mqtt_port = 1883
mqtt_topic = "v1/devices/me/telemetry"
client = mqtt.Client(client_id="2w2grzm3fyqszdckg5e5", callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
client.connect(mqtt_broker, mqtt_port)


while True:
    timestampUTC_now = datetime.datetime.now(datetime.UTC)
    timestampJD_now = julian.to_jd(timestampUTC_now, fmt='jd')
    if GNSS_epoch.JDUTCTimestamp != None and PMSensor_epoch.JDUTCTimestamp != None:
        deltaTimeSensors = np.abs(GNSS_epoch.JDUTCTimestamp-PMSensor_epoch.JDUTCTimestamp)*86400
        deltaTimeSensorsVSnow = np.abs(timestampJD_now-PMSensor_epoch.JDUTCTimestamp)*86400
#         print(deltaTimeSensorsVSnow)
        if deltaTimeSensors < 2 and deltaTimeSensorsVSnow < 2:
            if GNSS_epoch.dataAlreadySent == False and PMSensor_epoch.dataAlreadySent == False:
                print("sendData")
                GNSS_epoch.dataAlreadySent = True
                PMSensor_epoch.dataAlreadySent = True
#                 print(GNSS_epoch.JDUTCTimestamp)
                
                list_data = []
                dictData = {"name":"lat_deg",
                            "value":GNSS_epoch.lat_deg,
                            "unit":"degres decimaux"}
                list_data.append(dictData)
                
                dictData = {"name":"lon_deg",
                            "value":GNSS_epoch.lon_deg,
                            "unit":"degres decimaux"}
                list_data.append(dictData)
                
                dictData = {"name":"h_ell",
                            "value":GNSS_epoch.h_ell,
                            "unit":"m"}
                list_data.append(dictData)
                
#                 dictData = {"name":"calendarUTCTimestamp",
#                             "value":GNSS_epoch.calendarUTCTimestamp,
#                             "unit":"datetime"}
#                 list_data.append(dictData)
                
                dictData = {"name":"JDUTCTimestamp",
                            "value":GNSS_epoch.JDUTCTimestamp,
                            "unit":"JD day"}
                list_data.append(dictData)
                
                dictData = {"name":"PM1p0",
                            "value":PMSensor_epoch.PM1p0,
                            "unit":"microgramme/m3"}
                list_data.append(dictData)
                
                dictData = {"name":"PM2p5",
                            "value":PMSensor_epoch.PM2p5,
                            "unit":"microgramme/m3"}
                list_data.append(dictData)
                
                dictData = {"name":"PM4p0",
                            "value":PMSensor_epoch.PM4p0,
                            "unit":"microgramme/m3"}
                list_data.append(dictData)
                
                dictData = {"name":"PM10p0",
                            "value":PMSensor_epoch.PM10p0,
                            "unit":"microgramme/m3"}
                list_data.append(dictData)
                
                dictData = {"name":"humidity",
                            "value":PMSensor_epoch.humidity,
                            "unit":"%RH"}
                list_data.append(dictData)
                
                dictData = {"name":"VOC_index",
                            "value":PMSensor_epoch.VOC_index,
                            "unit":"-"}
                list_data.append(dictData)
                
                dictData = {"name":"NOx_index",
                            "value":PMSensor_epoch.NOx_index,
                            "unit":"-"}
                list_data.append(dictData)
                
                dictData = {"name":"temperature",
                            "value":PMSensor_epoch.temperature,
                            "unit":"°C"}
                list_data.append(dictData)               
                
                #envoi MQTT
                dictPayload = {}
                dictPayload.update({"version":0.1})
                dictPayload.update({"name":"IUOC_INSIT_capteur_particules"})
                dictPayload.update({"cluster":"INSIT"})
                dictPayload.update({"data":list_data})
#                 dictPayload.update({"data":PMSensor_epoch.temperature})
                
                jsonPayload = json.dumps(dictPayload)
                client.publish(mqtt_topic, jsonPayload)
                print(dictPayload)
                
                
                
            
    time.sleep(0.1)

# bus.close()
