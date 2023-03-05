#!/usr/bin/env python3

from time import sleep

# from ev3dev.ev3 import *
from ev3dev.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_C, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev.sensor import INPUT_1
from ev3dev.sensor.lego import TouchSensor
from ev3dev.led import Leds

from ev3dev.ev3 import TouchSensor, Button, LargeMotor, Sound
# Na EV3 robotu je potrebno namestiti paketa ujson in pycurl:
# sudo apt-get update
# sudo apt-get install python3-pycurl
# sudo apt-get install python3-ujson
import pycurl
import ujson
import sys
import math
from io import BytesIO
from time import time, sleep
from enum import Enum
from collections import deque

# TODO: Add code here

def beep(duration=1000, freq=440):
    """
    Potrobi s frekvenco `freq` za cas `duration`. Klic ne blokira.
    """
    Sound.tone(freq, duration)
    # ce zelimo, da blokira, dokler se pisk ne konca.
    #Sound.tone(freq, duration).wait()

# Connect motors
left_motor = LargeMotor('outB')
right_motor = LargeMotor('outC')

beep()

# Set motor speeds and angles
left_motor.run_to_rel_pos(position_sp=180, speed_sp=300)
right_motor.run_to_rel_pos(position_sp=-180, speed_sp=300)

# Wait for the motors to finish turning
left_motor.wait_while('running')
right_motor.wait_while('running')

# Stop motors
left_motor.stop()
right_motor.stop()