from ev3dev2.sound import Sound
spkr = Sound()
spkr.speak("Neki neki")

from ev3dev2.sensor.lego import ColorSensor
color_sensor = ColorSensor()
print(color_sensor.color)