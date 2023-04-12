from ev3dev.ev3 import TouchSensor, Button, LargeMotor, Sound, ColorSensor, Motor
from ev3dev2.sound import Sound

def move_up(motor, position=0,speed=300):
    # motor.on_to_position(speed=speed, position=up)
    print('moving up')
    motor.run_to_abs_pos(position_sp = position, speed_sp = speed)

def move_down(motor,position=-60, speed=300):
    # motor.on_to_position(speed=speed, position=up)
    print('moving down')
    motor.run_to_abs_pos(position_sp= position, speed_sp = speed)

HANDLE_RANGE = 10 # razpon premika rocke


spkr = Sound()
spkr.speak("start")

port = 'outA'
handle = Motor(port)
handle.reset()
if not handle.connected:
    print('\nPriklopi motor na izhod ' + port +
            ' in pritisni ter spusti gumb DOL.')

print(handle.__dir__())

# save start position
start = handle.position
up = start
down = start - HANDLE_RANGE
print(up, down)

# set stop action (how the motor will stop)
# print(handle.stop_actions)
# handle.stop_action = 'brake'

spkr.speak("move down")
print(handle.commands)
move_down(handle)


handle.wait_until_not_moving()
print(handle.position)
spkr.speak("move up")
move_up(handle)
# handle.run_to_rel_pos(position_sp = handle.position, speed_sp = 300)

# Run to a position relative to the current position value. 
# The new position will be current position + position_sp. 
# When the new position is reached, the motor will stop 
# using the action specified by stop_action

# handle.run_to_rel_pos()

# run_to_abs_pos(**kwargs)
# Run to an absolute position specified by position_sp and then stop 
# using the action specified in stop_action.

# handle.run_to_abs_pos()


handle.wait_until_not_moving()
print(handle.position)
handle.reset()
spkr.speak("end")