## ManageKeystrokes.py
## Description: Manages keystokes to either Quit ('q'), Restart ('r'), or Sleep ('s').
##
## Some In-Depth Information about each option:
##    Quit ('q'):    Sets a variable that notifies the ManageSerial thread to close all of its processes and to quit the
##                   game in an orderly manner.
##    Restart ('r'): Restarts all aspects of simulator without quitting socat (so the serial port stays the same).
##    Sleep ('s'):   Toggles sleep mode: Saving images will be paused (to save system resources/battery power/etc...)

from bge import logic

controller = logic.getCurrentController()
object = controller.owner

QuitKey = controller.sensors["QuitKey"]
RestartKey = controller.sensors["RestartKey"]
SleepKey = controller.sensors["SleepKey"]
ResetRotation = controller.actuators["ManageMotion"]

if(QuitKey.positive):
    object['QuitState'] = 1
elif(RestartKey.positive):
    print("Restarting... Please wait...")
    
    object['CurrentHeading'] = 0
    object['CurrentDepth'] = 0
    object['CurrentPower'] = 100
    object['CurrentDropper'] = 0
    object['DesiredHeading'] = 0
    object['DesiredDepth'] = 0
    object['DesiredPower'] = 100
    object['DesiredDropper'] = 0
    
    ResetRotation.dRot = [0.0, 0.0, 0.0]
    object.worldPosition = [0.0, 0.0, 0.0]
elif(SleepKey.positive):
    if(object['SleepState'] == 0):
        print("Sleeping: True")
        object['SleepState'] = 1
    else:
        print("Sleeping: False")
        object['SleepState'] = 0