## ManageMotion.py
## Description: For every game frame, run updates on submarine movement based on new desired Heading/Depth/Power while
##              maintaining existing Heading/Depth/Power if no new data is avaliable.
##
## Some Basic FAQ about ManageMotion:
##
## What variables can we set and what do they do?
##    In the object UserPreferences, you will find all user-changable variables, as follows:
##    Each is designed to fine tune the rate of change of movement and scale of movement of the submarine.
##    Note that everything is updated ONCE PER FRAME.
##    
##    1) 'PowerScale': Sets the amount the submarine moves per unit of power (Ex: A PowerScale of 1 means the submarine
##        move 1 meter per frame).
##    2) 'HeadingRate': Changes the rate of change of the heading in degrees (to make movement more realistic).
##    3) 'DepthScale': Sets how much the submarine goes down per one unit of depth (Ex: A DepthScale of 1 with a desired
##        depth of 1 means that the submarine will go down to 1 meter; A DepthScale of 2 with a desired depth of 1 means that
##        the submarine will go down to 2 meters).
##    4) 'DepthRate': Changes the rate at which depth changes (in meters per frame).

from bge import render
from bge import logic
import math

controller = logic.getCurrentController()
object = controller.owner
userPreferences = logic.getCurrentScene().objects["UserPreferences"]
motion = controller.actuators["ManageMotion"]

Heading = -1*int(object.worldOrientation.to_euler().z*(180/math.pi))
object['CurrentPower'] = object['DesiredPower']
object['CurrentDepth'] = -1*object.position[2]*(1/userPreferences['DepthScale'])

if(Heading < 0):
    Heading = Heading + 360

object['CurrentHeading'] = Heading

if(Heading == object['DesiredHeading']):
    motion.dRot = [0.0, 0.0, 0.0]
else:
    HeadingRateRad = (userPreferences['HeadingRate']*(math.pi/180))
    
    HeadingDifference = Heading-object['DesiredHeading']

    if((HeadingDifference >= -180) and (HeadingDifference <= 0)):
        motion.dRot = [0.0, 0.0, (-1*HeadingRateRad)]
    elif((HeadingDifference >= 180) and (HeadingDifference <= 360)):
        motion.dRot = [0.0, 0.0, (-1*HeadingRateRad)]
    elif((HeadingDifference > -360) and (HeadingDifference < -180)):
        motion.dRot = [0.0, 0.0, HeadingRateRad]
    elif((HeadingDifference > 0) and (HeadingDifference < 180)):
        motion.dRot = [0.0, 0.0, HeadingRateRad]
    else:
        motion.dRot = [0.0, 0.0, 0.0]

DepthChange = 0.0
ScaledCurrentDepth = float(userPreferences['DepthScale']*object['CurrentDepth'])
ScaledDesiredDepth = float(userPreferences['DepthScale']*object['DesiredDepth'])

if(ScaledCurrentDepth != ScaledDesiredDepth):
    DepthDifference = ScaledCurrentDepth-ScaledDesiredDepth
    if(DepthDifference < 0):
        if(math.fabs(DepthDifference) < userPreferences['DepthRate']):
            DepthChange = -1*(userPreferences['DepthRate']-math.fabs(DepthDifference))
        else:
            DepthChange = -1*userPreferences['DepthRate']
    else:
        if(math.fabs(DepthDifference) < userPreferences['DepthRate']):
            DepthChange = userPreferences['DepthRate']-math.fabs(DepthDifference)
        else:
            DepthChange = userPreferences['DepthRate']

if(object['CurrentDropper'] != object['DesiredDropper']):
    logic.getCurrentScene().addObject("Dropper", object)
    object['CurrentDropper'] = object['DesiredDropper']

motion.dLoc = [0.0, ((object['CurrentPower']-100)*userPreferences['PowerScale']), DepthChange]

controller.activate(motion)