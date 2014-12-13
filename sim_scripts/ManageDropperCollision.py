## ManageDropperCollision.py
## Description: Determines whether the dropper made it into the bin.

from bge import logic

controller = logic.getCurrentController()

collisionNear = controller.sensors["Near"]

if(collisionNear.positive):
	print("Dropper Hit Object: " + str(collisionNear.hitObject.name))