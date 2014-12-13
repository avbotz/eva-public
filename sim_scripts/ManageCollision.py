## ManageCollision.py
## Description: Detects whether the submarine has collided with or is near a certain task.

from bge import logic

controller = logic.getCurrentController()

collisionNear = controller.sensors["CollisionNear"]
collisionFar = controller.sensors["CollisionFar"]

if(collisionNear.positive):
	print("Near Object Hit: " + str(collisionNear.hitObject.name))
elif(collisionFar.positive):
	print("Far Object Hit: " + str(collisionFar.hitObject.name))