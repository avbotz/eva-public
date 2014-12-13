## SaveImage.py
## Description: Saves Camera Images periodically (depending on the value set in the Logic Editor) for EVA.
##              If SleepState is true, image saving is paused.

from bge import render
from bge import logic

controller = logic.getCurrentController()
object = controller.owner
path = logic.expandPath('//')

if(object['SleepState'] == 0):
    render.setWindowSize(object['WindowWidth'], object['WindowHeight']*2)
    render.makeScreenshot(path + 'Camera.png')