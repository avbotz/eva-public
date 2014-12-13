# AVBotz Simulator Documentation - Version 1.2
## Preface
The AVBotz Simulator is designed for the sole purpose of aiding the development of Extensive Vehicular Automaton (EVA) for the RoboSub Competition.

Several additional programs and their licenses must be acknowledged due to their use in this program setup. [Blender](http://www.blender.org/) is under the GNU General Public License Version 3 or later. [Socat](http://www.dest-unreach.org/socat/) is under the GNU General Public License Version 2. [PySerial](http://pyserial.sourceforge.net/) is under a custom license, found [here](http://pyserial.sourceforge.net/appendix.html#license).

## Screenshots
View screenshots and reference pictures of the simulator [here](https://drive.google.com/folderview?id=0B6kWiLVmj0_2REF1c200UHl3Nkk&usp=sharing).

## General Information
### 1-0. Design
The simulator emulates the MBED (and corresponding AVNavControl) and cameras by creating a virtual environment in Blender, a 3D modeling software and environment simulator.

Normally, EVA interprets camera data and sends desired heading, depth, and power to the MBED via a serial connection, which then adjusts the motors. The simulator creates a virtual serial port to which EVA can send data and does adjustments in the virtual environment. It saves an image of this virtual environment periodically, which EVA can read and interpret.

### 1-1. Setup
*Note: The simulator has only been tested on Linux machines. It is not recommended that you attempt to use Windows or Apple machines to run this program, given the fact that some parts of EVA and the simulator depends on Linux system calls and Video4linux. A virtual machine is an alternative for this predicament.*

Before attempting to setup the simulator, ensure that you pull the latest version of EVA from Github. Similarly, ensure that you can compile EVA on your machine (Requires `Opencv`, `Libconfig++`, and `Video4linux` libraries).

The simulator requires [Blender](http://www.blender.org/) (Tested on Version 2.70) and [Socat](http://www.dest-unreach.org/socat/). Either download the programs from the links above or use a package manager.

Check that Socat is installed in a way that it is in your terminal’s PATH (inside of or linked to `/usr/bin`).

To install PySerial, download and extract a specialized version from [here](https://docs.google.com/file/d/0B6kWiLVmj0_2bFlYcjM3QnVyVHM/edit). Navigate to the base directory where Blender is installed and paste the extracted serial folder into `Blender/[2.70]/python/lib/python3.3`.

### 1-2. Basic Usage
Open Blender in a terminal window.

Go to `File > Open` and open the simulator.blend from EVA’s directory. On the right under Standalone Player, press Start.
The window that appears shows the simulator’s point of view, with the top half being the Front Camera and the bottom half being the Bottom Camera.

*Note: If your screen size is smaller than 720 pixels in height (or if there are any screen height restrictions like docks), the image may possibly be cropped and result in errors.*

In the terminal window, note the serial port it tells you to set as the MBED serial port (Ex: `Set the mbed serial port to: /dev/pts/5`).
Open `simulation.conf` in a text editor and change `mbed_port`’s value to this serial port. This variable should be located near the bottom of the file.

Run EVA by typing this in another terminal window: `./eva simulation.conf`
If you managed to make this work, you should see EVA logging and possibly some movement in the simulator window.

To close the simulator, press `Control-C` in EVA. Select the simulator window and press `Q` until the window closes.

#### 1-2-0. Keyboard Commands
There are several keyboard commands that can be used in the simulator window:

`Q`: Quits the simulator in an orderly manner. Necessary to avoid issues with Socat.

`R`: Restarts the simulator without quitting Socat. Good for easily re-running a task.

`S`: Toggles "sleep" mode, meaning that the simulator won't save images. Useful if you are not doing anything but don't want to quit/restart the simulator (it also saves system resources/battery life during idle time).

### 1-3. Advanced Usage
Now what? Well, you need to give the simulator some things to interact with!

#### 1-3-0. Simulator Abilities
The simulator is able to do the basic heading, depth, and power changes. In addition, it is able to simulate droppers (using the `r` flag).

Whenever the submarine collides with an object, the simulator will notify you by printing something in the terminal (Ex: `Near Object Hit: Buoy`). It will also notify you if the submarine is 2.5 meters away from an object or if the dropper made it into a bin.

#### 1-3-1. Using Props
Luckily, some props have been pre-fabricated for your testing usage. These include Gate, Path, Bins, Buoys (1 Red and 1 RGB), and 2014 Maneuvering Task. If these do not suit your fancy, you can model your own (using Google or other team members).

These props are located in different layers. Locate these different layers at the bottom. The top-left box is the active layer, or the layer where the simulator interacts. Click on different layers to get to the different props.

Right-click to select the prop (it should glow orange/yellow). Press `M` and select the top-left box to move the object to the active layer.

*Tips: Press `Shift-C` to zoom into the object of each layer. If you accidentally rotate the view, go to the bottom-left and select `View > Top`.*

Still selecting the object, press `G` to move the object around the scene. Press `R` to rotate the object.

It is currently in wireframe and orthographic projection mode. Press `Z` to toggle wireframe mode.

A basic knowledge of Blender can be helpful. Google is your friend.

#### 1-3-2. Selecting Tasks in EVA
To choose a specific task for EVA to run, open `simulation.conf` in a text editor. Under `Gate`, change `next_task` to the task that you wish to run.

### 1-4. User Preferences
There are several user preferences that can be set for the simulator, including rate of heading change, rate of depth change, scale of power, and scale of depth.

From the list of objects on the top-right, select `UserPreferences`. Select the menu (cube icon) from the bottom-left corner and select Logic Editor.

You can then change the variables on the left hand side. In all settings, it is per frame (frame rate is set as 20 frames per second).

#### Here’s an overview:
**PowerScale**: How many meters the submarine moves per unit power.

**HeadingRate**: How many degrees the submarine changes per frame.

**DepthScale**: How many meters the submarine moves down per unit depth.

**DepthRate**: How fast the submarine descends, in meters per frame.

There is more comprehensive information in `ManageMotion.py`.

## Miscellaneous Information
### 2-0. Libpng Errors
During the running of the simulator, EVA will probably print out `libpng error`. This is completely normal as, although it is an actual read error, it occurs when EVA tries to read an image when Blender is attempting to write an image. Since a while loop already accounts for this, it does not affect the simulator’s results.

### 2-1. Opencv and Nvidia Drivers
At least on Ubuntu Linux distributions, using `apt-get` to install opencv will result in the installation of the dependency `libopencv-ocl`, which will correspondingly install nvidia video drivers. This may possibly cause problems with your current video driver (unless you actually are using nvidia drivers) and will most likely cause problems with Blender. To counter this problem, install `ocl-icd-libopencl1` after installing opencv; this will replace the nvidia drivers as the provider for `libopencv-ocl`.

### 2-2. Notes on Mac OS X
Code modifications are required to use the simulator on Mac OS X. It would still be preferable to use Linux. If you’re determined, however…

Prerequisites: You need a C++ compiler (compatible with C++11: from Apple’s Developer website (clang/LLVM) or elsewhere), `OpenCV` libraries installed, `Libconfig++` libraries installed, and `Socat` (Pre-compiled Binary for Mac OS X Mavericks 64-Bit [Here](https://drive.google.com/file/d/0B6kWiLVmj0_2THFPRVRfSy1wa0U/edit?usp=sharing)).

*Note that you will probably have to compile some of these things yourself.* Undaunted by this task? Let’s continue, then.

* In `Eva.cpp`, Line 145-ish, change `vision = new VisionNormal(config);` to `exit(0);` or your favorite exit method.
* In `Eva.h`, remove line `#include "vision_normal.h"`.
* In `Types.h`, change `#include <string.h>` to `#include <string>`, because Mac OS X is unusual like that.
* Delete `Vision_normal.cpp`, `Vision_normal.h`, `Camera.cpp`, and `Camera.h`.
* Add the Serial Folder (see original directions) to `/Applications/Blender/blender.app/Contents/MacOS/[2.70]/python/lib/python3.3/` and `/Applications/Blender/blenderplayer.app/Contents/MacOS/[2.70]/python/lib/python3.3/`.

Run Blender through the terminal by using this command: `/Applications/Blender/blender.app/Contents/MacOS/blender`.

### 2-3. Simulator Code
Blender uses python to emulate the actions taken by the submarine. An actuator runs each python script separately (basically, each script runs at the same time as the others).

#### Here is an overview:
**ManageCollision.py**: Prints whether the submarine is near (2.5 meters near) or has collided with an object.

**ManageDropperCollision.py**: Prints out if the dropper object has collided with another object (used to see if the dropper hits the bins).

**ManageMotion.py**: Controls the motion of the submarine by comparing the difference between desired and current Heading/Depth/Power. Also drops the dropper object when requested.

**ManageSerial.py**: Manages the virtual serial port (communicates with EVA). It runs Socat and the serial port on a separate thread.

**Quit.py**:  When `Q` is pressed, it sets a variable, calling for the application to systematically quit.

**SaveImage.py**: Saves an image in png format every 5 frames.

### 2-4. Some Common Errors
#### Blender says something like “Xlib: extension "GLX" missing on display”?
See *OpenCV and Nvidia Drivers*.

#### Blender just doesn’t feel like working?
Make sure you installed Socat and PySerial libraries exactly as specified above. If you are using a version of Blender other than 2.70, download 2.70 and try using that.

#### CPU maxed out? Short on RAM?
Blender, being both the environment stager and simulator (and additionally being a 3D modeling program, a video editor, and image renderer) can take up a deal of system resources. It is unlikely, however, that it will consume all the resources of a modern computer unless it is running simultaneously with (perhaps) a 50 tabbed web browser.

#### EVA won’t compile?
Ensure that you have the correct libraries installed. `OpenCV`, `Libconfig++`, and `Video4Linux` should be all the external libraries required.

Possibly, your libraries are installed in a different location than what is currently defined. Edit the Makefile and change all the include and library paths (perhaps from /usr/local/include to /usr/include, or something similar).

#### See *Can’t create… No such file or directory*?
Create a folder named build in the root directory.

#### Outdated GCC version?
EVA requires a g++ version that support C++11.

#### Socat changes the serial ports every time I restart the simulator?
If you’re not pressing `Q` to quit the simulator, this happens. You can run `pkill socat` to kill all running instances of Socat.
If you are pressing `Q` to quit, then this is simply a random occurrence. You have to place the new serial port in the configuration file every time it changes.

If you want to avoid this problem, simply use the symlink instead of the raw serial port.

However, incorrectly quitting the simulator will result in this symlink persisting on your system (since Socat is still running in the background). You can run `pkill socat` to remove it.