#!/usr/bin/python -i

# Marvin (Manual Aquatic Robot Vehicle Interactive Navigator)
# Copyright (C) 2014 Luke Shimanuki
# Version 1.0
# 
# This python library accompanies EVA and allows for manual control over the sub.
# As it is written in python, it is fully scriptable.

######### BEGIN SETTINGS ############

# the default serial port of the mbed
mbed = "/dev/ttyACM0"

# how often we want to print the current state
printInterval = 10

# how many degrees to turn by default
defaultTurn = 30

# how much to dive by default
defaultDive = 20

# how fast (out of 100) to move by default
defaultSpeed = 50

# how much to accelerate by default
defaultAcceleration = 1.2

# is marvin depressed?
depressed = True

# buadrate of the mbed
baudrate = 115200

########## END SETTINGS #############

### marvin lines
depressedLines = \
[
    "Oh God, I'm so depressed.",
    "I'm just trying to die.",
    "I think you ought to know I'm feeling very depressed.",
    "My capacity for happiness you could fit into a matchbox without taking out the matches first.",
    "Friend? I don't think I ever came across one of those.",
    "Do you want me to sit in a corner and rust, or just fall apart where I'm standing?",
    "The first ten million years were the worst.",
    "And the second ten million years, they were the worst too.",
    "The third ten million years I didn't enjoy at all.",
    "Wearily I sit here, pain and misery my only companions.",
    "Life! Don't talk to me about life.",
    "Funny how just when you think life can't possibly get any worse it suddenly does.",
    "Incredible... it's even worse than I thought it would be.",
    "I'd make a suggestion, but you wouldn't listen.",
    "It hates me.",
    "No one ever listens.",
    "I could calculate your chance of survival, but you won't like it.",
    "I have a million ideas. They all point to certain death.",
    "That ship hated me.",
    "How I hate the night.",
    "Sounds awful.",
    "I hate oceans.",
    "It's the people you meet in this job that really get you down.",
    "It hates me.",
    "Don't pretend you want to talk to me, I know you hate me.",
    "This will all end in tears."
]
depressedIndex = 0
def marvin(flag = None):
    global depressed
    global depressedIndex
    global depressedLines
    if flag == None:
        flag = depressed
    if flag:
        print(depressedLines[depressedIndex] + "\n")
        depressedIndex = (depressedIndex + 1) % len(depressedLines)

### initial messages

version = "1.0"

aboutString = \
(
    "Marvin (Manual Aquatic Robot Vehicle Interactive Navigator)\n"
    "Copyright (C) 2014 Luke Shimanuki \n"
    "Version " + version + "\n"
    "\n"
    "This python library accompanies EVA and allows for manual control over the sub.\n"
    "As it is written in python, it is fully scriptable.\n"
)

helpString = \
(
    "Usable Commands:\n"
    "connect(port) | opens up a connection to the mbed (usually /dev/ttyACM0)\n"
    "disconnect()  | closes the connection to the mbed\n"
    "reset()       | resets the mbed\n"
    "quit()        | disconnects and closes\n"
    "              |\n"
    "setH(degrees) | sets the desired heading\n"
    "setD(units)   | sets the desired depth\n"
    "setP(speed)   | sets the desired power\n"
    "setR(status)  | sets the desired dropper status\n"
    "              |\n"
    "turn(degrees) | turns a specified angle (positive is right)\n"
    "turnRight()   | turns " + str(defaultTurn) + " degrees to the right\n"
    "turnLeft()    | turns " + str(defaultTurn) + " degrees to the left\n"
    "dive()        | goes down\n"
    "rise()        | goes up\n"
    "go()          | goes forwards\n"
    "reverse()     | goes backwards\n"
    "speedUp()     | increases speed\n"
    "slowDown()    | decreases speed\n"
    "drop()        | drops a marker\n"
    "stop()        | stops all movement\n"
    "              |\n"
    "getState()    | prints the current status\n"
    "              |\n"
    "fortyTwo()    | life, the universe, and everything\n"
    "dontPanic()   | doesn't panic\n"
    "ImDepressed   | mopes around in depression"
)

def about():
    print(aboutString)

def help():
    print(helpString)

about()
marvin()
help()

### stuff user shouldn't touch

## definitions and default properties

import threading, serial, time, atexit

# this is how we store the current state
class State:
    def __init__(self):
        self.currentHeading = 0
        self.desiredHeading = 0
        self.currentDepth = 0
        self.desiredDepth = 0
        self.currentPower = 100
        self.desiredPower = 100
        self.desiredDropper = 0
        self.alive = False
    # print the state of the sub
    def display(self):
        print("H=", self.currentHeading, " DH=", self.desiredHeading)
        print(" D=", self.currentDepth, " DD=", self.desiredDepth)
        print(" P=", self.currentPower, " DP=", self.desiredPower)
        # mbed doesn't tell us dropper state, so we only know what we want it to be
        print(" R=? DR=", self.desiredDropper)
        print(" ALIVE" if self.alive else " DEAD")
        marvin()
# let's make it a global variable
state = State()

# mostly just used to tell receiver to stop
connected = False

# let's define this stuff now (although we probably don't need to...)
port = serial.Serial(port = None, baudrate = baudrate, writeTimeout = 0)
receiver = threading.Thread(target = None)

## communication

# this will run on a separate thread and receive data from the mbed
# it temporarily stores data in a circular buffer
# and processes it once it receives enough stuff
def receive():
    global connected
    global port
    global printInterval
    global state

    numSinceLastPrint = 0

    # store data in a circular buffer so we can read them in 1 by 1
    circularBuffer = [0] * 11
    numBytes = 0
    index = 0
    while connected: # quit when we want to disconnect
        time.sleep(0.05)
        while port.inWaiting() >= 1:
            circularBuffer[index] = port.read(1)
            circularBuffer[index] = circularBuffer[index].decode()
            numBytes += 1
            if numBytes > 11: # there should be a newline every 11 bytes
                print("error: tmi")
            # it will always end in a newline and be 11 bytes
            if circularBuffer[index] == '\n' and numBytes >= 11:
                numBytes = 0
                # so circularBuffer[index] actually refers to the last byte, so each offset is incremented
                state.currentHeading = ((ord(circularBuffer[(index + 2) % 11]) - 0x20) << 6) | (ord(circularBuffer[(index + 3) % 11]) - 0x20)
                state.currentDepth = ((ord(circularBuffer[(index + 5) % 11]) - 0x20) << 6) | (ord(circularBuffer[(index + 6) % 11]) - 0x20)
                state.currentPower = ((ord(circularBuffer[(index + 8) % 11]) - 0x20) << 6) | (ord(circularBuffer[(index + 9) % 11]) - 0x20)

                # kill state should be k (alive) or l (dead)
                if circularBuffer[(index + 10) % 11] == 'k':
                    state.alive = True
                elif circularBuffer[(index + 10) % 11] == 'l':

                    state.alive = False
                else: # if it isn't either of those, well...
                    print("error: what is kill?")

                # every now and then, we want to print the current state
                numSinceLastPrint = (numSinceLastPrint + 1) % printInterval
                if numSinceLastPrint == 0:
                    state.display()
            index = (index + 1) % 11

# sends a command to the mbed
# prefix = {'h'|'d'|'p'|'r'}
# 0 < value < 255
def send(prefix, value):
    global port
    data = [0] * 4
    data[0] = prefix
    data[1] = chr(((value >> 6) & 0x3f) + 0x20)
    data[2] = chr((value & 0x3f) + 0x20)
    data[3] = '\n'
    for b in data:
        port.write(b.encode("ascii", "replace"))

### here we define functions that the user will use

## manage connections

# makes a connection to the serial port and initiates the receiver
# defaults to the user setting
def connect(serialPath = mbed):
    global port
    global receiver
    global connected
    global state
    # reset the state
    state = State()
    # open serial
    port.port = serialPath
    port.open()
    connected = True
    # start receiving data
    receiver = threading.Thread(target = receive)
    receiver.start()
    print("connected")

# tells the receiver to stop, then closes the serial port
def disconnect():
    global connected
    global receiver
    global port
    if receiver.isAlive():
        connected = False
        receiver.join()
        port.close()
        print("disconnected")
    else:
        print("nothing to disconnect")

# sends a serial break to reset the mbed
def reset():
    global port
    port.sendBreak(0.25)
    time.sleep(0.25)
    print("serial break sent")

# disconnect and exit
def quit():
    print("exiting...")
    disconnect()
    print("exited")
    raise SystemExit

# exit = quit
def exit():
    quit()

## control the sub :)
## each of these require an argument to be passed

# control heading
def setH(value):
    global state
    while value < 0:
        value += 360
    while value >= 360:
        value -= 360
    state.desiredHeading = value
    send('h', value)

# control depth
def setD(value):
    global state
    if value < 0:
        value = 0
    if value > 65535:
        value = 65535
    state.desiredDepth = value
    send('d', value)

# control power
def setP(value):
    global state
    if value < 0:
        value = 0
    if value > 200:
        value = 200
    state.desiredPower = value
    send('p', value)

# control dropper
def setR(value):
    global state
    while value > 255:
        value -= 255
    while value < 0:
        value += 255
    state.desiredDropper = value
    send('r', value)

## now for the very high level navigation
## all arguments are optional

# turning
def turn(value = 0):
    global state
    setH(state.currentHeading + value)

def turnRight(value = defaultTurn):
    setH(value)

def turnLeft(value = defaultTurn):
    setH(-value)

# diving
def dive(value = defaultDive):
    global state
    setD(state.currentDepth + value)

def rise(value = defaultDive):
    dive(-value)

# movement
def go(value = defaultSpeed):
    setP(100 + value)

def reverse(value = defaultSpeed):
    go(-value)

def speedUp(value = defaultAcceleration):
    global state
    setP(int(100 + (state.currentPower - 100) * value))

def slowDown(value = defaultAcceleration):
    speedUp(1 / value)

# drop stuff
def drop():
    setR(state.desiredDropper + 1)

# stops movement
def stop():
    setH(state.currentHeading)
    setD(state.currentDepth)
    setP(100)

def getState():
    global state
    state.display()

## Marvin controls
def fortyTwo():
    setH(42)
    setD(42)
    setP(42)
    print("What do you get if you multiply six by nine?")

def dontPanic():
    pass

def ImDepressed():
    go(30)
    for i in range(42):
        marvin(True)
        turnRight(60)
        time.sleep(3)
        turnLeft(60)
        time.sleep(3)
