## ManageSerial.py
## Description: Creates a new thread to accept and send MBED values to and from EVA. Utilizes the PySerial library and
##              the program socat. It also controls the quitting mechanism.

from bge import logic

from subprocess import Popen, PIPE, STDOUT
import os, pty, threading, serial, signal, time

controller = logic.getCurrentController()
object = controller.owner
path = logic.expandPath('//')

class ManageSerial(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
    def run(self):
        master, slave = pty.openpty()
        socatProcess = Popen('socat -d -d pty,raw,echo=0 pty,raw,echo=0,link=' + path + 'mbed', shell=True, stdin=PIPE, stdout=slave, stderr=slave, close_fds=True, preexec_fn=os.setsid)
        stdout = os.fdopen(master)
        
        portOne = stdout.readline()
        portTwo = stdout.readline()
        
        portOne = portOne[portOne.find("/", 20):-1]
        portTwo = portTwo[portTwo.find("/", 20):-1]
        
        port = serial.Serial(portOne, 115200, writeTimeout=0)
        
        print("Set the mbed serial port to: " + portTwo)
        print("          or to the symlink: " + path + "mbed")

        while True:
            if(object['QuitState'] == 1):
                print("Quiting... Please wait...")
                print("Closing port...")
                port.close()
                print("SIGTERM to socat...")
                os.killpg(socatProcess.pid, signal.SIGTERM)
                print("Done.")
                break
            
            if(port.inWaiting() >= 4):
                data = port.read(4)
                
                if(chr(data[3]) == '\n'):
                    if(chr(data[0]) == 'h'):
                        object['DesiredHeading'] = AVNavDecode(data[1], data[2])
                    elif(chr(data[0]) == 'p'):
                        object['DesiredPower'] = AVNavDecode(data[1], data[2])
                    elif(chr(data[0]) == 'd'):
                        object['DesiredDepth'] = AVNavDecode(data[1], data[2])
                    elif(chr(data[0]) == 'r'):
                        object['DesiredDropper'] = AVNavDecode(data[1], data[2])
                    else:
                        pass
                else:
                    pass
            else:
                sendData = AVNavEncode()
                for i in range(0, 11):
                    port.write(sendData[i].encode('ascii', 'replace'))
                time.sleep(0.1)
        logic.endGame()

def AVNavEncode():
    data = [0]*11
    
    currentHeading = object['CurrentHeading']
    currentDepth = object['CurrentDepth']
    currentPower = object['CurrentPower']
    
    data[0] = 'h'
    
    data_temp = currentHeading & 0x3fff
    data[1] = chr((data_temp >> 6) + 0x20)
    data[2] = chr((data_temp & 0x3f) + 0x20)
    
    data[3] = 'd'
    
    data_temp = currentDepth & 0x3fff
    data[4] = chr((data_temp >> 6) + 0x20)
    data[5] = chr((data_temp & 0x3f) + 0x20)
    
    data[6] = 'p'
    
    data_temp = currentPower & 0x3fff
    data[7] = chr((data_temp >> 6) + 0x20)
    data[8] = chr((data_temp & 0x3f) + 0x20)
    
    data[9] = 'k'
    
    data[10] = '\n'
    
    return data

def AVNavDecode(byte1, byte2):
    return int(((byte1 - 0x20) << 6) | (byte2 - 0x20))

print()
print("## AVBotz Submarine Simulator")
print("## Designed to work with Extensible Vehicular Automaton ONLY")
print("## NOTE: Threads & Blender don't normally mix: Press q to quit.")

CameraFront = logic.getCurrentScene().objects["CameraFront"]
CameraDown = logic.getCurrentScene().objects["CameraDown"]

CameraFront.setViewport(0, object['WindowHeight']*2, object['WindowWidth'], object['WindowHeight'])
CameraDown.setViewport(0, object['WindowHeight'], object['WindowWidth'], 0)

CameraFront.useViewport = True
CameraDown.useViewport = True

ManageSerialThread = ManageSerial()
ManageSerialThread.start()