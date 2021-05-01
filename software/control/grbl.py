import time
import serial
import numpy as np
import fnmatch
import re
import concurrent.futures
import concurrent
import threading
import queue

'''
All available GRBL Codes:
$0 = 10    (Step pulse time, microseconds)
$1 = 1    (Step idle delay, milliseconds)
$2 = 0    (Step pulse invert, mask)
$3 = 0    (Step direction invert, mask)
$4 = 0    (Invert step enable pin, boolean)
$5 = 0    (Invert limit pins, boolean)
$6 = 0    (Invert probe pin, boolean)
$10 = 1    (Status report options, mask)
$11 = 0.010    (Junction deviation, millimeters)
$12 = 0.002    (Arc tolerance, millimeters)
$13 = 0    (Report in inches, boolean)
$20 = 0    (Soft limits enable, boolean)
$21 = 0    (Hard limits enable, boolean)
$22 = 0    (Homing cycle enable, boolean)
$23 = 0    (Homing direction invert, mask)
$24 = 25.000    (Homing locate feed rate, mm/min)
$25 = 500.000    (Homing search seek rate, mm/min)
$26 = 250    (Homing switch debounce delay, milliseconds)
$27 = 1.000    (Homing switch pull-off distance, millimeters)
$30 = 1000    (Maximum spindle speed, RPM)
$31 = 0    (Minimum spindle speed, RPM)
$32 = 0    (Laser-mode enable, boolean)
$100 = 640.000    (X-axis travel resolution, step/mm)
$101 = 640.000    (Y-axis travel resolution, step/mm)
$102 = 100.000    (Z-axis travel resolution, step/mm)
$110 = 500.000    (X-axis maximum rate, mm/min)
$111 = 500.000    (Y-axis maximum rate, mm/min)
$112 = 500.000    (Z-axis maximum rate, mm/min)
$120 = 1.000    (X-axis acceleration, mm/sec^2)
$121 = 1.000    (Y-axis acceleration, mm/sec^2)
$122 = 1000.000    (Z-axis acceleration, mm/sec^2)
$130 = 200.000    (X-axis maximum travel, millimeters)
$131 = 200.000    (Y-axis maximum travel, millimeters)
$132 = 200.000    (Z-axis maximum travel, millimeters)


For the 28byj motor:
http://www.robotmaker.eu/ROBOTmaker/arduino-control-of-stepper-morot
Connecting 28BYJ-48 directly to Arduino UNO CNC Shield using GRBL:
1 ->Pink
2 -> Orange
3 -> Red
4 ->Blue

Commands: for Z-axis
    $102=32768      # Z Axis steps/mm. 32768 is 8*4096.which will give you 1 rev per millimeter, which needs to be adjusted depending on your own belt or geared setup accordingly. 
    $112 = 100.     # Z Axis maximim velocity (mm/min) 
    $122 = 20       # Z-Axis Acceleration (mm/sec2) 
'''

class grblboard:
    is_debug = False                # flag to switch on/off debugging messages
    is_homing_in_progress = False   # state if homing is in progress

    currentposition = (0,0,0) # XYZ
    zero_coordinates = currentposition
    backlash = 0

    # 
    steps_per_mm_x = 640 # how many revolution per fiven distnace => steps/mm
    steps_per_mm_y = 640 # Steps-per-Revolution*microsteps/mm per revolution
    steps_per_mm_z = 64 # 320 # in our case: 200*16*2mm= 6400
    steps_per_mm_z = 3276.8

    # accellation for motors
    accel_x = 20
    accel_y = 20
    accel_z = 10

    maxspeed_x = 200.
    maxspeed_y = 300.
    maxspeed_z = 30.

    is_moving = False
    is_sending = False

    t_idle_until_release = 5 # wait until the current to block motors is released (necessary for disruption free scanning)
    idletime = 25 # 255 # means infinity wait time
    is_idling = False # check if idling 
    is_motor_release = True
    wait_loop_factor = 20

    board = 'GRBL'
    firmware = 'DEFAULT'
    position = [0,0,0] # xyz
    speed = 200
    speed_z = 50

    laser_intensity = 0
    led_state = 0

    cmd_queue = None 
    rec_queue = None
    

    def __init__(self, serialport = "/dev/ttyUSB0",
                 currentposition=currentposition, backlash=backlash, is_homing=True):

        self.backlash = backlash
        self.is_homing = is_homing


        # initialize the thread/queue for serial communication
        self.cmd_queue = queue.Queue()
        self.serial_thread_write = threading.Thread(target=self.sendgrbl_thread, args=(self.cmd_queue,))
        self.serial_thread_write.setDaemon(True)
        self.serial_thread_write.start()

        print('Initializing XYZ-stepper')
        try:
            self.serialport = serial.Serial(serialport,115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=1, writeTimeout=0) #timeout=1) # Open grbl serial port
        except:
            available_ports = self.auto_detect_serial_unix()
            self.serialport = serial.Serial(available_ports[0], 115200,timeout=1)

        # Thread for receiving serial events
        self.rev_queue = queue.Queue()
        self._buf_receive = ""
        self._do_receive = False
        self.serialport.flushInput()
        self.serialport.flushOutput()
        self._do_receive = True

        self.serial_thread_read = threading.Thread(target=self._receiving)
        self.serial_thread_read.start()

        self.initserial()

    def close(self):
        """ CLose Serial Connection"""
        self._do_receive = False
        self.serial_thread_read.join()
        self.serial_thread_write.join()
        self.serialport.flushInput()
        self.serialport.flushOutput()
        self.serialport.close()        

    def initserial(self):
        """ Initiliazing the serial connection and set home coordinates """
        self.sendgrbl("\r\n\r\n") # Wake up grbl
        time.sleep(2)   # Wait for grbl to initialize 

        self.sendgrbl("$$") # Soft reset
        #self.sendgrbl("$X") # Clear possible errors
        time.sleep(.4)
        self.sendgrbl("$10=2")
        self.sendgrbl("$22=0") # disable homing on startup
        self.sendgrbl("$21=0") # disable hard limits
        self.sendgrbl("$20=0") # disable soft limits
        self.sendgrbl("$27=1.000") # Homing Pull-off (mm)
        self.sendgrbl("$3=1") # inverse Direction of X motor
        self.resethome()                # reset the boards coordinates to 0,0,0
        self.setphysicalcoords()        # turns per mm
        self.set_position_poll()
        self.currentposition = self.getcurrentpos()
        self.zero_coordinates = self.currentposition
        self.set_units()                # mm or inches?
        self.setaccellaration()         # limit maximum accellaration
        self.setspeed(self.maxspeed_x, self.maxspeed_y, self.maxspeed_z)
        self.setideltime()
        self.set_laser_intensity(0)
        self.set_led(0)
        if self.is_homing:
            self.go_home()
            #self.go_home_z(offsetz=-8)

    def set_position_poll(self):
            # assign a thread to periodically pull the position from the GRBL Board
            print("Setting up the position poll thread")
            threading.Thread(target=self.poll_position_periodically, args=(.1,)).start()
        
    def poll_position_periodically(self, delay=.2):
        while True:
            self.sendgrbl("?")
            time.sleep(delay)

    def zero_position(self):
        self.resethome()
        self.currentposition = self.getcurrentpos()

    def auto_detect_serial_unix(self, preferred_list=['*']):
        '''try to auto-detect serial ports on posix based OS'''
        import glob
        glist = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
        ret = []

        # try preferred ones first
        for d in glist:
            for preferred in preferred_list:
                if fnmatch.fnmatch(d, preferred):
                    #ret.append(SerialPort(d))
                    ret.append(d)
        if len(ret) > 0:
            return ret
        # now the rest
        for d in glist:
            #ret.append(SerialPort(d))
            ret.append(d)
        return ret

    def setideltime(self,idletime=None):
        if idletime is not None:
            self.idletime = idletime
        self.sendgrbl('$1='+str(self.idletime)) # default: 314.961 #,x step/mm) 

    def setspeed(self, maxspeed_x=None, maxspeed_y=None, maxspeed_z=None):
        if maxspeed_x is not None:
            self.maxspeed_x = maxspeed_x
        if maxspeed_y is not None:
            self.maxspeed_y = maxspeed_y
        if maxspeed_z is not None:
            self.maxspeed_z = maxspeed_z
        self.sendgrbl('$110='+str(self.maxspeed_x)) # default: 50.000
        self.sendgrbl('$111='+str(self.maxspeed_y)) # default:
        self.sendgrbl('$112='+str(self.maxspeed_z)) # default:

    def getspeed(self):
        return self.maxspeed_x, self.maxspeed_y, self.maxspeed_z

    def setaccellaration(self, accel_x=None, accel_y=None, accel_z=None):
        if accel_x is not None:
            self.accel_x = accel_x
        if accel_y is not None:
            self.accel_y = accel_y
        if accel_z is not None:
            self.accel_z = accel_z
        self.sendgrbl('$120='+str(self.accel_x)) # default: 50.000
        self.sendgrbl('$121='+str(self.accel_y)) # default:
        self.sendgrbl('$122='+str(self.accel_z)) # default:

    def getaccel(self):
        return self.accel_x, self.accel_y, self.accel_z

    def setphysicalcoords(self, steps_per_mm_x=None, steps_per_mm_y=None, steps_per_mm_z=None):
        if steps_per_mm_x is not None:
            self.steps_per_mm_x = steps_per_mm_x
        if steps_per_mm_y is not None:
            self.steps_per_mm_y = steps_per_mm_y
        if steps_per_mm_z is not None:
            self.steps_per_mm_z = steps_per_mm_z        
        self.sendgrbl('$100='+str(self.steps_per_mm_x)) # default: 314.961 #,x step/mm) 
        self.sendgrbl('$101='+str(self.steps_per_mm_y)) # default: 314.961 #,y step/mm) 
        self.sendgrbl('$102='+str(self.steps_per_mm_z)) # default: 314.961 #,z step/mm) 

        
    def go_home_z(self, offsetz=-10):
        self.resethome()
        speed_z_old = self.maxspeed_z
        self.setspeed(self.maxspeed_x, self.maxspeed_y, 200)
        self.move_rel((0,0,offsetz))
        self.move_rel((0,0,3))
        self.setspeed(self.maxspeed_x, self.maxspeed_y, speed_z_old)        
        self.resethome()

    def go_home(self, offsetx=0, offsety=0):
        self.sendgrbl("$$") # Soft reset
        self.sendgrbl("$21=1")
        self.sendgrbl("$22=1")
        self.sendgrbl("$H")
        self.is_homing_in_progress = True
        time_init = time.time()
        while True: #not :
            if (time.time() - time_init)>3 and not self.is_homing_in_progress:
                # wait for at least a few seconds to let the homing command get through - eventually still stuck in the queue
                break
            else:
                # just make sure it's not getting overwritten by the "IDLE"  command
                self.is_homing_in_progress = True
            if abs(time.time()-time_init) > 30: 
                break # if something goes wrong, break after half a minute
        self.sendgrbl("$X")
        self.sendgrbl("$22=0") # disable homing on startup
        self.sendgrbl("$21=0") # disable hard limits
        self.sendgrbl("$20=0") # disable soft limits
        self.setphysicalcoords()
        self.resethome()
        self.move_abs((offsetx,offsety,0))
        self.resethome()


    def setpos(self, x, y, z):
        self.sendgrbl("G10 L20 P1 "+"X"+str(x)+ " Y"+str(y)+" Z"+str(z))
        self.sendgrbl("G10 P0 L20  "+"X"+str(x)+ " Y"+str(y)+" Z"+str(z))
        if self.is_debug: print('my pos is: '+str(self.currentposition))

    def resethome(self):
        self.sendgrbl("G10 L20 P1 X0 Y0 Z0")
        self.sendgrbl("G10 P0 L20 X0 Y0 Z0")
        if self.is_debug: print('myhome is: '+str(self.currentposition))

    def getcurrentpos(self):
        # retrieve the latest position in the stack        
        return (self.currentposition[0], self.currentposition[1], self.currentposition[2])        

    def _receiving(self):
        while self._do_receive == True:
            data = self.serialport.read(1)
            waiting = self.serialport.inWaiting()
            data += self.serialport.read(waiting)
            self._handle_data(data)

    def _handle_data(self, data):
        try:
            asci = data.decode("ascii")
        except UnicodeDecodeError:
            asci = ""
            
        for i in range(0, len(asci)):
            char = asci[i]
            self._buf_receive += char
            # not all received lines are complete (end with \n)
            if char == "\n":
                #self.queue.put(self._buf_receive.strip())
                
                return_message = self._buf_receive.strip()
                if self.is_debug: print("Return_message: "+str(return_message))

                # try reading machine's response:
                if (return_message.find('WPos')>0 or return_message.find('MPos')>0):
                    # try to get the current state (only position for now)
                    poslist = return_message.split('WPos')[-1].split('|')[0].split(',')
                    try:
                        posx, posy, posz = float(poslist[0]),float(poslist[1]),float(poslist[2])
                    except:
                        try:
                            poslist = re.split(',|:',re.split('WPos|MPos', return_message)[-1].split('|')[0])[1:4]
                            posx, posy, posz = float(poslist[0]),float(poslist[1]),float(poslist[2])
                        except:
                            # revert to old position and force flushing the serial once again
                            posx, posy, posz = self.currentposition[0], self.currentposition[1], self.currentposition[2]
                    self.currentposition = (posx, posy, posz)
                if (return_message.find('error:9')>0 or return_message.find('Alarm')>0):
                    self.sendgrbl("$X") # resolve an error / unblock GRBL
                    print("Detected an error")
                if return_message.find('Idle')>0:
                    # The idle command comes once the homing is done
                    self.is_homing_in_progress = False
                
                self._buf_receive = ""


    def sendgrbl_thread(self, q):
        while True:
            mytime = time.time()
            cmd = q.get()
            # if self.is_debug: print("Thread sending: "+cmd+", Thread Size: "+str(q.qsize()))
            if cmd:
                q.task_done()
                if self.is_debug: print("Thread sending: "+cmd)
                self.serialport.write((cmd + '\n').encode())

                if cmd.find("$H")>0:
                    # homing in progress 
                    self.is_homing_in_progress = True

    def sendgrbl(self, cmd="?"):
        if self.is_debug: print("GRBL sending: "+cmd)
        self.cmd_queue.put(cmd)
        return_message = ''
            
        return return_message
      
    def move_rel(self, position=(0,0,0), wait_until_done=False):
        # check if stage is moving or not
        self.go_to(position[0], position[1], position[2], 'rel')
        while wait_until_done:
            if not self.is_moving:
                break
        return ""

    def move_abs(self, position=(0,0,0), wait_until_done=False):
        self.go_to(position[0], position[1], position[2], 'abs')
        while wait_until_done:
            if not self.is_moving:
                break
        return ""

    def set_units(self, is_metric=True):
        if is_metric:
            cmd='$13=0' #  for mm
            self.sendgrbl(cmd)
        else:
            cmd='$13=1' #  for inches
            self.sendgrbl(cmd)


    def go_to(self, pos_x=0, pos_y=0, pos_z=0, mode='abs'):
        # trigger a motion event
        while True:
            #if self.is_debug: print("waiting until stage is free..")
            if not self.is_moving:
                break
        
        # make sure motors don't stop when repeated movements are done
        if not self.is_idling and self.is_motor_release: # don't idle if we don't want to do that
            self.is_idling = True
            self.idling_thread = threading.Thread(target=self.start_idling)
            self.idling_thread.start()

        
        # block the stage
        self.is_moving = True #    

        # do actual movement
        self.moving_thread = threading.Thread(target=self.go_to_thread, args=(pos_x, pos_y, pos_z, mode,))
        self.moving_thread.start()

    def start_idling(self):
        # workaround to not switch off the motors (hence causing a discruption of the system)
        # https://github.com/grbl/grbl/issues/597
        self.setideltime(255)

        plan_to_leave_loop = False
        while True:
            if not self.is_moving and not plan_to_leave_loop:
                plan_to_leave_loop = True
                time_over = time.time()
            
            if plan_to_leave_loop:
                time.sleep(.1)
                if(time.time()-time_over)>self.t_idle_until_release:
                    break
            
            if self.is_moving:
                plan_to_leave_loop = False
        

        self.setideltime(25)
        # odd hackaround, otherwise it won't apply the change 
        #self.moving_thread = threading.Thread(target=self.go_to_thread, args=(0.01, 0, 0, 'rel',))
        # apply change in idle time (really weird, $1=2000 doesn't do anything and limits it to 128)
        self.sendgrbl("$1=25")
        time.sleep(0.4)
        self.sendgrbl("$J=G21G91X-.003F100")
        self.sendgrbl("$J=G21G91X.003F100")
        self.is_idling = False




    def go_to_thread(self, pos_x=0, pos_y=0, pos_z=0, mode='abs'):     
        
        # Stream g-code to grbl
        g_dim = "G21"                           # This measures Metric 
        g_dist = "G90"
        ''' obsolote with the mode==rel condition below
        if mode=="abs": 
            g_dist = "G90"                          # G91 is for incremental, G90 is for absolute distance
        elif mode == 'rel':
            g_dist = "G91"
        '''
        g_dist = "G90"                          # G91 is for incremental, G90 is for absolute distance
        
        # steps to go:
        old_position = self.currentposition

        # set speed
        g_speed = "F"+format(self.speed, '.10f')#

        # construct command string
        cmd = g_dim + " " + g_dist 

        # super weird hack around - only to make OFM happy
        
        if mode == 'rel':
            # calculate difference coordinates where you want to go
            togo_x = (old_position[0]+pos_x)
            togo_y = (old_position[1]+pos_y)
            togo_z = (old_position[2]+pos_z)
            wait_loop = np.max((abs(pos_x),abs(pos_y),abs(pos_z)))*self.wait_loop_factor# define a wait threshold until the loop below should break without reaching the final position

        elif mode == 'abs': # remember: We are always in absolute coordinates!!
            togo_x = pos_x 
            togo_y = pos_y
            togo_z = pos_z
            wait_loop = np.max((abs(old_position[0]-pos_x), abs(old_position[1]-pos_y), abs(old_position[2]-pos_z)))*self.wait_loop_factor # define a wait threshold until the loop below should break without reaching the final position

        # construct command string
        min_lim_x = -np.inf
        min_lim_y = -np.inf
        max_lim_x = -np.inf
        max_lim_y = -np.inf
        if not togo_x == old_position[0]:# and togo_x > min_lim_x and togo_x <= max_lim_x:
            if(0):
                if togo_x > min_lim_x:
                    togo_x = min_lim_x
                    print("Reaching Minimum in X")
                if togo_x <= max_lim_x:
                    togo_x = max_lim_x
                    print("Reaching Maximum in X")
            cmd += "X"+str(togo_x)

        if not togo_y == old_position[1]: # and togo_y < min_lim_y and togo_y >= max_lim_y:
            if(0):
                if togo_y > min_lim_y:
                    togo_y = min_lim_y
                    print("Reaching Minimum in y")
                if togo_y <= max_lim_y:
                    togo_y = max_lim_y
                    print("Reaching Maximum in y")
            cmd += "Y"+str(togo_y) 
            
        if not togo_z == old_position[2]: 
            cmd += "Z"+str(togo_z) 
            g_speed = "F"+format(self.speed_z, '.10f')#    

        # finalize string
        cmd += " " + g_speed

        # message from the GRBL board:
        self.sendgrbl(cmd)

        #%% Make sure the destination is reached
        diffx, diffy, diffz = 0,0,0
        t1 = time.time()
        while(True): 
            # read position list
            self.currentposition = self.getcurrentpos()
            pos_x_current, pos_y_current, pos_z_current = self.currentposition[0],self.currentposition[1],self.currentposition[2]
            # onyl check the difference if there is a motion in this direction
            if(cmd.find('X')>=0): diffx = togo_x - pos_x_current
            if(cmd.find('Y')>=0): diffy = togo_y - pos_y_current
            if(cmd.find('Z')>=0): diffz = togo_z - pos_z_current 

            if(abs(diffx)<.01 and 
                abs(diffy)<.01 and
                abs(diffz)<.01):
                break

            if abs(time.time()-t1)>wait_loop:
                print("Not reaching limit yet..")# timelimit reached
                print('DIFF: x: '+str(diffx) + ', y: '+str(diffy) + ', z: '+str(diffz))
                break            
            #%%
        self.is_moving = False


    def set_laser_intensity(self, intensity):
        self.laser_intensity = intensity
        if self.led_state:
            prefix = "M4"
        else:
            prefix = "M3"
        
        cmd =  "G21 G90 " + prefix+" S"+str(self.laser_intensity)
        return self.sendgrbl(cmd)

    def set_led(self, state=1):
        # state is either 1 or 0
        self.led_state = state
        if self.led_state:
            prefix = "M4"
        else:
            prefix = "M3"
        
        cmd =  "G21 G90 " + prefix + " S"+str(self.laser_intensity)
        return self.sendgrbl(cmd)
