import platform
import serial
import serial.tools.list_ports
import time
import numpy as np

from control._def import *
import control.grbldriver as grbldriver

# constants depending on the configuration
PHYS_TO_GRBL_FAC = 1000 #74820/1000 # 1000 steps are 74.2 mm
PHYS_TO_GRBL_FAC_Z = 1000 # 10
# reverse display vs. motion?
DIR_X = 1
DIR_Y = 1
DIR_Z = 1

# add user to the dialout group to avoid the need to use sudo

class Microcontroller():
    def __init__(self,parent=None,serialport="COM5", is_homing=False):
        self.port = serialport
        self.board = grbldriver.GrblDriver(self.port)


        # Initialise backlash storage, used by property setter/getter
        self._backlash = None
        self.settle_time = 0.5  # Default move settle time
        self._position_on_enter = None

        # init the stage
        self.board.write_global_config()
        self.board.write_all_settings()
        self.board.verify_settings()

        #self.board.home()
        self.board.reset_stage()

    def close(self):
        """Cleanly close communication with the stage"""
        if hasattr(self, "board"):
            self.board.close()

    def toggle_LED(self,state):
        self.board.set_led(state)

    def toggle_laser(self,state):
        self.board.set_laser_intensity(state)

    def turn_on_illumination(self):
        self.board.set_led(1)
#        self.board.set_laser_intensity(10)

    def turn_off_illumination(self):
        self.board.set_led(0)
        self.board.set_laser_intensity(0)

    def set_illumination(self,illumination_source,intensity):
        if illumination_source == 13:
            self.board.set_laser_intensity(int(2.55*intensity))
        if illumination_source == 0:
            state = int(intensity>50)
            self.board.set_led(intensity)
        print("Set Illumination: "+str(illumination_source)+" - "+str(intensity))

    def move_x(self,delta):
        self.board.move_rel((delta*PHYS_TO_GRBL_FAC,0,0), blocking=False)

    def move_x_usteps(self,usteps):
        self.board.move_rel((usteps*PHYS_TO_GRBL_FAC,0,0), blocking=False)

    def move_y(self,delta):
        self.board.move_rel((0,delta*PHYS_TO_GRBL_FAC,0), blocking=False)

    def move_y_usteps(self,usteps):
        self.board.move_rel((0,usteps*PHYS_TO_GRBL_FAC,0), blocking=False)

    def move_z(self,delta):
        self.board.move_rel((0,0,delta*PHYS_TO_GRBL_FAC_Z), blocking=False)

    def move_z_usteps(self,usteps):
        self.board.move_rel((0,usteps*PHYS_TO_GRBL_FAC,0), blocking=False)

    def send_command(self,command):
        '''
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
        '''
        self.board.sendgrbl(command)

    def read_received_packet(self):
        # wait to receive data
        pass
        # Don't need that?!
        data = self.board.positions
        return data

    def read_received_packet_nowait(self):
        # wait to receive data
        data = self.board.positions
        return tuple(data)

class Microcontroller_Simulation():
    def __init__(self,parent=None):
        pass

    def close(self):
        pass

    def toggle_LED(self,state):
        pass

    def toggle_laser(self,state):
        pass

    def move_x(self,delta):
        pass

    def move_y(self,delta):
        pass

    def move_z(self,delta):
        pass

    def move_x_usteps(self,usteps):
        pass

    def move_y_usteps(self,usteps):
        pass

    def move_z_usteps(self,usteps):
        pass

    def send_command(self,command):
        pass

    def read_received_packet(self):
        pass

    def read_received_packet_nowait(self):
        return None

    def turn_on_illumination(self):
        pass

    def turn_off_illumination(self):
        pass

    def set_illumination(self,illumination_source,intensity):
        pass
