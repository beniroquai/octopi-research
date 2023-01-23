# qt libraries
from qtpy.QtCore import QObject, Signal # type: ignore
from qtpy.QtWidgets import QFrame, QVBoxLayout

from control._def import *

import numpy as np
import pyqtgraph as pg
import cv2
import time
import math

from typing import Optional, List, Union, Tuple

import control.microcontroller as microcontroller
from control.typechecker import TypecheckFunction

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)
    
    thetaPos = Signal(float)
    xyPos = Signal(float,float)
    signal_joystick_button_pressed = Signal()

    @TypecheckFunction
    def __init__(self,
        microcontroller:microcontroller.Microcontroller,
    ):
        QObject.__init__(self)
        self.microcontroller = microcontroller

        self.x_pos_mm = 0
        self.y_pos_mm = 0
        self.z_pos_mm = 0

        self.well_pos_is_stale:bool=True
        self.well_pos_row:int=0
        self.well_pos_column:int=0

        self.z_pos_usteps = 0
        self.theta_pos_rad = 0

        self.enable_joystick_button_action:bool = True

        # to be moved to gui for transparency
        self.microcontroller.set_callback(self.update_pos)

        self.is_in_loading_position:bool=False

    @property
    def plate_type(self)->WellplateFormatPhysical:
        return WELLPLATE_FORMATS[MACHINE_CONFIG.MUTABLE_STATE.WELLPLATE_FORMAT]

    @TypecheckFunction
    def move_x(self,x_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.move_x_usteps(int(x_mm/self.microcontroller.mm_per_ustep_x),wait_for_completion=wait_for_completion,wait_for_stabilization=wait_for_stabilization)

    @TypecheckFunction
    def move_y(self,y_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.move_y_usteps(int(y_mm/self.microcontroller.mm_per_ustep_y),wait_for_completion=wait_for_completion,wait_for_stabilization=wait_for_stabilization)

    @TypecheckFunction
    def move_z(self,z_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        """ this takes 210 ms ?! """
        self.move_z_usteps(int(z_mm/self.microcontroller.mm_per_ustep_z),wait_for_completion=wait_for_completion,wait_for_stabilization=wait_for_stabilization)

    @TypecheckFunction
    def move_x_usteps(self,usteps:int,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.microcontroller.move_x_usteps(usteps)
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_X/1000)

    @TypecheckFunction
    def move_y_usteps(self,usteps:int,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.microcontroller.move_y_usteps(usteps)
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_Y/1000)

    @TypecheckFunction
    def move_z_usteps(self,usteps:int,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        """ this takes 210 ms ?! """

        self.microcontroller.move_z_usteps(usteps)
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_Z/1000)

    @TypecheckFunction
    def move_x_to(self,x_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.microcontroller.move_x_to_usteps(int(x_mm/self.microcontroller.mm_per_ustep_x))
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_X/1000)

    @TypecheckFunction
    def move_y_to(self,y_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.microcontroller.move_y_to_usteps(int(y_mm/self.microcontroller.mm_per_ustep_y))
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_Y/1000)

    @TypecheckFunction
    def move_z_to(self,z_mm:float,wait_for_completion:Optional[Any]=None,wait_for_stabilization:bool=False):
        self.microcontroller.move_z_to_usteps(int(z_mm/self.microcontroller.mm_per_ustep_z))
        if not wait_for_completion is None:
            self.microcontroller.wait_till_operation_is_completed(**wait_for_completion)
        if wait_for_stabilization:
            time.sleep(MACHINE_CONFIG.SCAN_STABILIZATION_TIME_MS_Z/1000)

    
    def move_by_mm(self,x_mm:Optional[float]=None,y_mm:Optional[float]=None,z_mm:Optional[float]=None,row:Optional[int]=None,column:Optional[int]=None):
        if not x_mm is None:
            self.move_x(x_mm,wait_for_completion={},wait_for_stabilization=y_mm is None)
        if not y_mm is None:
            self.move_y(y_mm,wait_for_completion={},wait_for_stabilization=True)
        if not z_mm is None:
            self.move_y(y_mm,wait_for_completion={},wait_for_stabilization=True)

        self.well_pos_is_stale=not(x_mm is None and y_mm is None)

        if not row is None:
            self.well_pos_row=row
        if not column is None:
            self.well_pos_column=column

        self.well_pos_is_stale=not(column is None and row is None)

    def move_to_mm(self,x_mm:Optional[float]=None,y_mm:Optional[float]=None,z_mm:Optional[float]=None,row:Optional[int]=None,column:Optional[int]=None):
        if not x_mm is None:
            self.move_x_to(x_mm,wait_for_completion={},wait_for_stabilization=y_mm is None)
        if not y_mm is None:
            self.move_y_to(y_mm,wait_for_completion={},wait_for_stabilization=True)
        if not z_mm is None:
            self.move_y_to(y_mm,wait_for_completion={},wait_for_stabilization=True)

        self.well_pos_is_stale=not(x_mm is None and y_mm is None)

        if not row is None:
            self.well_pos_row=row
        if not column is None:
            self.well_pos_column=column

        self.well_pos_is_stale=not(column is None and row is None)

    @TypecheckFunction
    def update_pos(self,microcontroller:microcontroller.Microcontroller):
        # get position from the microcontroller
        x_pos, y_pos, z_pos, theta_pos = microcontroller.get_pos()
        self.z_pos_usteps = z_pos
        
        # calculate position in mm or rad
        if MACHINE_CONFIG.USE_ENCODER_X:
            self.x_pos_mm = x_pos*MACHINE_CONFIG.ENCODER_POS_SIGN_X*MACHINE_CONFIG.ENCODER_STEP_SIZE_X_MM
        else:
            self.x_pos_mm = x_pos*MACHINE_CONFIG.STAGE_POS_SIGN_X*self.microcontroller.mm_per_ustep_x

        if MACHINE_CONFIG.USE_ENCODER_Y:
            self.y_pos_mm = y_pos*MACHINE_CONFIG.ENCODER_POS_SIGN_Y*MACHINE_CONFIG.ENCODER_STEP_SIZE_Y_MM
        else:
            self.y_pos_mm = y_pos*MACHINE_CONFIG.STAGE_POS_SIGN_Y*self.microcontroller.mm_per_ustep_y

        if MACHINE_CONFIG.USE_ENCODER_Z:
            self.z_pos_mm = z_pos*MACHINE_CONFIG.ENCODER_POS_SIGN_Z*MACHINE_CONFIG.ENCODER_STEP_SIZE_Z_MM
        else:
            self.z_pos_mm = z_pos*MACHINE_CONFIG.STAGE_POS_SIGN_Z*self.microcontroller.mm_per_ustep_z

        if MACHINE_CONFIG.USE_ENCODER_THETA:
            self.theta_pos_rad = theta_pos*MACHINE_CONFIG.ENCODER_POS_SIGN_THETA*MACHINE_CONFIG.ENCODER_STEP_SIZE_THETA
        else:
            self.theta_pos_rad = theta_pos*MACHINE_CONFIG.STAGE_POS_SIGN_THETA*(2*math.pi/(MACHINE_CONFIG.MICROSTEPPING_DEFAULT_THETA*MACHINE_CONFIG.FULLSTEPS_PER_REV_THETA))

        # emit the updated position
        self.xPos.emit(self.x_pos_mm)
        self.yPos.emit(self.y_pos_mm)
        self.zPos.emit(self.z_pos_mm*1000)
        self.thetaPos.emit(self.theta_pos_rad*360/(2*math.pi))
        self.xyPos.emit(self.x_pos_mm,self.y_pos_mm)

        if microcontroller.signal_joystick_button_pressed_event:
            if self.enable_joystick_button_action:
                self.signal_joystick_button_pressed.emit()
            print('joystick button pressed')
            microcontroller.signal_joystick_button_pressed_event = False

    def home_x(self):
        self.microcontroller.home_x()

    def home_y(self):
        self.microcontroller.home_y()

    def home_z(self):
        self.microcontroller.home_z()

    def home_theta(self):
        self.microcontroller.home_theta()

    def home_xy(self):
        self.microcontroller.home_xy()

    def zero_x(self):
        self.microcontroller.zero_x()

    def zero_y(self):
        self.microcontroller.zero_y()

    def zero_z(self):
        self.microcontroller.zero_z()

    def zero_theta(self):
        self.microcontroller.zero_theta()

    def loading_position_enter(self,home_x:bool=True,home_y:bool=True,home_z:bool=True):
        assert not self.is_in_loading_position
        if home_z:
			# retract the objective
            self.home_z()
			# wait for the operation to finish
            self.microcontroller.wait_till_operation_is_completed(10, time_step=0.005, timeout_msg='z homing timeout, the program will exit')

            self.is_in_loading_position=True

            print('homing - objective retracted')

            if home_z and home_y and home_x:
                # for the new design, need to home y before home x; x also needs to be at > + 10 mm when homing y
                self.move_x(12.0)
                self.microcontroller.wait_till_operation_is_completed(10, time_step=0.005, timeout_msg='x moving timeout, the program will exit')
                
                self.home_y()
                self.microcontroller.wait_till_operation_is_completed(10, time_step=0.005, timeout_msg='y homing timeout, the program will exit')
                
                self.home_x()
                self.microcontroller.wait_till_operation_is_completed(10, time_step=0.005, timeout_msg='x homing timeout, the program will exit')

                print("homing - in loading position")

    def loading_position_leave(self,home_x:bool=True,home_y:bool=True,home_z:bool=True):
        if home_z:
            assert self.is_in_loading_position

            if home_z and home_y and home_x:
                # move by (from home to) (20 mm, 20 mm)
                self.move_x(x_mm=20.0,wait_for_completion={'timeout_limit_s':10, 'time_step':0.005})
                self.move_y(y_mm=20.0,wait_for_completion={'timeout_limit_s':10, 'time_step':0.005})
            
                self.set_x_limit_pos_mm(MACHINE_CONFIG.SOFTWARE_POS_LIMIT.X_POSITIVE)
                self.set_x_limit_neg_mm(MACHINE_CONFIG.SOFTWARE_POS_LIMIT.X_NEGATIVE)
                self.set_y_limit_pos_mm(MACHINE_CONFIG.SOFTWARE_POS_LIMIT.Y_POSITIVE)
                self.set_y_limit_neg_mm(MACHINE_CONFIG.SOFTWARE_POS_LIMIT.Y_NEGATIVE)
                self.set_z_limit_pos_mm(MACHINE_CONFIG.SOFTWARE_POS_LIMIT.Z_POSITIVE)
                
                print("homing - left loading position")

			# move the objective back
            self.move_z(MACHINE_CONFIG.DEFAULT_Z_POS_MM)
			# wait for the operation to finish
            self.microcontroller.wait_till_operation_is_completed(10, time_step=0.005, timeout_msg='z return timeout, the program will exit')

            self.is_in_loading_position=False

            print("homing - objective raised")


    def home(self,home_x:bool=True,home_y:bool=True,home_z:bool=True):
        self.loading_position_enter(home_x,home_y,home_z)
        self.loading_position_leave(home_x,home_y,home_z)

    def set_x_limit_pos_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_x)
        limit_code=LIMIT_CODE.X_POSITIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X > 0 else LIMIT_CODE.X_NEGATIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)

    def set_x_limit_neg_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_x)
        limit_code=LIMIT_CODE.X_NEGATIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X > 0 else LIMIT_CODE.X_POSITIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_X

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)

    def set_y_limit_pos_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_y)
        limit_code=LIMIT_CODE.Y_POSITIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y > 0 else LIMIT_CODE.Y_NEGATIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)

    def set_y_limit_neg_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_y)
        limit_code=LIMIT_CODE.Y_NEGATIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y > 0 else LIMIT_CODE.Y_POSITIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Y

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)

    def set_z_limit_pos_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_z)
        limit_code=LIMIT_CODE.Z_POSITIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z > 0 else LIMIT_CODE.Z_NEGATIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)

    def set_z_limit_neg_mm(self,value_mm):
        u_steps=int(value_mm/self.microcontroller.mm_per_ustep_z)
        limit_code=LIMIT_CODE.Z_NEGATIVE if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z > 0 else LIMIT_CODE.Z_POSITIVE
        u_steps_factor=1 if MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z > 0 else MACHINE_CONFIG.STAGE_MOVEMENT_SIGN_Z

        self.microcontroller.set_lim(limit_code,u_steps_factor*u_steps)
