# set QT_API environment variable
import os 
os.environ["QT_API"] = "pyqt5"
import qtpy

# qt libraries
from qtpy.QtCore import *
from qtpy.QtWidgets import *
from qtpy.QtGui import *

import control.utils as utils
from control._def import *

from queue import Queue
from threading import Thread, Lock
import time
import numpy as np
import pyqtgraph as pg
import cv2
from datetime import datetime

from lxml import etree as ET
from pathlib import Path
import control.utils_config as utils_config

import math
import json
import pandas as pd

import imageio as iio

from typing import Optional, List, Union, Tuple
import widgets

import microcontroller
import camera

class StreamHandler(QObject):

    image_to_display = Signal(np.ndarray)
    packet_image_to_write = Signal(np.ndarray, int, float)
    packet_image_for_tracking = Signal(np.ndarray, int, float)
    signal_new_frame_received = Signal()

    def __init__(self,crop_width=Acquisition.CROP_WIDTH,crop_height=Acquisition.CROP_HEIGHT,display_resolution_scaling=1):
        QObject.__init__(self)
        self.fps_display = 1
        self.fps_save = 1
        self.fps_track = 1
        self.timestamp_last_display = 0
        self.timestamp_last_save = 0
        self.timestamp_last_track = 0

        self.crop_width = crop_width
        self.crop_height = crop_height
        self.display_resolution_scaling = display_resolution_scaling

        self.save_image_flag = False
        self.track_flag = False
        self.handler_busy = False

        # for fps measurement
        self.timestamp_last = 0
        self.counter = 0
        self.fps_real = 0

    def start_recording(self):
        self.save_image_flag = True

    def stop_recording(self):
        self.save_image_flag = False

    def start_tracking(self):
        self.tracking_flag = True

    def stop_tracking(self):
        self.tracking_flag = False

    def set_display_fps(self,fps):
        self.fps_display = fps

    def set_save_fps(self,fps):
        self.fps_save = fps

    def set_crop(self,crop_width,crop_height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_display_resolution_scaling(self, display_resolution_scaling):
        self.display_resolution_scaling = display_resolution_scaling/100
        print(self.display_resolution_scaling)

    def on_new_frame(self, camera):

        if camera.is_live:

            camera.image_locked = True
            self.handler_busy = True
            self.signal_new_frame_received.emit() # self.liveController.turn_off_illumination()

            # measure real fps
            timestamp_now = round(time.time())
            if timestamp_now == self.timestamp_last:
                self.counter = self.counter+1
            else:
                self.timestamp_last = timestamp_now
                self.fps_real = self.counter
                self.counter = 0
                # print('real camera fps is ' + str(self.fps_real))

            # moved down (so that it does not modify the camera.current_frame, which causes minor problems for simulation) - 1/30/2022
            # # rotate and flip - eventually these should be done in the camera
            # camera.current_frame = utils.rotate_and_flip_image(camera.current_frame,rotate_image_angle=camera.rotate_image_angle,flip_image=camera.flip_image)

            # crop image
            image_cropped = utils.crop_image(camera.current_frame,self.crop_width,self.crop_height)
            image_cropped = np.squeeze(image_cropped)
            
            # # rotate and flip - moved up (1/10/2022)
            # image_cropped = utils.rotate_and_flip_image(image_cropped,rotate_image_angle=ROTATE_IMAGE_ANGLE,flip_image=FLIP_IMAGE)
            # added on 1/30/2022
            # @@@ to move to camera 
            image_cropped = utils.rotate_and_flip_image(image_cropped,rotate_image_angle=camera.rotate_image_angle,flip_image=camera.flip_image)

            # send image to display
            time_now = time.time()
            if time_now-self.timestamp_last_display >= 1/self.fps_display:
                # self.image_to_display.emit(cv2.resize(image_cropped,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                self.image_to_display.emit(utils.crop_image(image_cropped,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)))
                self.timestamp_last_display = time_now

            # send image to write
            if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
                if camera.is_color:
                    image_cropped = cv2.cvtColor(image_cropped,cv2.COLOR_RGB2BGR)
                self.packet_image_to_write.emit(image_cropped,camera.frame_ID,camera.timestamp)
                self.timestamp_last_save = time_now

            # send image to track
            if self.track_flag and time_now-self.timestamp_last_track >= 1/self.fps_track:
                # track is a blocking operation - it needs to be
                # @@@ will cropping before emitting the signal lead to speedup?
                self.packet_image_for_tracking.emit(image_cropped,camera.frame_ID,camera.timestamp)
                self.timestamp_last_track = time_now

            self.handler_busy = False
            camera.image_locked = False

    '''
    def on_new_frame_from_simulation(self,image,frame_ID,timestamp):
        # check whether image is a local copy or pointer, if a pointer, needs to prevent the image being modified while this function is being executed
        
        self.handler_busy = True

        # crop image
        image_cropped = utils.crop_image(image,self.crop_width,self.crop_height)

        # send image to display
        time_now = time.time()
        if time_now-self.timestamp_last_display >= 1/self.fps_display:
            self.image_to_display.emit(cv2.resize(image_cropped,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
            self.timestamp_last_display = time_now

        # send image to write
        if self.save_image_flag and time_now-self.timestamp_last_save >= 1/self.fps_save:
            self.packet_image_to_write.emit(image_cropped,frame_ID,timestamp)
            self.timestamp_last_save = time_now

        # send image to track
        if time_now-self.timestamp_last_display >= 1/self.fps_track:
            # track emit
            self.timestamp_last_track = time_now

        self.handler_busy = False
    '''

class ImageSaver(QObject):

    stop_recording = Signal()

    def __init__(self,image_format=Acquisition.IMAGE_FORMAT):
        QObject.__init__(self)
        self.base_path:str = './'
        self.experiment_ID:str = ''
        self.image_format = image_format
        self.max_num_image_per_folder:int = 1000
        self.queue:Queue = Queue(10) # max 10 items in the queue
        self.image_lock:Lock = Lock()
        self.stop_signal_received:bool = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()
        self.counter:int = 0
        self.recording_start_time:float = 0.0
        self.recording_time_limit:float = -1.0

    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image,frame_ID,timestamp] = self.queue.get(timeout=0.1)
                self.image_lock.acquire(True)
                folder_ID = int(self.counter/self.max_num_image_per_folder)
                file_ID = int(self.counter%self.max_num_image_per_folder)
                # create a new folder
                if file_ID == 0:
                    os.mkdir(os.path.join(self.base_path,self.experiment_ID,str(folder_ID)))

                if image.dtype == np.uint16:
                    # need to use tiff when saving 16 bit images
                    saving_path = os.path.join(self.base_path,self.experiment_ID,str(folder_ID),str(file_ID) + '_' + str(frame_ID) + '.tiff')
                    iio.imwrite(saving_path,image)
                else:
                    saving_path = os.path.join(self.base_path,self.experiment_ID,str(folder_ID),str(file_ID) + '_' + str(frame_ID) + '.' + self.image_format)
                    cv2.imwrite(saving_path,image)

                self.counter = self.counter + 1
                self.queue.task_done()
                self.image_lock.release()
            except:
                pass
                            
    def enqueue(self,image,frame_ID,timestamp):
        try:
            self.queue.put_nowait([image,frame_ID,timestamp])
            if ( self.recording_time_limit>0 ) and ( time.time()-self.recording_start_time >= self.recording_time_limit ):
                self.stop_recording.emit()
            # when using self.queue.put(str_), program can be slowed down despite multithreading because of the block and the GIL
        except:
            print('imageSaver queue is full, image discarded')

    def set_base_path(self,path):
        self.base_path = path

    def set_recording_time_limit(self,time_limit):
        self.recording_time_limit = time_limit

    def start_new_experiment(self,experiment_ID:str,add_timestamp:bool=True):
        if add_timestamp:
            # generate unique experiment ID
            self.experiment_ID = experiment_ID + '_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%-S.%f')
        else:
            self.experiment_ID = experiment_ID
        self.recording_start_time = time.time()
        # create a new folder
        try:
            os.mkdir(os.path.join(self.base_path,self.experiment_ID))
            # to do: save configuration
        except:
            pass
        # reset the counter
        self.counter = 0

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()


'''
class ImageSaver_MultiPointAcquisition(QObject):
'''

class ImageDisplay(QObject):

    image_to_display = Signal(np.ndarray)

    def __init__(self):
        QObject.__init__(self)
        self.queue = Queue(10) # max 10 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread = Thread(target=self.process_queue)
        self.thread.start()        
        
    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image,frame_ID,timestamp] = self.queue.get(timeout=0.1)
                self.image_lock.acquire(True)
                self.image_to_display.emit(image)
                self.image_lock.release()
                self.queue.task_done()
            except:
                pass

    # def enqueue(self,image,frame_ID,timestamp):
    def enqueue(self,image):
        try:
            self.queue.put_nowait([image,None,None])
            # when using self.queue.put(str_) instead of try + nowait, program can be slowed down despite multithreading because of the block and the GIL
            pass
        except:
            print('imageDisplay queue is full, image discarded')

    def emit_directly(self,image):
        self.image_to_display.emit(image)

    def close(self):
        self.queue.join()
        self.stop_signal_received = True
        self.thread.join()

class Configuration:
    def __init__(self,mode_id=None,name=None,camera_sn=None,exposure_time=None,analog_gain=None,illumination_source=None,illumination_intensity=None):
        self.id = mode_id
        self.name = name
        self.exposure_time:float = exposure_time
        self.analog_gain:float = analog_gain
        self.illumination_source:int = illumination_source
        self.illumination_intensity:float = illumination_intensity
        self.camera_sn = camera_sn

class ConfigurationManager(QObject):
    def __init__(self,filename=str(Path.home() / "configurations_default.xml")):
        QObject.__init__(self)
        self.config_filename:str = filename
        self.configurations:List[Configuration] = []
        self.read_configurations()
        
    def save_configurations(self):
        self.write_configuration(self.config_filename)

    def write_configuration(self,filename):
        self.config_xml_tree.write(filename, encoding="utf-8", xml_declaration=True, pretty_print=True)

    def read_configurations(self):
        if(os.path.isfile(self.config_filename)==False):
            utils_config.generate_default_configuration(self.config_filename)
        self.config_xml_tree = ET.parse(self.config_filename)
        self.config_xml_tree_root = self.config_xml_tree.getroot()
        self.num_configurations = 0
        for mode in self.config_xml_tree_root.iter('mode'):
            self.num_configurations = self.num_configurations + 1
            self.configurations.append(
                Configuration(
                    mode_id = mode.get('ID'),
                    name = mode.get('Name'),
                    exposure_time = float(mode.get('ExposureTime')),
                    analog_gain = float(mode.get('AnalogGain')),
                    illumination_source = int(mode.get('IlluminationSource')),
                    illumination_intensity = float(mode.get('IlluminationIntensity')),
                    camera_sn = mode.get('CameraSN'))
            )

    def update_configuration(self,configuration_id,attribute_name,new_value):
        list = self.config_xml_tree_root.xpath("//mode[contains(@ID," + "'" + str(configuration_id) + "')]")
        mode_to_update = list[0]
        mode_to_update.set(attribute_name,str(new_value))
        self.save_configurations()


class LiveController(QObject):

    def __init__(self,
        camera:camera.Camera,
        microcontroller:microcontroller.Microcontroller,
        configurationManager:ConfigurationManager,
        control_illumination:bool=True,
        use_internal_timer_for_hardware_trigger:bool=True
    ):
        QObject.__init__(self)
        self.camera = camera
        self.microcontroller = microcontroller
        self.configurationManager:core.ConfigurationManager = configurationManager
        self.currentConfiguration:Optional[Configuration] = None
        self.trigger_mode = TriggerMode.SOFTWARE # @@@ change to None
        self.is_live:bool = False
        self.control_illumination = control_illumination
        self.illumination_on:bool = False
        self.use_internal_timer_for_hardware_trigger = use_internal_timer_for_hardware_trigger # use QTimer vs timer in the MCU

        self.fps_trigger = 1
        self.timer_trigger_interval = (1/self.fps_trigger)*1000

        self.timer_trigger = QTimer()
        self.timer_trigger.setInterval(self.timer_trigger_interval)
        self.timer_trigger.timeout.connect(self.trigger_acquisition)

        self.trigger_ID = -1

        self.fps_real = 0
        self.counter = 0
        self.timestamp_last = 0

        self.display_resolution_scaling = DEFAULT_DISPLAY_CROP/100

    # illumination control
    def turn_on_illumination(self):
        self.microcontroller.turn_on_illumination()
        self.illumination_on = True

    def turn_off_illumination(self):
        self.microcontroller.turn_off_illumination()
        self.illumination_on = False

    def set_illumination(self,illumination_source:int,intensity:int):
        if illumination_source < 10: # LED matrix
            self.microcontroller.set_illumination_led_matrix(illumination_source,r=(intensity/100)*LED_MATRIX_R_FACTOR,g=(intensity/100)*LED_MATRIX_G_FACTOR,b=(intensity/100)*LED_MATRIX_B_FACTOR)
        else:
            self.microcontroller.set_illumination(illumination_source,intensity)

    def start_live(self):
        self.is_live = True
        self.camera.is_live = True
        self.camera.start_streaming()
        if self.trigger_mode == TriggerMode.SOFTWARE or ( self.trigger_mode == TriggerMode.HARDWARE and self.use_internal_timer_for_hardware_trigger ):
            self._start_triggerred_acquisition()

    def stop_live(self):
        if self.is_live:
            self.is_live = False
            self.camera.is_live = False
            if self.trigger_mode == TriggerMode.SOFTWARE:
                self._stop_triggerred_acquisition()
            # self.camera.stop_streaming() # 20210113 this line seems to cause problems when using af with multipoint
            if self.trigger_mode == TriggerMode.CONTINUOUS:
                self.camera.stop_streaming()
            if ( self.trigger_mode == TriggerMode.SOFTWARE ) or ( self.trigger_mode == TriggerMode.HARDWARE and self.use_internal_timer_for_hardware_trigger ):
                self._stop_triggerred_acquisition()
            if self.control_illumination:
                self.turn_off_illumination()

    # software trigger related
    def trigger_acquisition(self):
        if self.trigger_mode == TriggerMode.SOFTWARE:
            if self.control_illumination and self.illumination_on == False:
                self.turn_on_illumination()
            self.trigger_ID = self.trigger_ID + 1
            self.camera.send_trigger()
            # measure real fps
            timestamp_now = round(time.time())
            if timestamp_now == self.timestamp_last:
                self.counter = self.counter+1
            else:
                self.timestamp_last = timestamp_now
                self.fps_real = self.counter
                self.counter = 0
                # print('real trigger fps is ' + str(self.fps_real))
        elif self.trigger_mode == TriggerMode.HARDWARE:
            self.trigger_ID = self.trigger_ID + 1
            self.microcontroller.send_hardware_trigger(control_illumination=True,illumination_on_time_us=self.camera.exposure_time*1000)

    def _start_triggerred_acquisition(self):
        self.timer_trigger.start()

    def _set_trigger_fps(self,fps_trigger):
        self.fps_trigger = fps_trigger
        self.timer_trigger_interval = (1/self.fps_trigger)*1000
        self.timer_trigger.setInterval(self.timer_trigger_interval)

    def _stop_triggerred_acquisition(self):
        self.timer_trigger.stop()

    # trigger mode and settings
    def set_trigger_mode(self,mode:TriggerMode):
        if mode == TriggerMode.SOFTWARE:
            if self.is_live and ( self.trigger_mode == TriggerMode.HARDWARE and self.use_internal_timer_for_hardware_trigger ):
                self._stop_triggerred_acquisition()
            self.camera.set_software_triggered_acquisition()
            if self.is_live:
                self._start_triggerred_acquisition()
        if mode == TriggerMode.HARDWARE:
            if self.trigger_mode == TriggerMode.SOFTWARE and self.is_live:
                self._stop_triggerred_acquisition()
            # self.camera.reset_camera_acquisition_counter()
            self.camera.set_hardware_triggered_acquisition()
            self.microcontroller.set_strobe_delay_us(self.camera.strobe_delay_us)
            if self.is_live and self.use_internal_timer_for_hardware_trigger:
                self._start_triggerred_acquisition()
        if mode == TriggerMode.CONTINUOUS: 
            if ( self.trigger_mode == TriggerMode.SOFTWARE ) or ( self.trigger_mode == TriggerMode.HARDWARE and self.use_internal_timer_for_hardware_trigger ):
                self._stop_triggerred_acquisition()
            self.camera.set_continuous_acquisition()
        self.trigger_mode = mode

    def set_trigger_fps(self,fps):
        if ( self.trigger_mode == TriggerMode.SOFTWARE ) or ( self.trigger_mode == TriggerMode.HARDWARE and self.use_internal_timer_for_hardware_trigger ):
            self._set_trigger_fps(fps)
    
    # set microscope mode
    # @@@ to do: change softwareTriggerGenerator to TriggerGeneratror
    def set_microscope_mode(self,configuration:Configuration):

        self.currentConfiguration = configuration
        print("setting microscope mode to " + self.currentConfiguration.name)
        
        # temporarily stop live while changing mode
        if self.is_live is True:
            self.timer_trigger.stop()
            if self.control_illumination:
                self.turn_off_illumination()

        # set camera exposure time and analog gain
        self.camera.set_exposure_time(self.currentConfiguration.exposure_time)
        self.camera.set_analog_gain(self.currentConfiguration.analog_gain)

        # set illumination
        if self.control_illumination:
            self.set_illumination(self.currentConfiguration.illumination_source,self.currentConfiguration.illumination_intensity)

        # restart live 
        if self.is_live is True:
            if self.control_illumination:
                self.turn_on_illumination()
            self.timer_trigger.start()

    def get_trigger_mode(self):
        return self.trigger_mode

    # slot
    def on_new_frame(self):
        if self.fps_trigger <= 5:
            if self.control_illumination and self.illumination_on == True:
                self.turn_off_illumination()

    def set_display_resolution_scaling(self, display_resolution_scaling):
        self.display_resolution_scaling = display_resolution_scaling/100

class NavigationController(QObject):

    xPos = Signal(float)
    yPos = Signal(float)
    zPos = Signal(float)
    thetaPos = Signal(float)
    xyPos = Signal(float,float)
    signal_joystick_button_pressed = Signal()

    def __init__(self,microcontroller:microcontroller.Microcontroller):
        QObject.__init__(self)
        self.microcontroller = microcontroller
        self.x_pos_mm = 0
        self.y_pos_mm = 0
        self.z_pos_mm = 0
        self.z_pos = 0
        self.theta_pos_rad = 0
        self.x_microstepping = MICROSTEPPING_DEFAULT_X
        self.y_microstepping = MICROSTEPPING_DEFAULT_Y
        self.z_microstepping = MICROSTEPPING_DEFAULT_Z
        self.theta_microstepping = MICROSTEPPING_DEFAULT_THETA
        self.enable_joystick_button_action:bool = True

        # to be moved to gui for transparency
        self.microcontroller.set_callback(self.update_pos)

        # self.timer_read_pos = QTimer()
        # self.timer_read_pos.setInterval(PosUpdate.INTERVAL_MS)
        # self.timer_read_pos.timeout.connect(self.update_pos)
        # self.timer_read_pos.start()

    def move_x(self,delta):
        self.microcontroller.move_x_usteps(int(delta/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def move_y(self,delta):
        self.microcontroller.move_y_usteps(int(delta/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def move_z(self,delta):
        self.microcontroller.move_z_usteps(int(delta/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))

    def move_x_to(self,delta):
        self.microcontroller.move_x_to_usteps(int(delta/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def move_y_to(self,delta):
        self.microcontroller.move_y_to_usteps(int(delta/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def move_z_to(self,delta):
        self.microcontroller.move_z_to_usteps(int(delta/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))

    def move_x_usteps(self,usteps):
        self.microcontroller.move_x_usteps(usteps)

    def move_y_usteps(self,usteps):
        self.microcontroller.move_y_usteps(usteps)

    def move_z_usteps(self,usteps):
        self.microcontroller.move_z_usteps(usteps)

    def update_pos(self,microcontroller):
        # get position from the microcontroller
        x_pos, y_pos, z_pos, theta_pos = microcontroller.get_pos()
        self.z_pos = z_pos
        # calculate position in mm or rad
        if USE_ENCODER_X:
            self.x_pos_mm = x_pos*ENCODER_POS_SIGN_X*ENCODER_STEP_SIZE_X_MM
        else:
            self.x_pos_mm = x_pos*STAGE_POS_SIGN_X*(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))
        if USE_ENCODER_Y:
            self.y_pos_mm = y_pos*ENCODER_POS_SIGN_Y*ENCODER_STEP_SIZE_Y_MM
        else:
            self.y_pos_mm = y_pos*STAGE_POS_SIGN_Y*(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))
        if USE_ENCODER_Z:
            self.z_pos_mm = z_pos*ENCODER_POS_SIGN_Z*ENCODER_STEP_SIZE_Z_MM
        else:
            self.z_pos_mm = z_pos*STAGE_POS_SIGN_Z*(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))
        if USE_ENCODER_THETA:
            self.theta_pos_rad = theta_pos*ENCODER_POS_SIGN_THETA*ENCODER_STEP_SIZE_THETA
        else:
            self.theta_pos_rad = theta_pos*STAGE_POS_SIGN_THETA*(2*math.pi/(self.theta_microstepping*FULLSTEPS_PER_REV_THETA))
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
        self.microcontroller.zero_tehta()

    def home(self):
        pass

    def set_x_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_X > 0:
            self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,STAGE_MOVEMENT_SIGN_X*int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def set_x_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_X > 0:
            self.microcontroller.set_lim(LIMIT_CODE.X_NEGATIVE,int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.X_POSITIVE,STAGE_MOVEMENT_SIGN_X*int(value_mm/(SCREW_PITCH_X_MM/(self.x_microstepping*FULLSTEPS_PER_REV_X))))

    def set_y_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Y > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,STAGE_MOVEMENT_SIGN_Y*int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def set_y_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Y > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Y_NEGATIVE,int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Y_POSITIVE,STAGE_MOVEMENT_SIGN_Y*int(value_mm/(SCREW_PITCH_Y_MM/(self.y_microstepping*FULLSTEPS_PER_REV_Y))))

    def set_z_limit_pos_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Z > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,STAGE_MOVEMENT_SIGN_Z*int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))

    def set_z_limit_neg_mm(self,value_mm):
        if STAGE_MOVEMENT_SIGN_Z > 0:
            self.microcontroller.set_lim(LIMIT_CODE.Z_NEGATIVE,int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
        else:
            self.microcontroller.set_lim(LIMIT_CODE.Z_POSITIVE,STAGE_MOVEMENT_SIGN_Z*int(value_mm/(SCREW_PITCH_Z_MM/(self.z_microstepping*FULLSTEPS_PER_REV_Z))))
    
    def move_to(self,x_mm,y_mm):
        self.move_x_to(x_mm)
        self.move_y_to(y_mm)

class SlidePositionControlWorker(QObject):
    
    finished = Signal()
    signal_stop_live = Signal()
    signal_resume_live = Signal()

    def __init__(self,slidePositionController:core.SlidePositionController,home_x_and_y_separately:bool=False):
        QObject.__init__(self)
        self.slidePositionController = slidePositionController
        self.navigationController = slidePositionController.navigationController
        self.microcontroller = self.navigationController.microcontroller
        self.liveController = self.slidePositionController.liveController
        self.home_x_and_y_separately = home_x_and_y_separately

    def wait_till_operation_is_completed(self,timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)
            if time.time() - timestamp_start > SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S:
                print('Error - slide position switching timeout, the program will exit')
                self.navigationController.move_x(0)
                self.navigationController.move_y(0)
                exit()

    def move_to_slide_loading_position(self):
        was_live = self.liveController.is_live
        if was_live:
            self.signal_stop_live.emit()
        if self.slidePositionController.homing_done == False or SLIDE_POTISION_SWITCHING_HOME_EVERYTIME:
            if self.home_x_and_y_separately:
                timestamp_start = time.time()
                self.navigationController.home_x()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_x()
                self.navigationController.move_x(SLIDE_POSITION.LOADING_X_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.home_y()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_y()
                self.navigationController.move_y(SLIDE_POSITION.LOADING_Y_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            else:
                timestamp_start = time.time()
                self.navigationController.home_xy()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_x()
                self.navigationController.zero_y()
                self.navigationController.move_x(SLIDE_POSITION.LOADING_X_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.move_y(SLIDE_POSITION.LOADING_Y_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.slidePositionController.homing_done = True
        else:
            timestamp_start = time.time()
            self.navigationController.move_y(SLIDE_POSITION.LOADING_Y_MM-self.navigationController.y_pos_mm)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.move_x(SLIDE_POSITION.LOADING_X_MM-self.navigationController.x_pos_mm)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        if was_live:
            self.signal_resume_live.emit()
        self.slidePositionController.slide_loading_position_reached = True
        self.finished.emit()

    def move_to_slide_scanning_position(self):
        was_live = self.liveController.is_live
        if was_live:
            self.signal_stop_live.emit()
        if self.slidePositionController.homing_done == False or SLIDE_POTISION_SWITCHING_HOME_EVERYTIME:
            if self.home_x_and_y_separately:
                timestamp_start = time.time()
                self.navigationController.home_y()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_y()
                self.navigationController.move_y(SLIDE_POSITION.SCANNING_Y_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.home_x()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_x()
                self.navigationController.move_x(SLIDE_POSITION.SCANNING_X_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            else:
                timestamp_start = time.time()
                self.navigationController.home_xy()
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.zero_x()
                self.navigationController.zero_y()
                self.navigationController.move_y(SLIDE_POSITION.SCANNING_Y_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
                self.navigationController.move_x(SLIDE_POSITION.SCANNING_X_MM)
                self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.slidePositionController.homing_done = True
        else:
            timestamp_start = time.time()
            self.navigationController.move_y(SLIDE_POSITION.SCANNING_Y_MM-self.navigationController.y_pos_mm)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
            self.navigationController.move_x(SLIDE_POSITION.SCANNING_X_MM-self.navigationController.x_pos_mm)
            self.wait_till_operation_is_completed(timestamp_start, SLIDE_POTISION_SWITCHING_TIMEOUT_LIMIT_S)
        if was_live:
            self.signal_resume_live.emit()
        self.slidePositionController.slide_scanning_position_reached = True
        self.finished.emit()

class SlidePositionController(QObject):

    signal_slide_loading_position_reached = Signal()
    signal_slide_scanning_position_reached = Signal()
    signal_clear_slide = Signal()

    def __init__(self,navigationController,liveController):
        QObject.__init__(self)
        self.navigationController:core.NavigationController = navigationController
        self.liveController:core.LiveController = liveController
        self.slide_loading_position_reached:bool = False
        self.slide_scanning_position_reached:bool = False
        self.homing_done:bool = False

    def move_to_slide_loading_position(self):
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.slidePositionControlWorker = SlidePositionControlWorker(self)
        # move the worker to the thread
        self.slidePositionControlWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.slidePositionControlWorker.move_to_slide_loading_position)
        self.slidePositionControlWorker.signal_stop_live.connect(self.slot_stop_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.signal_resume_live.connect(self.slot_resume_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.finished.connect(self.signal_slide_loading_position_reached.emit)
        self.slidePositionControlWorker.finished.connect(self.slidePositionControlWorker.deleteLater)
        self.slidePositionControlWorker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()

    def move_to_slide_scanning_position(self):
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.slidePositionControlWorker = SlidePositionControlWorker(self)
        # move the worker to the thread
        self.slidePositionControlWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.slidePositionControlWorker.move_to_slide_scanning_position)
        self.slidePositionControlWorker.signal_stop_live.connect(self.slot_stop_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.signal_resume_live.connect(self.slot_resume_live,type=Qt.BlockingQueuedConnection)
        self.slidePositionControlWorker.finished.connect(self.signal_slide_scanning_position_reached.emit)
        self.slidePositionControlWorker.finished.connect(self.slidePositionControlWorker.deleteLater)
        self.slidePositionControlWorker.finished.connect(self.thread.quit)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()
        self.signal_clear_slide.emit()

    def slot_stop_live(self):
        self.liveController.stop_live()

    def slot_resume_live(self):
        self.liveController.start_live()

class AutofocusWorker(QObject):

    finished = Signal()
    image_to_display = Signal(np.ndarray)
    # signal_current_configuration = Signal(Configuration)

    def __init__(self,autofocusController:core.AutoFocusController):
        QObject.__init__(self)
        self.autofocusController = autofocusController

        self.camera = self.autofocusController.camera
        self.microcontroller = self.autofocusController.navigationController.microcontroller
        self.navigationController = self.autofocusController.navigationController
        self.liveController = self.autofocusController.liveController

        self.N = self.autofocusController.N
        self.deltaZ = self.autofocusController.deltaZ
        self.deltaZ_usteps = self.autofocusController.deltaZ_usteps
        
        self.crop_width = self.autofocusController.crop_width
        self.crop_height = self.autofocusController.crop_height

    def run(self):
        self.run_autofocus()
        self.finished.emit()

    def wait_till_operation_is_completed(self):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)

    def run_autofocus(self):
        # @@@ to add: increase gain, decrease exposure time
        # @@@ can move the execution into a thread - done 08/21/2021
        focus_measure_vs_z = [0]*self.N
        focus_measure_max = 0

        z_af_offset_usteps = self.deltaZ_usteps*round(self.N/2)
        # self.navigationController.move_z_usteps(-z_af_offset_usteps) # combine with the back and forth maneuver below
        # self.wait_till_operation_is_completed()

        # maneuver for achiving uniform step size and repeatability when using open-loop control
        # can be moved to the firmware
        _usteps_to_clear_backlash = max(160,20*self.navigationController.z_microstepping)
        self.navigationController.move_z_usteps(-_usteps_to_clear_backlash-z_af_offset_usteps)
        self.wait_till_operation_is_completed()
        self.navigationController.move_z_usteps(_usteps_to_clear_backlash)
        self.wait_till_operation_is_completed()

        steps_moved = 0
        for i in range(self.N):
            self.navigationController.move_z_usteps(self.deltaZ_usteps)
            self.wait_till_operation_is_completed()
            steps_moved = steps_moved + 1
            # trigger acquisition (including turning on the illumination)
            if self.liveController.trigger_mode == TriggerMode.SOFTWARE:
                self.liveController.turn_on_illumination()
                self.wait_till_operation_is_completed()
                self.camera.send_trigger()
            elif self.liveController.trigger_mode == TriggerMode.HARDWARE:
                self.microcontroller.send_hardware_trigger(control_illumination=True,illumination_on_time_us=self.camera.exposure_time*1000)
            # read camera frame
            image = self.camera.read_frame()
            # tunr of the illumination if using software trigger
            if self.liveController.trigger_mode == TriggerMode.SOFTWARE:
                self.liveController.turn_off_illumination()
            image = utils.crop_image(image,self.crop_width,self.crop_height)
            self.image_to_display.emit(image)
            QApplication.processEvents()
            focus_measure = utils.calculate_focus_measure(image,FOCUS_MEASURE_OPERATOR)
            focus_measure_vs_z[i] = focus_measure
            focus_measure_max = max(focus_measure, focus_measure_max)
            if focus_measure < focus_measure_max*AF.STOP_THRESHOLD:
                break

        # move to the starting location
        # self.navigationController.move_z_usteps(-steps_moved*self.deltaZ_usteps) # combine with the back and forth maneuver below
        # self.wait_till_operation_is_completed()

        # maneuver for achiving uniform step size and repeatability when using open-loop control
        self.navigationController.move_z_usteps(-_usteps_to_clear_backlash-steps_moved*self.deltaZ_usteps)
        # determine the in-focus position
        idx_in_focus = focus_measure_vs_z.index(max(focus_measure_vs_z))
        self.wait_till_operation_is_completed()
        self.navigationController.move_z_usteps(_usteps_to_clear_backlash+(idx_in_focus+1)*self.deltaZ_usteps)
        self.wait_till_operation_is_completed()

        # move to the calculated in-focus position
        # self.navigationController.move_z_usteps(idx_in_focus*self.deltaZ_usteps)
        # self.wait_till_operation_is_completed() # combine with the movement above
        if idx_in_focus == 0:
            print('moved to the bottom end of the AF range')
        if idx_in_focus == self.N-1:
            print('moved to the top end of the AF range')

class AutoFocusController(QObject):

    z_pos = Signal(float)
    autofocusFinished = Signal()
    image_to_display = Signal(np.ndarray)

    def __init__(self,camera:camera.Camera,navigationController:core.NavigationController,liveController:core.LiveController):
        QObject.__init__(self)
        self.camera = camera
        self.navigationController:core.NavigationController = navigationController
        self.liveController:core.LiveController = liveController
        self.N = None
        self.deltaZ = None
        self.deltaZ_usteps = None
        self.crop_width:int = AF.CROP_WIDTH
        self.crop_height:int = AF.CROP_HEIGHT
        self.autofocus_in_progress:bool = False

    def set_N(self,N):
        self.N = N

    def set_deltaZ(self,deltaZ_um:float):
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaZ = deltaZ_um/1000
        self.deltaZ_usteps = round((deltaZ_um/1000)/mm_per_ustep_Z)

    def set_crop(self,crop_width:int,crop_height:int):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def autofocus(self):
        # stop live
        if self.liveController.is_live:
            self.was_live_before_autofocus = True
            self.liveController.stop_live()
        else:
            self.was_live_before_autofocus = False

        # temporarily disable call back -> image does not go through streamHandler
        if self.camera.callback_is_enabled:
            self.callback_was_enabled_before_autofocus = True
            self.camera.disable_callback()
        else:
            self.callback_was_enabled_before_autofocus = False

        self.autofocus_in_progress = True

        # create a QThread object
        try:
            if self.thread.isRunning():
                print('*** autofocus thread is still running ***')
                self.thread.terminate()
                self.thread.wait()
                print('*** autofocus threaded manually stopped ***')
        except:
            pass
        self.thread = QThread()
        # create a worker object
        self.autofocusWorker = AutofocusWorker(self)
        # move the worker to the thread
        self.autofocusWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.autofocusWorker.run)
        self.autofocusWorker.finished.connect(self._on_autofocus_completed)
        self.autofocusWorker.finished.connect(self.autofocusWorker.deleteLater)
        self.autofocusWorker.finished.connect(self.thread.quit)
        self.autofocusWorker.image_to_display.connect(self.slot_image_to_display)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()
        
    def _on_autofocus_completed(self):
        # re-enable callback
        if self.callback_was_enabled_before_autofocus:
            self.camera.enable_callback()
        
        # re-enable live if it's previously on
        if self.was_live_before_autofocus:
            self.liveController.start_live()

        # emit the autofocus finished signal to enable the UI
        self.autofocusFinished.emit()
        QApplication.processEvents()
        print('autofocus finished')

        # update the state
        self.autofocus_in_progress = False

    def slot_image_to_display(self,image):
        self.image_to_display.emit(image)

    def wait_till_autofocus_has_completed(self):
        while self.autofocus_in_progress == True:
            time.sleep(0.005)
        print('autofocus wait has completed, exit wait')

class MultiPointWorker(QObject):

    finished = Signal()
    image_to_display = Signal(np.ndarray)
    spectrum_to_display = Signal(np.ndarray)
    image_to_display_multi = Signal(np.ndarray,int)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)

    def __init__(self,multiPointController:core.MultiPointController):
        QObject.__init__(self)
        self.multiPointController = multiPointController

        self.camera = self.multiPointController.camera
        self.microcontroller = self.multiPointController.microcontroller
        self.navigationController = self.multiPointController.navigationController
        self.liveController = self.multiPointController.liveController
        self.autofocusController = self.multiPointController.autofocusController
        self.configurationManager = self.multiPointController.configurationManager
        self.NX = self.multiPointController.NX
        self.NY = self.multiPointController.NY
        self.NZ = self.multiPointController.NZ
        self.Nt = self.multiPointController.Nt
        self.deltaX = self.multiPointController.deltaX
        self.deltaX_usteps = self.multiPointController.deltaX_usteps
        self.deltaY = self.multiPointController.deltaY
        self.deltaY_usteps = self.multiPointController.deltaY_usteps
        self.deltaZ = self.multiPointController.deltaZ
        self.deltaZ_usteps = self.multiPointController.deltaZ_usteps
        self.dt = self.multiPointController.deltat
        self.do_autofocus = self.multiPointController.do_autofocus
        self.crop_width = self.multiPointController.crop_width
        self.crop_height = self.multiPointController.crop_height
        self.display_resolution_scaling = self.multiPointController.display_resolution_scaling
        self.counter = self.multiPointController.counter
        self.experiment_ID = self.multiPointController.experiment_ID
        self.base_path = self.multiPointController.base_path
        self.selected_configurations = self.multiPointController.selected_configurations

        self.timestamp_acquisition_started = self.multiPointController.timestamp_acquisition_started
        self.time_point = 0

    def run(self):
        while self.time_point < self.Nt:
            # continous acquisition
            if self.dt == 0:
                self.run_single_time_point()
                if self.multiPointController.abort_acqusition_requested:
                    break
                self.time_point = self.time_point + 1
            # timed acquisition
            else:
                self.run_single_time_point()
                if self.multiPointController.abort_acqusition_requested:
                    break
                self.time_point = self.time_point + 1
                # check if the aquisition has taken longer than dt or integer multiples of dt, if so skip the next time point(s)
                while time.time() > self.timestamp_acquisition_started + self.time_point*self.dt:
                    print('skip time point ' + str(self.time_point+1))
                    self.time_point = self.time_point+1
                if self.time_point == self.Nt:
                    break # no waiting after taking the last time point
                # wait until it's time to do the next acquisition
                while time.time() < self.timestamp_acquisition_started + self.time_point*self.dt:
                    time.sleep(0.05)
        self.finished.emit()

    def wait_till_operation_is_completed(self):
        while self.microcontroller.is_busy():
            time.sleep(SLEEP_TIME_S)

    def run_single_time_point(self):

        # disable joystick button action
        self.navigationController.enable_joystick_button_action = False

        self.FOV_counter = 0
        print('multipoint acquisition - time point ' + str(self.time_point+1))
        
        # for each time point, create a new folder
        current_path = os.path.join(self.base_path,self.experiment_ID,str(self.time_point))
        os.mkdir(current_path)

        # create a dataframe to save coordinates
        coordinates_pd = pd.DataFrame(columns = ['i', 'j', 'k', 'x (mm)', 'y (mm)', 'z (um)'])

        if self.multiPointController.scanCoordinates!=None:
            # use scan coordinates for the scan
            self.multiPointController.scanCoordinates.get_selected_wells()
            self.scan_coordinates_mm = self.multiPointController.scanCoordinates.coordinates_mm
            self.scan_coordinates_name = self.multiPointController.scanCoordinates.name
            self.use_scan_coordinates = True
        else:
            # use the current position for the scan
            self.scan_coordinates_mm = [(self.navigationController.x_pos_mm,self.navigationController.y_pos_mm)]
            self.scan_coordinates_name = ['']
            self.use_scan_coordinates = False

        n_regions = len(self.scan_coordinates_name)
        for coordinate_id in range(n_regions):
            coordiante_mm = self.scan_coordinates_mm[coordinate_id]
            coordiante_name = self.scan_coordinates_name[coordinate_id]
            if self.use_scan_coordinates:
                # move to the specified coordinate
                self.navigationController.move_x_to(coordiante_mm[0]-self.deltaX*(self.NX-1)/2)
                self.wait_till_operation_is_completed()
                time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)
                self.navigationController.move_y_to(coordiante_mm[1]-self.deltaY*(self.NY-1)/2)
                self.wait_till_operation_is_completed()
                time.sleep(SCAN_STABILIZATION_TIME_MS_Y/1000)
                # add '_' to the coordinate name
                coordiante_name = coordiante_name + '_'

            x_scan_direction = 1
            dx_usteps = 0
            dy_usteps = 0
            dz_usteps = 0
            z_pos = self.navigationController.z_pos

            # z stacking config
            if Z_STACKING_CONFIG == 'FROM TOP':
                self.deltaZ_usteps = -abs(self.deltaZ_usteps)

            # along y
            for i in range(self.NY):

                self.FOV_counter = 0 # so that AF at the beginning of each new row

                # along x
                for j in range(self.NX):

                    # perform AF only if when not taking z stack or doing z stack from center
                    if ( (self.NZ == 1) or Z_STACKING_CONFIG == 'FROM CENTER' ) and (self.do_autofocus) and (self.FOV_counter%Acquisition.NUMBER_OF_FOVS_PER_AF==0):
                    # temporary: replace the above line with the line below to AF every FOV
                    # if (self.NZ == 1) and (self.do_autofocus):
                        configuration_name_AF = self.multiPointController.autofocus_channel_name
                        config_AF = next((config for config in self.configurationManager.configurations if config.name == configuration_name_AF))
                        self.signal_current_configuration.emit(config_AF)
                        self.autofocusController.autofocus()
                        self.autofocusController.wait_till_autofocus_has_completed()
                    
                    if (self.NZ > 1):
                        # move to bottom of the z stack
                        if Z_STACKING_CONFIG == 'FROM CENTER':
                            self.navigationController.move_z_usteps(-self.deltaZ_usteps*round((self.NZ-1)/2))
                            self.wait_till_operation_is_completed()
                            time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)
                        # maneuver for achiving uniform step size and repeatability when using open-loop control
                        self.navigationController.move_z_usteps(-160)
                        self.wait_till_operation_is_completed()
                        self.navigationController.move_z_usteps(160)
                        self.wait_till_operation_is_completed()
                        time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)

                    # z-stack
                    for k in range(self.NZ):
                        
                        file_ID = coordiante_name + str(i) + '_' + str(j if x_scan_direction==1 else self.NX-1-j) + '_' + str(k)
                        # metadata = dict(x = self.navigationController.x_pos_mm, y = self.navigationController.y_pos_mm, z = self.navigationController.z_pos_mm)
                        # metadata = json.dumps(metadata)

                        # iterate through selected modes
                        for config in self.selected_configurations:
                            if 'USB Spectrometer' not in config.name:
                                # update the current configuration
                                self.signal_current_configuration.emit(config)
                                self.wait_till_operation_is_completed()
                                # trigger acquisition (including turning on the illumination)
                                if self.liveController.trigger_mode == TriggerMode.SOFTWARE:
                                    self.liveController.turn_on_illumination()
                                    self.wait_till_operation_is_completed()
                                    self.camera.send_trigger()
                                elif self.liveController.trigger_mode == TriggerMode.HARDWARE:
                                    self.microcontroller.send_hardware_trigger(control_illumination=True,illumination_on_time_us=self.camera.exposure_time*1000)
                                # read camera frame
                                image = self.camera.read_frame()
                                # tunr of the illumination if using software trigger
                                if self.liveController.trigger_mode == TriggerMode.SOFTWARE:
                                    self.liveController.turn_off_illumination()
                                # process the image -  @@@ to move to camera
                                image = utils.crop_image(image,self.crop_width,self.crop_height)
                                image = utils.rotate_and_flip_image(image,rotate_image_angle=self.camera.rotate_image_angle,flip_image=self.camera.flip_image)
                                # self.image_to_display.emit(cv2.resize(image,(round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling)),cv2.INTER_LINEAR))
                                image_to_display = utils.crop_image(image,round(self.crop_width*self.display_resolution_scaling), round(self.crop_height*self.display_resolution_scaling))
                                self.image_to_display.emit(image_to_display)
                                self.image_to_display_multi.emit(image_to_display,config.illumination_source)
                                if image.dtype == np.uint16:
                                    saving_path = os.path.join(current_path, file_ID + '_' + str(config.name).replace(' ','_') + '.tiff')
                                    if self.camera.is_color:
                                        if 'BF LED matrix' in config.name:
                                            if MULTIPOINT_BF_SAVING_OPTION == 'RGB2GRAY':
                                                image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
                                            elif MULTIPOINT_BF_SAVING_OPTION == 'Green Channel Only':
                                                image = image[:,:,1]
                                    iio.imwrite(saving_path,image)
                                else:
                                    saving_path = os.path.join(current_path, file_ID + '_' + str(config.name).replace(' ','_') + '.' + Acquisition.IMAGE_FORMAT)
                                    if self.camera.is_color:
                                        if 'BF LED matrix' in config.name:
                                            if MULTIPOINT_BF_SAVING_OPTION == 'Raw':
                                                image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                                            elif MULTIPOINT_BF_SAVING_OPTION == 'RGB2GRAY':
                                                image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
                                            elif MULTIPOINT_BF_SAVING_OPTION == 'Green Channel Only':
                                                image = image[:,:,1]
                                        else:
                                            image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                                    cv2.imwrite(saving_path,image)
                                QApplication.processEvents()
                            else:
                                pass

                        # add the coordinate of the current location
                        coordinates_pd = pd.concat([
                            coordinates_pd,
                            pd.DataFrame([{'i':i,'j':j,'k':k,
                                            'x (mm)':self.navigationController.x_pos_mm,
                                            'y (mm)':self.navigationController.y_pos_mm,
                                            'z (um)':self.navigationController.z_pos_mm*1000}])
                        ])

                        # register the current fov in the navigationViewer 
                        self.signal_register_current_fov.emit(self.navigationController.x_pos_mm,self.navigationController.y_pos_mm)

                        # check if the acquisition should be aborted
                        if self.multiPointController.abort_acqusition_requested:
                            self.liveController.turn_off_illumination()
                            self.navigationController.move_x_usteps(-dx_usteps)
                            self.wait_till_operation_is_completed()
                            self.navigationController.move_y_usteps(-dy_usteps)
                            self.wait_till_operation_is_completed()
                            self.navigationController.move_z_usteps(-dz_usteps)
                            self.wait_till_operation_is_completed()
                            coordinates_pd.to_csv(os.path.join(current_path,'coordinates.csv'),index=False,header=True)
                            self.navigationController.enable_joystick_button_action = True
                            return

                        if self.NZ > 1:
                            # move z
                            if k < self.NZ - 1:
                                self.navigationController.move_z_usteps(self.deltaZ_usteps)
                                self.wait_till_operation_is_completed()
                                time.sleep(SCAN_STABILIZATION_TIME_MS_Z/1000)
                                dz_usteps = dz_usteps + self.deltaZ_usteps
                    
                    if self.NZ > 1:
                        # move z back
                        if Z_STACKING_CONFIG == 'FROM CENTER':
                            self.navigationController.move_z_usteps( -self.deltaZ_usteps*(self.NZ-1) + self.deltaZ_usteps*round((self.NZ-1)/2) )
                            self.wait_till_operation_is_completed()
                            dz_usteps = dz_usteps - self.deltaZ_usteps*(self.NZ-1) + self.deltaZ_usteps*round((self.NZ-1)/2)
                        else:
                            self.navigationController.move_z_usteps(-self.deltaZ_usteps*(self.NZ-1))
                            self.wait_till_operation_is_completed()
                            dz_usteps = dz_usteps - self.deltaZ_usteps*(self.NZ-1)

                    # update FOV counter
                    self.FOV_counter = self.FOV_counter + 1

                    if self.NX > 1:
                        # move x
                        if j < self.NX - 1:
                            self.navigationController.move_x_usteps(x_scan_direction*self.deltaX_usteps)
                            self.wait_till_operation_is_completed()
                            time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)
                            dx_usteps = dx_usteps + x_scan_direction*self.deltaX_usteps

                '''
                # instead of move back, reverse scan direction (12/29/2021)
                if self.NX > 1:
                    # move x back
                    self.navigationController.move_x_usteps(-self.deltaX_usteps*(self.NX-1))
                    self.wait_till_operation_is_completed()
                    time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)
                '''
                x_scan_direction = -x_scan_direction

                if self.NY > 1:
                    # move y
                    if i < self.NY - 1:
                        self.navigationController.move_y_usteps(self.deltaY_usteps)
                        self.wait_till_operation_is_completed()
                        time.sleep(SCAN_STABILIZATION_TIME_MS_Y/1000)
                        dy_usteps = dy_usteps + self.deltaY_usteps

            if n_regions == 1:
                # only move to the start position if there's only one region in the scan
                if self.NY > 1:
                    # move y back
                    self.navigationController.move_y_usteps(-self.deltaY_usteps*(self.NY-1))
                    self.wait_till_operation_is_completed()
                    time.sleep(SCAN_STABILIZATION_TIME_MS_Y/1000)
                    dy_usteps = dy_usteps - self.deltaY_usteps*(self.NY-1)

                # move x back at the end of the scan
                if x_scan_direction == -1:
                    self.navigationController.move_x_usteps(-self.deltaX_usteps*(self.NX-1))
                    self.wait_till_operation_is_completed()
                    time.sleep(SCAN_STABILIZATION_TIME_MS_X/1000)

                # move z back
                self.navigationController.microcontroller.move_z_to_usteps(z_pos)
                self.wait_till_operation_is_completed()

        coordinates_pd.to_csv(os.path.join(current_path,'coordinates.csv'),index=False,header=True)
        self.navigationController.enable_joystick_button_action = True

class ScanCoordinates(object):
    def __init__(self):
        self.coordinates_mm = []
        self.name = []
        self.well_selector:Optional[widgets.WellSelectionWidget] = None

    def add_well_selector(self,well_selector:widgets.WellSelectionWidget):
        self.well_selector = well_selector

    def get_selected_wells(self):
        # get selected wells from the widget
        assert not self.well_selector is None
        selected_wells = self.well_selector.get_selected_cells()
        selected_wells = np.array(selected_wells)
        # clear the previous selection
        self.coordinates_mm = []
        self.name = []
        # populate the coordinates
        rows = np.unique(selected_wells[:,0])
        _increasing = True
        for row in rows:
            items = selected_wells[selected_wells[:,0]==row]
            columns = items[:,1]
            columns = np.sort(columns)
            if _increasing==False:
                columns = np.flip(columns)
            for column in columns:
                wellplateformat_384=WELLPLATE_FORMATS[384]
                x_mm = X_MM_384_WELLPLATE_UPPERLEFT + wellplateformat_384.well_size_mm/2 - (wellplateformat_384.A1_x_mm+wellplateformat_384.well_spacing_mm*NUMBER_OF_SKIP_384) + column*WELL_SPACING_MM + A1_X_MM + WELLPLATE_OFFSET_X_mm
                y_mm = Y_MM_384_WELLPLATE_UPPERLEFT + wellplateformat_384.well_size_mm/2 - (wellplateformat_384.A1_y_mm+wellplateformat_384.well_spacing_mm*NUMBER_OF_SKIP_384) + row*WELL_SPACING_MM + A1_Y_MM + WELLPLATE_OFFSET_Y_mm
                self.coordinates_mm.append((x_mm,y_mm))
                self.name.append(chr(ord('A')+row)+str(column+1))
            _increasing = not _increasing

class MultiPointController(QObject):

    acquisitionFinished = Signal()
    image_to_display = Signal(np.ndarray)
    image_to_display_multi = Signal(np.ndarray,int)
    spectrum_to_display = Signal(np.ndarray)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)

    def __init__(self,
        camera:camera.Camera,
        navigationController:core.NavigationController,
        liveController:core.LiveController,
        autofocusController:core.AutoFocusController,
        configurationManager:core.ConfigurationManager,
        scanCoordinates:Optional[ScanCoordinates]=None
    ):
        QObject.__init__(self)

        self.camera = camera
        self.microcontroller = navigationController.microcontroller # to move to gui for transparency
        self.navigationController = navigationController
        self.liveController = liveController
        self.autofocusController = autofocusController
        self.configurationManager = configurationManager
        self.NX = 1
        self.NY = 1
        self.NZ = 1
        self.Nt = 1
        mm_per_ustep_X = SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X)
        mm_per_ustep_Y = SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y)
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaX = Acquisition.DX
        self.deltaX_usteps = round(self.deltaX/mm_per_ustep_X)
        self.deltaY = Acquisition.DY
        self.deltaY_usteps = round(self.deltaY/mm_per_ustep_Y)
        self.deltaZ = Acquisition.DZ/1000
        self.deltaZ_usteps = round(self.deltaZ/mm_per_ustep_Z)
        self.deltat = 0
        self.do_autofocus = False
        self.crop_width = Acquisition.CROP_WIDTH
        self.crop_height = Acquisition.CROP_HEIGHT
        self.display_resolution_scaling = Acquisition.IMAGE_DISPLAY_SCALING_FACTOR
        self.counter = 0
        self.experiment_ID: Optional[str] = None
        self.base_path  = None
        self.selected_configurations = []
        self.scanCoordinates = scanCoordinates
        self.autofocus_channel_name=MULTIPOINT_AUTOFOCUS_CHANNEL

    def set_NX(self,N):
        self.NX = N
    def set_NY(self,N):
        self.NY = N
    def set_NZ(self,N):
        self.NZ = N
    def set_Nt(self,N):
        self.Nt = N
    def set_deltaX(self,delta):
        mm_per_ustep_X = SCREW_PITCH_X_MM/(self.navigationController.x_microstepping*FULLSTEPS_PER_REV_X)
        self.deltaX = delta
        self.deltaX_usteps = round(delta/mm_per_ustep_X)
    def set_deltaY(self,delta):
        mm_per_ustep_Y = SCREW_PITCH_Y_MM/(self.navigationController.y_microstepping*FULLSTEPS_PER_REV_Y)
        self.deltaY = delta
        self.deltaY_usteps = round(delta/mm_per_ustep_Y)
    def set_deltaZ(self,delta_um):
        mm_per_ustep_Z = SCREW_PITCH_Z_MM/(self.navigationController.z_microstepping*FULLSTEPS_PER_REV_Z)
        self.deltaZ = delta_um/1000
        self.deltaZ_usteps = round((delta_um/1000)/mm_per_ustep_Z)
    def set_deltat(self,delta):
        self.deltat = delta
    def set_af_flag(self,flag):
        self.do_autofocus = flag

    def set_crop(self,crop_width,crop_height):
        self.crop_width = crop_width
        self.crop_height = crop_height

    def set_base_path(self,path):
        self.base_path = path

    def start_new_experiment(self,experiment_ID): # @@@ to do: change name to prepare_folder_for_new_experiment
        # generate unique experiment ID
        self.experiment_ID = experiment_ID.replace(' ','_') + '_' + datetime.now().strftime('%Y-%m-%d_%H-%M-%-S.%f')
        self.recording_start_time = time.time()
        # create a new folder
        os.mkdir(os.path.join(self.base_path,self.experiment_ID))
        self.configurationManager.write_configuration(os.path.join(self.base_path,self.experiment_ID)+"/configurations.xml") # save the configuration for the experiment
        acquisition_parameters = {'dx(mm)':self.deltaX, 'Nx':self.NX, 'dy(mm)':self.deltaY, 'Ny':self.NY, 'dz(um)':self.deltaZ*1000,'Nz':self.NZ,'dt(s)':self.deltat,'Nt':self.Nt,'with AF':self.do_autofocus}
        f = open(os.path.join(self.base_path,self.experiment_ID)+"/acquisition parameters.json","w")
        f.write(json.dumps(acquisition_parameters))
        f.close()


    def set_selected_configurations(self, selected_configurations_name):
        self.selected_configurations = []
        for configuration_name in selected_configurations_name:
            self.selected_configurations.append(next((config for config in self.configurationManager.configurations if config.name == configuration_name)))
        
    def run_experiment(self):
        print(f"start multipoint {self.Nt=} {self.NX=} {self.NY=} {self.NZ=}")
 
        self.abort_acqusition_requested = False
 
        self.configuration_before_running_multipoint = self.liveController.currentConfiguration
        # stop live
        if self.liveController.is_live:
            self.liveController_was_live_before_multipoint = True
            self.liveController.stop_live() # @@@ to do: also uncheck the live button
        else:
            self.liveController_was_live_before_multipoint = False
 
        # LiveController.start_live enables this, but LiveController.stop_live does not disable, and a comment there explains that this (turning streaming off) causes issues
        # with af in multipoint (the issue is a crash because the camera does not send a picture).
        # if the live button was not pressed before multipoint acquisition is started, camera is not yet streaming, therefore crash -> start streaming when multipoint starts
        self.camera.start_streaming()

        # disable callback
        if self.camera.callback_is_enabled:
            self.camera_callback_was_enabled_before_multipoint = True
            self.camera.disable_callback()
        else:
            self.camera_callback_was_enabled_before_multipoint = False

        # run the acquisition
        self.timestamp_acquisition_started = time.time()
        # create a QThread object
        self.thread = QThread()
        # create a worker object
        self.multiPointWorker = MultiPointWorker(self)
        # move the worker to the thread
        self.multiPointWorker.moveToThread(self.thread)
        # connect signals and slots
        self.thread.started.connect(self.multiPointWorker.run)
        self.multiPointWorker.finished.connect(self._on_acquisition_completed)
        self.multiPointWorker.finished.connect(self.multiPointWorker.deleteLater)
        self.multiPointWorker.finished.connect(self.thread.quit)
        self.multiPointWorker.image_to_display.connect(self.slot_image_to_display)
        self.multiPointWorker.image_to_display_multi.connect(self.slot_image_to_display_multi)
        self.multiPointWorker.spectrum_to_display.connect(self.slot_spectrum_to_display)
        self.multiPointWorker.signal_current_configuration.connect(self.slot_current_configuration,type=Qt.BlockingQueuedConnection)
        self.multiPointWorker.signal_register_current_fov.connect(self.slot_register_current_fov)
        # self.thread.finished.connect(self.thread.deleteLater)
        self.thread.finished.connect(self.thread.quit)
        # start the thread
        self.thread.start()

    def _on_acquisition_completed(self):
        # restore the previous selected mode
        self.signal_current_configuration.emit(self.configuration_before_running_multipoint)

        # re-enable callback
        if self.camera_callback_was_enabled_before_multipoint:
            self.camera.enable_callback()
            self.camera_callback_was_enabled_before_multipoint = False
        
        # re-enable live if it's previously on
        if self.liveController_was_live_before_multipoint:
            self.liveController.start_live()
        
        # emit the acquisition finished signal to enable the UI
        self.acquisitionFinished.emit()
        QApplication.processEvents()

    def request_abort_aquisition(self):
        self.abort_acqusition_requested = True

    def slot_image_to_display(self,image):
        self.image_to_display.emit(image)

    def slot_spectrum_to_display(self,data):
        self.spectrum_to_display.emit(data)

    def slot_image_to_display_multi(self,image,illumination_source):
        self.image_to_display_multi.emit(image,illumination_source)

    def slot_current_configuration(self,configuration):
        self.signal_current_configuration.emit(configuration)

    def slot_register_current_fov(self,x_mm,y_mm):
        self.signal_register_current_fov.emit(x_mm,y_mm)

class ImageDisplayWindow(QMainWindow):

    def __init__(self, window_title='', draw_crosshairs = False, show_LUT=False, autoLevels=False):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()
        self.show_LUT = show_LUT
        self.autoLevels = autoLevels

        # interpret image data as row-major instead of col-major
        pg.setConfigOptions(imageAxisOrder='row-major')

        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.view = self.graphics_widget.addViewBox()
        self.graphics_widget.view.invertY()
        
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        
        ## Create image item
        if self.show_LUT:
            self.graphics_widget.view = pg.ImageView()
            self.graphics_widget.img = self.graphics_widget.view.getImageItem()
            self.graphics_widget.img.setBorder('w')
            self.graphics_widget.view.ui.roiBtn.hide()
            self.graphics_widget.view.ui.menuBtn.hide()
            # self.LUTWidget = self.graphics_widget.view.getHistogramWidget()
            # self.LUTWidget.autoHistogramRange()
            # self.graphics_widget.view.autolevels()
        else:
            self.graphics_widget.img = pg.ImageItem(border='w')
            self.graphics_widget.view.addItem(self.graphics_widget.img)

        ## Create ROI
        self.roi_pos = (500,500)
        self.roi_size = (500,500)
        self.ROI = pg.ROI(self.roi_pos, self.roi_size, scaleSnap=True, translateSnap=True)
        self.ROI.setZValue(10)
        self.ROI.addScaleHandle((0,0), (1,1))
        self.ROI.addScaleHandle((1,1), (0,0))
        self.graphics_widget.view.addItem(self.ROI)
        self.ROI.hide()
        self.ROI.sigRegionChanged.connect(self.update_ROI)
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

        ## Variables for annotating images
        self.draw_rectangle = False
        self.ptRect1 = None
        self.ptRect2 = None
        self.DrawCirc = False
        self.centroid = None
        self.DrawCrossHairs = False
        self.image_offset = np.array([0, 0])

        ## Layout
        layout = QGridLayout()
        if self.show_LUT:
            layout.addWidget(self.graphics_widget.view, 0, 0) 
        else:
            layout.addWidget(self.graphics_widget, 0, 0) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget();
        width = min(desktopWidget.height()*0.9,1000) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    def display_image(self,image):
        self.graphics_widget.img.setImage(image,autoLevels=self.autoLevels)

    def update_ROI(self):
        self.roi_pos = self.ROI.pos()
        self.roi_size = self.ROI.size()

    def show_ROI_selector(self):
        self.ROI.show()

    def hide_ROI_selector(self):
        self.ROI.hide()

    def get_roi(self):
        return self.roi_pos,self.roi_size

    def update_bounding_box(self,pts):
        self.draw_rectangle=True
        self.ptRect1=(pts[0][0],pts[0][1])
        self.ptRect2=(pts[1][0],pts[1][1])

    def get_roi_bounding_box(self):
        self.update_ROI()
        width = self.roi_size[0]
        height = self.roi_size[1]
        xmin = max(0, self.roi_pos[0])
        ymin = max(0, self.roi_pos[1])
        return np.array([xmin, ymin, width, height])

    def set_autolevel(self,enabled):
        self.autoLevels = enabled
        print('set autolevel to ' + str(enabled))

class NavigationViewer(QFrame):

    def __init__(self, sample = 'glass slide', invertX = False, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setFrameStyle(QFrame.Panel | QFrame.Raised)

        # interpret image data as row-major instead of col-major
        pg.setConfigOptions(imageAxisOrder='row-major')
        self.graphics_widget = pg.GraphicsLayoutWidget()
        self.graphics_widget.setBackground("w")
        self.graphics_widget.view = self.graphics_widget.addViewBox(invertX=invertX,invertY=True)
        ## lock the aspect ratio so pixels are always square
        self.graphics_widget.view.setAspectLocked(True)
        ## Create image item
        self.graphics_widget.img = pg.ImageItem(border='w')
        self.graphics_widget.view.addItem(self.graphics_widget.img)

        self.grid = QVBoxLayout()
        self.grid.addWidget(self.graphics_widget)
        self.setLayout(self.grid)
 
        self.last_fov_drawn=None
        self.set_wellplate_type(sample)
 
        self.location_update_threshold_mm = 0.4    
 
        self.box_color = (255, 0, 0)
        self.box_line_thickness = 2
 
        self.x_mm = None
        self.y_mm = None
 
        self.update_display()
 
    def set_wellplate_type(self,wellplate_type:str):
        wellplate_type_image={
            'glass slide'    : 'images/slide carrier_828x662.png',
            '384 well plate' : 'images/384 well plate_1509x1010.png',
            '96 well plate'  : 'images/96 well plate_1509x1010.png',
            '24 well plate'  : 'images/24 well plate_1509x1010.png',
            '12 well plate'  : 'images/12 well plate_1509x1010.png',
            '6 well plate'   : 'images/6 well plate_1509x1010.png'
        }
        assert wellplate_type in wellplate_type_image, f"{wellplate_type} is not a valid plate type"
 
        self.background_image=cv2.imread(wellplate_type_image[wellplate_type])
 
        self.current_image = np.copy(self.background_image)
        self.current_image_display = np.copy(self.background_image)
        self.image_height = self.background_image.shape[0]
        self.image_width = self.background_image.shape[1]
 
        self.sample = wellplate_type
 
        if wellplate_type == 'glass slide':
            self.origin_bottom_left_x = 200
            self.origin_bottom_left_y = 120
            self.mm_per_pixel = 0.1453
            self.fov_size_mm = 3000*1.85/(50/9)/1000
        else:
            self.location_update_threshold_mm = 0.05
            self.mm_per_pixel = 0.084665
            self.fov_size_mm = 3000*1.85/(50/10)/1000
            self.origin_bottom_left_x = X_ORIGIN_384_WELLPLATE_PIXEL - (X_MM_384_WELLPLATE_UPPERLEFT)/self.mm_per_pixel
            self.origin_bottom_left_y = Y_ORIGIN_384_WELLPLATE_PIXEL - (Y_MM_384_WELLPLATE_UPPERLEFT)/self.mm_per_pixel
 
        self.clear_imaged_positions()
 
    def update_current_location(self,x_mm,y_mm):
        if self.x_mm != None and self.y_mm != None:
            # update only when the displacement has exceeded certain value
            if abs(x_mm - self.x_mm) > self.location_update_threshold_mm or abs(y_mm - self.y_mm) > self.location_update_threshold_mm:
                self.draw_current_fov(x_mm,y_mm)
                self.update_display()
                self.x_mm = x_mm
                self.y_mm = y_mm
        else:
            self.draw_current_fov(x_mm,y_mm)
            self.update_display()
            self.x_mm = x_mm
            self.y_mm = y_mm

    def draw_current_fov(self,x_mm,y_mm):
        self.current_image_display = np.copy(self.current_image)
        if self.sample == 'glass slide':
            current_FOV_top_left = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel - self.fov_size_mm/2/self.mm_per_pixel),
                                    round(self.image_height - (self.origin_bottom_left_y + y_mm/self.mm_per_pixel) - self.fov_size_mm/2/self.mm_per_pixel))
            current_FOV_bottom_right = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel + self.fov_size_mm/2/self.mm_per_pixel),
                                    round(self.image_height - (self.origin_bottom_left_y + y_mm/self.mm_per_pixel) + self.fov_size_mm/2/self.mm_per_pixel))
        else:
            current_FOV_top_left = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel - self.fov_size_mm/2/self.mm_per_pixel),
                                    round((self.origin_bottom_left_y + y_mm/self.mm_per_pixel) - self.fov_size_mm/2/self.mm_per_pixel))
            current_FOV_bottom_right = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel + self.fov_size_mm/2/self.mm_per_pixel),
                                    round((self.origin_bottom_left_y + y_mm/self.mm_per_pixel) + self.fov_size_mm/2/self.mm_per_pixel))
        cv2.rectangle(self.current_image_display, current_FOV_top_left, current_FOV_bottom_right, self.box_color, self.box_line_thickness)

        self.last_fov_drawn=(x_mm,y_mm)

    def clear_imaged_positions(self):
        self.current_image = np.copy(self.background_image)
        if not self.last_fov_drawn is None:
            self.draw_current_fov(self.last_fov_drawn[0],self.last_fov_drawn[1])
        self.update_display()

    def update_display(self):
        self.graphics_widget.img.setImage(self.current_image_display,autoLevels=False)

    def clear_slide(self):
        self.current_image = np.copy(self.background_image)
        self.current_image_display = np.copy(self.background_image)
        self.update_display()

    def register_fov(self,x_mm,y_mm):
        color = (0,0,255)
        if self.sample == 'glass slide':
            current_FOV_top_left = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel - self.fov_size_mm/2/self.mm_per_pixel),
                                    round(self.image_height - (self.origin_bottom_left_y + y_mm/self.mm_per_pixel) - self.fov_size_mm/2/self.mm_per_pixel))
            current_FOV_bottom_right = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel + self.fov_size_mm/2/self.mm_per_pixel),
                                    round(self.image_height - (self.origin_bottom_left_y + y_mm/self.mm_per_pixel) + self.fov_size_mm/2/self.mm_per_pixel))
        else:
            current_FOV_top_left = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel - self.fov_size_mm/2/self.mm_per_pixel),
                                    round((self.origin_bottom_left_y + y_mm/self.mm_per_pixel) - self.fov_size_mm/2/self.mm_per_pixel))
            current_FOV_bottom_right = (round(self.origin_bottom_left_x + x_mm/self.mm_per_pixel + self.fov_size_mm/2/self.mm_per_pixel),
                                    round((self.origin_bottom_left_y + y_mm/self.mm_per_pixel) + self.fov_size_mm/2/self.mm_per_pixel))
        cv2.rectangle(self.current_image, current_FOV_top_left, current_FOV_bottom_right, color, self.box_line_thickness)

class ImageArrayDisplayWindow(QMainWindow):

    def __init__(self, window_title=''):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint)
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint)
        self.widget = QWidget()

        # interpret image data as row-major instead of col-major
        pg.setConfigOptions(imageAxisOrder='row-major')

        self.graphics_widget_1 = pg.GraphicsLayoutWidget()
        self.graphics_widget_1.view = self.graphics_widget_1.addViewBox()
        self.graphics_widget_1.view.setAspectLocked(True)
        self.graphics_widget_1.img = pg.ImageItem(border='w')
        self.graphics_widget_1.view.addItem(self.graphics_widget_1.img)

        self.graphics_widget_2 = pg.GraphicsLayoutWidget()
        self.graphics_widget_2.view = self.graphics_widget_2.addViewBox()
        self.graphics_widget_2.view.setAspectLocked(True)
        self.graphics_widget_2.img = pg.ImageItem(border='w')
        self.graphics_widget_2.view.addItem(self.graphics_widget_2.img)

        self.graphics_widget_3 = pg.GraphicsLayoutWidget()
        self.graphics_widget_3.view = self.graphics_widget_3.addViewBox()
        self.graphics_widget_3.view.setAspectLocked(True)
        self.graphics_widget_3.img = pg.ImageItem(border='w')
        self.graphics_widget_3.view.addItem(self.graphics_widget_3.img)

        self.graphics_widget_4 = pg.GraphicsLayoutWidget()
        self.graphics_widget_4.view = self.graphics_widget_4.addViewBox()
        self.graphics_widget_4.view.setAspectLocked(True)
        self.graphics_widget_4.img = pg.ImageItem(border='w')
        self.graphics_widget_4.view.addItem(self.graphics_widget_4.img)

        ## Layout
        layout = QGridLayout()
        layout.addWidget(self.graphics_widget_1, 0, 0)
        layout.addWidget(self.graphics_widget_2, 0, 1)
        layout.addWidget(self.graphics_widget_3, 1, 0)
        layout.addWidget(self.graphics_widget_4, 1, 1) 
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget();
        width = min(desktopWidget.height()*0.9,1000) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    def display_image(self,image,illumination_source):
        if illumination_source < 11:
            self.graphics_widget_1.img.setImage(image,autoLevels=False)
        elif illumination_source == 11:
            self.graphics_widget_2.img.setImage(image,autoLevels=False)
        elif illumination_source == 12:
            self.graphics_widget_3.img.setImage(image,autoLevels=False)
        elif illumination_source == 13:
            self.graphics_widget_4.img.setImage(image,autoLevels=False)
