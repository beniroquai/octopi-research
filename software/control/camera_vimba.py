import argparse
import cv2
import time
import numpy as np
import copy
import threading
import queue
import os

from typing import Optional
from vimba import *

from control._def import *

from PIL import Image

try:
    from control.vimbapy.vicamera_frameproducer import *
    from control.vimbapy.vicamera_frameconsumer import *
    from control.vimbapy.vicamera import VimbaCameraThread
except:
    from vicamera_frameproducer import *
    from vicamera_frameconsumer import *

FRAME_HEIGHT = 1088
FRAME_WIDTH = 1456

# Camera Settings
#CAM_GAIN = 20 # dB
T_EXPOSURE_MAX = 1e6 # µs => 1s
ExposureTime = 50e3


class Camera(object):

    def __init__(self,sn=None):

        # many to be purged
        self.sn = sn
        self.device_info_list = None
        self.device_index = 0
        self.camera = None
        self.is_color = None
        self.gamma_lut = None
        self.contrast_lut = None
        self.color_correction_param = None

        self.blacklevel = 00
        self.exposure_time = 0
        self.analog_gain = 0
        self.frame_ID = -1
        self.frame_ID_software = -1
        self.frame_ID_offset_hardware_trigger = 0
        self.timestamp = 0

        self.image_locked = False
        self.current_frame = None

        self.callback_is_enabled = False
        self.callback_was_enabled_before_autofocus = False
        self.callback_was_enabled_before_multipoint = False
        self.is_streaming = False

        self.GAIN_MAX = 24
        self.GAIN_MIN = 0
        self.GAIN_STEP = 1
        self.EXPOSURE_TIME_MS_MIN = 0.01
        self.EXPOSURE_TIME_MS_MAX = 4000

        self.ROI_offset_x = CAMERA.ROI_OFFSET_X_DEFAULT
        self.ROI_offset_y = CAMERA.ROI_OFFSET_X_DEFAULT
        self.ROI_width = CAMERA.ROI_WIDTH_DEFAULT
        self.ROI_height = CAMERA.ROI_HEIGHT_DEFAULT

    def open(self,index=0):
        # no camera thread open, generate one!
        self.camera = VimbaCameraThread()
        # temporary
        self.camera.start()
        self.camera.setExposureTime(1000)
        
    def set_callback(self,function):
        self.new_image_callback_external = function

    def enable_callback(self):
        user_param = None
        self.camera.register_capture_callback(self._on_frame_callback)
        #self.camera.register_capture_callback(user_param,self._on_frame_callback)
        self.callback_is_enabled = True

    def disable_callback(self):
        #self.camera.unregister_capture_callback()
        self.callback_is_enabled = False

    def open_by_sn(self,sn):
        pass

    def close(self):
        self.camera.stop_preview()
        self.camera.close()
        self.device_info_list = None
        self.camera = None
        self.is_color = None
        self.gamma_lut = None
        self.contrast_lut = None
        self.color_correction_param = None
        self.last_raw_image = None
        self.last_converted_image = None
        self.last_numpy_image = None

    def set_exposure_time(self,exposure_time):
        self.exposure_time = exposure_time
        self.camera.setExposureTime(self.exposure_time*1000)

    def set_analog_gain(self,analog_gain):
        self.analog_gain = analog_gain
        self.camera.setGain(self.analog_gain)
        
    def get_awb_ratios(self):
        '''
        self.camera.BalanceWhiteAuto.set(2)
        self.camera.BalanceRatioSelector.set(0)
        awb_r = self.camera.BalanceRatio.get()
        self.camera.BalanceRatioSelector.set(1)
        awb_g = self.camera.BalanceRatio.get()
        self.camera.BalanceRatioSelector.set(2)
        awb_b = self.camera.BalanceRatio.get()
        '''
        awb_r, awb_g, awb_b = 1,1,1
        return (awb_r, awb_g, awb_b)

    def set_wb_ratios(self, wb_r=None, wb_g=None, wb_b=None):
        pass

    def start_streaming(self):
        self.is_streaming = True

    def stop_streaming(self):
        self.is_streaming = False

    def set_pixel_format(self,format):
        #TODO: Implement
        '''
        if self.is_streaming == True:
            was_streaming = True
            self.stop_streaming()
        else:
            was_streaming = False

        if self.camera.PixelFormat.is_implemented() and self.camera.PixelFormat.is_writable():
            if format == 'MONO8':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.MONO8)
            if format == 'MONO12':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.MONO12)
            if format == 'MONO14':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.MONO14)
            if format == 'MONO16':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.MONO16)
            if format == 'BAYER_RG8':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.BAYER_RG8)
            if format == 'BAYER_RG12':
                self.camera.PixelFormat.set(gx.GxPixelFormatEntry.BAYER_RG12)
        else:
            print("pixel format is not implemented or not writable")

        if was_streaming:
           self.start_streaming()
        '''
        pass

    def set_continuous_acquisition(self):
        '''
        self.camera.TriggerMode.set(gx.GxSwitchEntry.OFF)
        '''
        pass

    def set_software_triggered_acquisition(self):
        '''
        self.camera.TriggerMode.set(gx.GxSwitchEntry.ON)
        self.camera.TriggerSource.set(gx.GxTriggerSourceEntry.SOFTWARE)
        '''
        pass
    
    def set_hardware_triggered_acquisition(self):
        '''
        self.camera.TriggerMode.set(gx.GxSwitchEntry.ON)
        self.camera.TriggerSource.set(gx.GxTriggerSourceEntry.LINE2)
        # self.camera.TriggerSource.set(gx.GxTriggerActivationEntry.RISING_EDGE)
        self.frame_ID_offset_hardware_trigger = self.frame_ID
        '''
        pass

    def send_trigger(self):
        '''
        if self.is_streaming:
            self.camera.TriggerSoftware.send_command()
        else:
        	print('trigger not sent - camera is not streaming')
        '''
        pass

    def read_frame(self):
        # get frame and save
        numpy_image = np.squeeze(self.camera.getLatestFrame(is_raw=True))
        return numpy_image

    def _on_frame_callback(self, user_param, raw_image):
        if raw_image is None:
            print("Getting image failed.")
            return
        if raw_image is None:
            print("Got an incomplete frame")
            return
        if self.image_locked:
            print('last image is still being processed, a frame is dropped')
            return
        if self.is_color:
            rgb_image = raw_image.convert("RGB")
            numpy_image = rgb_image.get_numpy_array()
        else:
            numpy_image = raw_image
        if numpy_image is None:
            return
        self.current_frame = numpy_image
        self.frame_ID_software = self.frame_ID_software + 1
        self.frame_ID = self.camera.consumer.iframe
        self.timestamp = time.time()
        self.new_image_callback_external(self)

        # self.frameID = self.frameID + 1
        # print(self.frameID)
    
    def set_ROI(self,offset_x=None,offset_y=None,width=None,height=None):
        '''
        if offset_x is not None:
            self.ROI_offset_x = offset_x
            # stop streaming if streaming is on
            if self.is_streaming == True:
                was_streaming = True
                self.stop_streaming()
            else:
                was_streaming = False
            # update the camera setting
            if self.camera.OffsetX.is_implemented() and self.camera.OffsetX.is_writable():
                self.camera.OffsetX.set(self.ROI_offset_x)
            else:
                print("OffsetX is not implemented or not writable")
            # restart streaming if it was previously on
            if was_streaming == True:
                self.start_streaming()

        if offset_y is not None:
            self.ROI_offset_y = offset_y
                # stop streaming if streaming is on
            if self.is_streaming == True:
                was_streaming = True
                self.stop_streaming()
            else:
                was_streaming = False
            # update the camera setting
            if self.camera.OffsetY.is_implemented() and self.camera.OffsetY.is_writable():
                self.camera.OffsetY.set(self.ROI_offset_y)
            else:
                print("OffsetX is not implemented or not writable")
            # restart streaming if it was previously on
            if was_streaming == True:
                self.start_streaming()

        if width is not None:
            self.ROI_width = width
            # stop streaming if streaming is on
            if self.is_streaming == True:
                was_streaming = True
                self.stop_streaming()
            else:
                was_streaming = False
            # update the camera setting
            if self.camera.Width.is_implemented() and self.camera.Width.is_writable():
                self.camera.Width.set(self.ROI_width)
            else:
                print("OffsetX is not implemented or not writable")
            # restart streaming if it was previously on
            if was_streaming == True:
                self.start_streaming()


        if height is not None:
            self.ROI_height = height
            # stop streaming if streaming is on
            if self.is_streaming == True:
                was_streaming = True
                self.stop_streaming()
            else:
                was_streaming = False
            # update the camera setting
            if self.camera.Height.is_implemented() and self.camera.Height.is_writable():
                self.camera.Height.set(self.ROI_height)
            else:
                print("Height is not implemented or not writable")
            # restart streaming if it was previously on
            if was_streaming == True:
                self.start_streaming()
        '''
        pass

    def reset_camera_acquisition_counter(self):
        pass

    def set_line3_to_strobe(self):
        pass
        
    def set_line3_to_exposure_active(self):
        pass
        
        
class Camera_Simulation(object):
    
    def __init__(self,sn=None):
        # many to be purged
        self.sn = sn
        self.device_info_list = None
        self.device_index = 0
        self.camera = None
        self.is_color = None
        self.gamma_lut = None
        self.contrast_lut = None
        self.color_correction_param = None

        self.exposure_time = 0
        self.analog_gain = 0
        self.frame_ID = -1
        self.frame_ID_software = -1
        self.frame_ID_offset_hardware_trigger = 0
        self.timestamp = 0

        self.image_locked = False
        self.current_frame = None

        self.callback_is_enabled = False
        self.callback_was_enabled_before_autofocus = False
        self.callback_was_enabled_before_multipoint = False

        self.GAIN_MAX = 24
        self.GAIN_MIN = 0
        self.GAIN_STEP = 1
        self.EXPOSURE_TIME_MS_MIN = 0.01
        self.EXPOSURE_TIME_MS_MAX = 4000

    def open(self,index=0):
        pass

    def set_callback(self,function):
        self.new_image_callback_external = function

    def enable_callback(self):
        self.callback_is_enabled = True

    def disable_callback(self):
        self.callback_is_enabled = False

    def open_by_sn(self,sn):
        pass

    def close(self):
        pass

    def set_exposure_time(self,exposure_time):
        pass

    def set_analog_gain(self,analog_gain):
        pass

    def get_awb_ratios(self):
        pass

    def set_wb_ratios(self, wb_r=None, wb_g=None, wb_b=None):
        pass

    def start_streaming(self):
        self.frame_ID_software = 0

    def stop_streaming(self):
        pass

    def set_pixel_format(self,format):
        print(format)

    def set_continuous_acquisition(self):
        pass

    def set_software_triggered_acquisition(self):
        pass

    def set_hardware_triggered_acquisition(self):
        pass

    def send_trigger(self):
        self.frame_ID = self.frame_ID + 1
        self.timestamp = time.time()
        if self.frame_ID == 1:
            self.current_frame = np.random.randint(255,size=(2000,2000),dtype=np.uint8)
            self.current_frame[901:1100,901:1100] = 200
        else:
            self.current_frame = np.roll(self.current_frame,10,axis=0)
            pass 
            # self.current_frame = np.random.randint(255,size=(768,1024),dtype=np.uint8)
        if self.new_image_callback_external is not None:
            self.new_image_callback_external(self)

    def read_frame(self):
        return self.current_frame

    def _on_frame_callback(self, user_param, raw_image):
        pass

    def set_ROI(self,offset_x=None,offset_y=None,width=None,height=None):
        pass

    def reset_camera_acquisition_counter(self):
        pass

    def set_line3_to_strobe(self):
        pass

    def set_line3_to_exposure_active(self):
        pass