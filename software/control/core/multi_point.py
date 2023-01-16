# qt libraries
from qtpy.QtCore import QObject, Signal, QThread, Qt # type: ignore
from qtpy.QtWidgets import QApplication

from control._def import *

import os
import time
import cv2
from datetime import datetime

import json
import pandas as pd
import numpy

from typing import Optional, List, Union, Tuple, Callable

import control.camera as camera
from control.core import StreamingCamera, Configuration, NavigationController, LiveController, AutoFocusController, ConfigurationManager, ImageSaver #, LaserAutofocusController
from control.typechecker import TypecheckFunction

ENABLE_TQDM_STUFF:bool=False
if ENABLE_TQDM_STUFF:
    from tqdm import tqdm
else:
    def tqdm(iter,*args,**kwargs):
        return iter

from pathlib import Path

from control.gui import *

class AbortAcquisitionException(Exception):
    def __init__(self):
        super().__init__()


class MultiPointWorker(QObject):

    finished = Signal()
    image_to_display = Signal(numpy.ndarray)
    spectrum_to_display = Signal(numpy.ndarray)
    image_to_display_multi = Signal(numpy.ndarray,int)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)
    signal_new_acquisition=Signal(str)

    def __init__(self,
        multiPointController,
        scan_coordinates:Tuple[List[str],List[Tuple[float,float]]]
    ):
        super().__init__()
        self.multiPointController:MultiPointController = multiPointController

        # copy all (relevant) fields to unlock multipointcontroller on thread start
        self.camera = self.multiPointController.camera
        self.microcontroller = self.multiPointController.microcontroller
        self.navigation = self.multiPointController.navigation
        self.liveController = self.multiPointController.liveController
        self.autofocusController = self.multiPointController.autofocusController
        self.laserAutofocusController = self.multiPointController.laserAutofocusController
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
        self.do_reflection_af= self.multiPointController.do_reflection_af
        self.crop_width = self.multiPointController.crop_width
        self.crop_height = self.multiPointController.crop_height
        self.counter = self.multiPointController.counter
        self.selected_configurations = self.multiPointController.selected_configurations
        self.grid_mask=self.multiPointController.grid_mask
        self.output_path:str=self.multiPointController.output_path

        if not self.grid_mask is None:
            assert len(self.grid_mask)==self.NY
            assert len(self.grid_mask[0])==self.NX

        self.reflection_af_initialized = self.multiPointController.laserAutofocusController.is_initialized and not self.multiPointController.laserAutofocusController.x_reference is None

        self.timestamp_acquisition_started = self.multiPointController.timestamp_acquisition_started
        self.time_point:int = 0

        self.scan_coordinates_name,self.scan_coordinates_mm = scan_coordinates

    def run(self):
        try:
            while self.time_point < self.Nt:
                # continous acquisition
                if self.dt == 0.0:
                    QApplication.processEvents()
                    self.run_single_time_point()
                    QApplication.processEvents()

                    if self.multiPointController.abort_acqusition_requested:
                        raise AbortAcquisitionException()

                    self.time_point = self.time_point + 1

                # timed acquisition
                else:
                    QApplication.processEvents()
                    self.run_single_time_point()
                    QApplication.processEvents()

                    if self.multiPointController.abort_acqusition_requested:
                        raise AbortAcquisitionException()

                    if self.Nt==1:
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
                        
        except AbortAcquisitionException:
            pass
            
        self.finished.emit()
        QApplication.processEvents()

        print("\nfinished multipoint acquisition\n")

    def perform_software_autofocus(self):
        """ run software autofocus to focus on current fov """

        configuration_name_AF = MACHINE_CONFIG.MUTABLE_STATE.MULTIPOINT_AUTOFOCUS_CHANNEL
        config_AF = next((config for config in self.configurationManager.configurations if config.name == configuration_name_AF))
        self.signal_current_configuration.emit(config_AF)
        self.autofocusController.autofocus()
        self.autofocusController.wait_till_autofocus_has_completed()

    def image_config(self,config:Configuration,saving_path:str):
        """ take image for specified configuration and save to specified path """

        if 'USB Spectrometer' in config.name:
            raise Exception("usb spectrometer not supported")

        class Timer:
            def __init__(self,name:str):
                self.name=name

            def __enter__(self):
                self.start_time=time.monotonic()
                QApplication.processEvents()
            def __exit__(self,_a,_b,_c):
                pass#print(f"{self.name}: {((time.monotonic()-self.start_time)*1000):.3f}ms")

        with Timer("total"):
            # move to channel specific offset (if required)
            target_um=config.channel_z_offset or 0.0
            um_to_move=target_um-self.movement_deviation_from_focusplane
            if numpy.abs(um_to_move)>MACHINE_CONFIG.LASER_AUTOFOCUS_TARGET_MOVE_THRESHOLD_UM:
                #print(f"moving to relative offset {target_um}µm")
                self.movement_deviation_from_focusplane=target_um
                self.navigation.move_z(um_to_move/1000-self.microcontroller.clear_z_backlash_mm,wait_for_completion={})
                self.navigation.move_z(self.microcontroller.clear_z_backlash_mm,wait_for_completion={}) # dont wait for stabilization. hopefully movement is small enough to not require this

            image = self.liveController.snap(config,crop=True,override_crop_height=self.crop_height,override_crop_width=self.crop_width)

            # process the image -  @@@ to move to camera
            self.image_to_display.emit(image)
            self.image_to_display_multi.emit(image,config.illumination_source)

            QApplication.processEvents()
                
            if self.camera.is_color:
                if 'BF LED matrix' in config.name:
                    if MACHINE_CONFIG.MUTABLE_STATE.MULTIPOINT_BF_SAVING_OPTION == BrightfieldSavingMode.RAW and image.dtype!=numpy.uint16:
                        image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)
                    elif MACHINE_CONFIG.MUTABLE_STATE.MULTIPOINT_BF_SAVING_OPTION == BrightfieldSavingMode.RGB2GRAY:
                        image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
                    elif MACHINE_CONFIG.MUTABLE_STATE.MULTIPOINT_BF_SAVING_OPTION == BrightfieldSavingMode.GREEN_ONLY:
                        image = image[:,:,1]
                else:
                    image = cv2.cvtColor(image,cv2.COLOR_RGB2BGR)

            with Timer("save"):
                ImageSaver.save_image(path=saving_path,image=numpy.asarray(image),file_format=Acquisition.IMAGE_FORMAT)

                QApplication.processEvents()

        self.signal_new_acquisition.emit('c')

    def image_well_at_position(self,x:int,y:int,coordinate_name:str):

        j=x # todo actually rename the variables in this code
        i=y # todo actually rename the variables in this code

        ret_coords=[]

        # autofocus
        if self.do_reflection_af == False:
            # perform AF only when (not taking z stack) or (doing z stack from center)
            if ( (self.NZ == 1) or MACHINE_CONFIG.Z_STACKING_CONFIG == 'FROM CENTER' ) and (self.do_autofocus) and (self.FOV_counter % Acquisition.NUMBER_OF_FOVS_PER_AF == 0):
                self.perform_software_autofocus()
        else:
            # first FOV
            if self.reflection_af_initialized==False:
                # initialize the reflection AF
                self.laserAutofocusController.initialize_auto()
                # do contrast AF for the first FOV
                if ( (self.NZ == 1) or MACHINE_CONFIG.Z_STACKING_CONFIG == 'FROM CENTER' ) and (self.do_autofocus) and (self.FOV_counter==0):
                    self.perform_software_autofocus()
                # set the current plane as reference
                self.laserAutofocusController.set_reference(z_pos_mm=0.0) # z pos does not matter here
                self.reflection_af_initialized = True
            else:
                self.laserAutofocusController.move_to_target(0.0)

        QApplication.processEvents()

        if (self.NZ > 1):
            # move to bottom of the z stack
            if MACHINE_CONFIG.Z_STACKING_CONFIG == 'FROM CENTER':
                base_z=-self.deltaZ_usteps*round((self.NZ-1)/2)
                self.navigation.move_z_usteps(base_z,wait_for_completion={})
            # maneuver for achiving uniform step size and repeatability when using open-loop control
            self.navigation.move_z_usteps(-self.microcontroller.clear_z_backlash_mm,wait_for_completion={})
            self.navigation.move_z_usteps(self.microcontroller.clear_z_backlash_mm,wait_for_completion={},wait_for_stabilization=True)

        QApplication.processEvents()

        # z-stack
        for k in range(self.NZ):
            if self.num_positions_per_well>1:
                _=next(self.well_tqdm_iter,0)

            if self.NZ > 1:
                file_ID = f'{coordinate_name}_z{k}'
            else:
                file_ID = f'{coordinate_name}'
            # metadata = dict(x = self.navigation.x_pos_mm, y = self.navigation.y_pos_mm, z = self.navigation.z_pos_mm)
            # metadata = json.dumps(metadata)

            self.movement_deviation_from_focusplane=0.0

            QApplication.processEvents()

            # iterate through selected modes
            for config in tqdm(self.selected_configurations,desc="channel",unit="channel",leave=False):
                saving_path = os.path.join(self.current_path, file_ID + '_' + str(config.name).replace(' ','_'))

                if self.multiPointController.abort_acqusition_requested:
                    raise AbortAcquisitionException()
                    
                time_start=time.monotonic()
                self.image_config(config=config,saving_path=saving_path)
                print(f"imaging config {config.name} took {(time.monotonic()-time_start):.2f}s even though exposure time is only {config.exposure_time:.2f}s")

            QApplication.processEvents()

            # add the coordinate of the current location
            ret_coords.append({
                'i':i,'j':j,'k':k,
                'x (mm)':self.navigation.x_pos_mm,
                'y (mm)':self.navigation.y_pos_mm,
                'z (um)':self.navigation.z_pos_mm*1000
            })

            # register the current fov in the navigationViewer 
            self.signal_register_current_fov.emit(self.navigation.x_pos_mm,self.navigation.y_pos_mm)

            QApplication.processEvents()

            # check if the acquisition should be aborted
            if self.multiPointController.abort_acqusition_requested:
                raise AbortAcquisitionException()

            if self.NZ > 1:
                # move z
                if k < self.NZ - 1:
                    self.navigation.move_z_usteps(self.deltaZ_usteps,wait_for_completion={},wait_for_stabilization=True)
                    self.on_abort_dz_usteps = self.on_abort_dz_usteps + self.deltaZ_usteps

            self.signal_new_acquisition.emit('z')

            QApplication.processEvents()
        
        if self.NZ > 1:
            # move z back
            latest_offset=-self.deltaZ_usteps*(self.NZ-1)
            if MACHINE_CONFIG.Z_STACKING_CONFIG == 'FROM CENTER':
                latest_offset+=self.deltaZ_usteps*round((self.NZ-1)/2)

            self.on_abort_dz_usteps += latest_offset
            self.navigation.move_z_usteps(latest_offset,wait_for_completion={})

        QApplication.processEvents()

        # update FOV counter
        self.FOV_counter = self.FOV_counter + 1

        return ret_coords

    def run_single_time_point(self):
        with StreamingCamera(self.camera):

            # disable joystick button action
            self.navigation.enable_joystick_button_action = False

            self.FOV_counter = 0

            print('multipoint acquisition - time point ' + str(self.time_point+1))

            if self.Nt > 1:
                # for each time point, create a new folder
                current_path = str(self.output_path/self.time_point)
                self.current_path=current_path
                os.mkdir(current_path)
            else:
                # only one time point, save it directly in the experiment folder
                self.current_path=str(self.output_path)

            # create a dataframe to save coordinates
            coordinates_pd = pd.DataFrame(columns = ['i', 'j', 'k', 'x (mm)', 'y (mm)', 'z (um)'])

            if not self.grid_mask is None:
                self.num_positions_per_well=numpy.sum(self.grid_mask)*self.NZ
            else:
                self.num_positions_per_well=self.NX*self.NY*self.NZ

            # each region is a well
            n_regions = len(self.scan_coordinates_name)
            for coordinate_id in range(n_regions) if n_regions==1 else tqdm(range(n_regions),desc="well on plate",unit="well"):
                coordinate_mm = self.scan_coordinates_mm[coordinate_id]
                base_coordinate_name = self.scan_coordinates_name[coordinate_id]

                base_x=coordinate_mm[0]-self.deltaX*(self.NX-1)/2
                base_y=coordinate_mm[1]-self.deltaY*(self.NY-1)/2

                # move to the specified coordinate
                self.navigation.move_x_to(base_x,wait_for_completion={},wait_for_stabilization=True)
                self.navigation.move_y_to(base_y,wait_for_completion={},wait_for_stabilization=True)

                self.x_scan_direction = 1 # will be flipped between {-1, 1} to alternate movement direction in rows within the same well (instead of moving to same edge of row and wasting time by doing so)
                self.on_abort_dx_usteps = 0
                self.on_abort_dy_usteps = 0
                self.on_abort_dz_usteps = 0
                z_pos = self.navigation.z_pos

                # z stacking config
                if MACHINE_CONFIG.Z_STACKING_CONFIG == 'FROM TOP':
                    self.deltaZ_usteps = -abs(self.deltaZ_usteps)

                if self.num_positions_per_well>1:
                    # show progress when iterating over all well positions (do not differentiatte between xyz in this progress bar, it's too quick for that)
                    well_tqdm=tqdm(range(self.num_positions_per_well),desc="pos in well", unit="pos",leave=False)
                    self.well_tqdm_iter=iter(well_tqdm)

                # along y
                for i in range(self.NY):

                    QApplication.processEvents()

                    self.FOV_counter = 0 # so that AF at the beginning of each new row

                    # along x
                    for j in range(self.NX):

                        QApplication.processEvents()

                        j_actual = j if self.x_scan_direction==1 else self.NX-1-j
                        site_index = 1 + j_actual + i * self.NX
                        coordinate_name = f'{base_coordinate_name}_s{site_index}_x{j_actual}_y{i}' # _z{k} added later (if needed)

                        image_position=True

                        if not self.grid_mask is None:
                            image_position=self.grid_mask[i][j_actual]

                        if image_position:
                            try:
                                QApplication.processEvents()
                                imaged_coords_dict_list=self.image_well_at_position(
                                    x=j,y=i,
                                    coordinate_name=coordinate_name,
                                )
                                QApplication.processEvents()

                                coordinates_pd = pd.concat([
                                    coordinates_pd,
                                    pd.DataFrame(imaged_coords_dict_list)
                                ])
                            except AbortAcquisitionException:
                                if ENABLE_TQDM_STUFF:
                                    self.well_tqdm_iter.close()

                                self.liveController.turn_off_illumination()
                                self.navigation.move_x_usteps(-self.on_abort_dx_usteps,wait_for_completion={})
                                self.navigation.move_y_usteps(-self.on_abort_dy_usteps,wait_for_completion={})
                                self.navigation.move_z_usteps(-self.on_abort_dz_usteps,wait_for_completion={})

                                coordinates_pd.to_csv(os.path.join(self.current_path,'coordinates.csv'),index=False,header=True)
                                self.navigation.enable_joystick_button_action = True

                                raise AbortAcquisitionException()

                        self.signal_new_acquisition.emit('x')

                        if self.NX > 1:
                            # move x
                            if j < self.NX - 1:
                                self.navigation.move_x_usteps(self.x_scan_direction*self.deltaX_usteps,wait_for_completion={},wait_for_stabilization=True)
                                self.on_abort_dx_usteps = self.on_abort_dx_usteps + self.x_scan_direction*self.deltaX_usteps

                        QApplication.processEvents()

                    # instead of move back, reverse scan direction (12/29/2021)
                    self.x_scan_direction = -self.x_scan_direction

                    if self.NY > 1:
                        # move y
                        if i < self.NY - 1:
                            self.navigation.move_y_usteps(self.deltaY_usteps,wait_for_completion={},wait_for_stabilization=True)
                            self.on_abort_dy_usteps = self.on_abort_dy_usteps + self.deltaY_usteps

                    self.signal_new_acquisition.emit('y')

                # exhaust tqdm iterator
                if self.num_positions_per_well>1:
                    _=next(self.well_tqdm_iter,0)

                QApplication.processEvents()

                if n_regions == 1:
                    # only move to the start position if there's only one region in the scan
                    if self.NY > 1:
                        # move y back
                        self.navigation.move_y_usteps(-self.deltaY_usteps*(self.NY-1),wait_for_completion={},wait_for_stabilization=True)
                        self.on_abort_dy_usteps = self.on_abort_dy_usteps - self.deltaY_usteps*(self.NY-1)

                    # move x back at the end of the scan
                    if self.x_scan_direction == -1:
                        self.navigation.move_x_usteps(-self.deltaX_usteps*(self.NX-1),wait_for_completion={},wait_for_stabilization=True)

                    # move z back
                    self.navigation.microcontroller.move_z_to_usteps(z_pos)
                    self.navigation.microcontroller.wait_till_operation_is_completed()

                QApplication.processEvents()

            coordinates_pd.to_csv(os.path.join(self.current_path,'coordinates.csv'),index=False,header=True)
            self.navigation.enable_joystick_button_action = True

            self.signal_new_acquisition.emit('t')

class MultiPointController(QObject):

    acquisitionStarted = Signal()
    acquisitionFinished = Signal()
    image_to_display = Signal(numpy.ndarray)
    image_to_display_multi = Signal(numpy.ndarray,int)
    spectrum_to_display = Signal(numpy.ndarray)
    signal_current_configuration = Signal(Configuration)
    signal_register_current_fov = Signal(float,float)

    #@TypecheckFunction
    def __init__(self,
        camera:camera.Camera,
        navigationController:NavigationController,
        liveController:LiveController,
        autofocusController:AutoFocusController,
        laserAutofocusController,#:LaserAutofocusController,
        configurationManager:ConfigurationManager,
        parent:Optional[Any]=None,
    ):
        QObject.__init__(self)

        self.camera = camera
        self.microcontroller = navigationController.microcontroller # to move to gui for transparency
        self.navigation = navigationController
        self.liveController = liveController
        self.autofocusController = autofocusController
        self.laserAutofocusController = laserAutofocusController
        self.configurationManager = configurationManager

        self.NX:int = DefaultMultiPointGrid.DEFAULT_Nx
        self.NY:int = DefaultMultiPointGrid.DEFAULT_Ny
        self.NZ:int = DefaultMultiPointGrid.DEFAULT_Nz
        self.Nt:int = DefaultMultiPointGrid.DEFAULT_Nt
        self.deltaX:float = DefaultMultiPointGrid.DEFAULT_DX_MM
        self.deltaY:float = DefaultMultiPointGrid.DEFAULT_DY_MM
        self.deltaZ:float = DefaultMultiPointGrid.DEFAULT_DZ_MM
        self.deltat:float = DefaultMultiPointGrid.DEFAULT_DT_S

        self.do_autofocus:bool = False
        self.do_reflection_af:bool = False

        self.crop_width = Acquisition.CROP_WIDTH
        self.crop_height = Acquisition.CROP_HEIGHT
        
        self.counter:int = 0
        self.output_path: Optional[str] = None
        self.selected_configurations = []
        self.thread:Optional[QThread]=None
        self.parent = parent

        # set some default values to avoid introducing new attributes outside constructor
        self.abort_acqusition_requested = False
        self.configuration_before_running_multipoint:Optional[Configuration] = None
        self.liveController_was_live_before_multipoint = False
        self.camera_callback_was_enabled_before_multipoint = False

    @property
    def autofocus_channel_name(self)->str:
        return MACHINE_CONFIG.MUTABLE_STATE.MULTIPOINT_AUTOFOCUS_CHANNEL

    @property
    def deltaX_usteps(self)->int:
        return round(self.deltaX/self.microcontroller.mm_per_ustep_x)
    @property
    def deltaY_usteps(self)->int:
        return round(self.deltaY/self.microcontroller.mm_per_ustep_y)
    @property
    def deltaZ_usteps(self)->int:
        return round(self.deltaZ/self.microcontroller.mm_per_ustep_z)

    @TypecheckFunction
    def set_NX(self,N:int):
        self.NX = N
    @TypecheckFunction
    def set_NY(self,N:int):
        self.NY = N
    @TypecheckFunction
    def set_NZ(self,N:int):
        self.NZ = N
    @TypecheckFunction
    def set_Nt(self,N:int):
        self.Nt = N

    @TypecheckFunction
    def set_deltaX(self,delta_mm:float):
        self.deltaX = delta_mm
    @TypecheckFunction
    def set_deltaY(self,delta_mm:float):
        self.deltaY = delta_mm
    @TypecheckFunction
    def set_deltaZ(self,delta_mm:float):
        self.deltaZ = delta_mm
    @TypecheckFunction
    def set_deltat(self,delta_s:float):
        self.deltat = delta_s

    @TypecheckFunction
    def set_software_af_flag(self,flag:Union[int,bool]):
        if type(flag)==bool:
            self.do_autofocus=flag
        else:
            self.do_autofocus = bool(flag)
    @TypecheckFunction
    def set_laser_af_flag(self,flag:Union[int,bool]):
        if type(flag)==bool:
            self.do_reflection_af=flag
        else:
            self.do_reflection_af = bool(flag)

    @TypecheckFunction
    def set_crop(self,crop_width:int,crop_height:int):
        self.crop_width = crop_width
        self.crop_height = crop_height

    @TypecheckFunction
    def prepare_folder_for_new_experiment(self,output_path:str,complete_experiment_data:dict):
        # try generating unique experiment ID (that includes current timestamp) until successfull
        while True:
            now = datetime.now()
            now = now.replace(microsecond=0)  # setting microsecond=0 makes it not show up in isoformat
            now_str = now.isoformat(sep='_')  # separate with date and time with _ (rather than T or space)

            experiment_pathname = output_path + '_' + now_str
            experiment_pathname = experiment_pathname.replace(' ', '_')
            experiment_pathname = experiment_pathname.replace(':', '.') # windows does not support colon in filenames
            experiment_path=Path(experiment_pathname)
            if experiment_path.exists():
                time.sleep(1) # wait until next second to get a unique experiment ID
            else:
                experiment_path.mkdir(parents=True) # create a new folder
                self.output_path = experiment_pathname
                break

        self.recording_start_time = time.time()

        # save different sets of config data to the experiment output directory

        # config : complete set of config used for the experiment
        complete_data_path = experiment_path / 'parameters.json'
        complete_data_path.write_text(json.encoder.JSONEncoder(indent=2).encode(complete_experiment_data))

    @TypecheckFunction
    def set_selected_configurations(self, selected_configurations_name:List[str]):
        self.selected_configurations = []
        for configuration_name in selected_configurations_name:
            self.selected_configurations.append(next((config for config in self.configurationManager.configurations if config.name == configuration_name)))
        
    @TypecheckFunction
    def run_experiment(self,
        well_selection:Tuple[List[str],List[Tuple[float,float]]],
        set_num_acquisitions_callback:Optional[Callable[[int],None]],
        on_new_acquisition:Optional[Callable[[str],None]],

        grid_mask:Optional[Any]=None,
        headless:bool=True
    )->Optional[QThread]:
        while not self.thread is None:
            time.sleep(0.05)
            self.thread.quit() # this should result in the self.thread.finished signal being sent, which calls self.on_thread_finished, which sets self.thread=None

        image_positions=well_selection

        num_wells=len(image_positions[0])
        if grid_mask is None:
            num_images_per_well=self.NX*self.NY*self.NZ*self.Nt
        else:
            num_images_per_well=numpy.sum(grid_mask)*self.NZ*self.Nt
        num_channels=len(self.selected_configurations)

        self.abort_acqusition_requested = False
        self.liveController_was_live_before_multipoint = False
        self.camera_callback_was_enabled_before_multipoint = False
        self.configuration_before_running_multipoint = self.liveController.currentConfiguration
        self.grid_mask=grid_mask

        if num_wells==0:
            warning_text="No wells have been selected, so nothing to acquire. Consider selecting some wells before starting the multi point acquisition."
            if headless:
                print(f"! warning: {warning_text} !")
            else:
                MessageBox(title="No wells selected",text=warning_text,mode="warning").run()
            self._on_acquisition_completed()
        elif num_channels==0:
            warning_text="No channels have been selected, so nothing to acquire. Consider selecting some channels before starting the multi point acquisition."
            if headless:
                print(f"! warning: {warning_text} !")
            else:
                MessageBox(title="No channels selected",text=warning_text,mode="warning").run()
            self._on_acquisition_completed()
        elif num_images_per_well==0:
            warning_text="Somehow no images would be acquired if acquisition were to start. Maybe all positions were de-selected in the grid mask?"
            if headless:
                print(f"! warning: {warning_text} !")
            else:
                MessageBox(title="No images to acquire?",text=warning_text,mode="warning").run()
            self._on_acquisition_completed()
        else:
            total_num_acquisitions=num_wells*num_images_per_well*num_channels
            print(f"start multipoint with {num_wells} wells, {num_images_per_well} images per well, {num_channels} channels, total={total_num_acquisitions} images (AF is {'on' if self.do_autofocus else 'off'})")

            if not set_num_acquisitions_callback is None:
                set_num_acquisitions_callback(total_num_acquisitions)
        
            # stop live
            if self.liveController.is_live:
                self.liveController.stop_live()

            self.acquisitionStarted.emit()

            # run the acquisition
            self.timestamp_acquisition_started = time.time()

            self.multiPointWorker = MultiPointWorker(self,image_positions)
            RUN_WORKER_ASYNC=True
            if RUN_WORKER_ASYNC:
                self.thread = QThread()
                self.multiPointWorker.moveToThread(self.thread)

                # connect signals and slots
                self.thread.started.connect(self.multiPointWorker.run)
                if not on_new_acquisition is None:
                    self.multiPointWorker.signal_new_acquisition.connect(on_new_acquisition)

                self.multiPointWorker.image_to_display.connect(self.slot_image_to_display)
                self.multiPointWorker.image_to_display_multi.connect(self.slot_image_to_display_multi)
                self.multiPointWorker.spectrum_to_display.connect(self.slot_spectrum_to_display)
                self.multiPointWorker.signal_current_configuration.connect(self.slot_current_configuration,type=Qt.BlockingQueuedConnection)
                self.multiPointWorker.signal_register_current_fov.connect(self.slot_register_current_fov)

                self.multiPointWorker.finished.connect(self.on_multipointworker_finished)

                self.thread.finished.connect(self.on_thread_finished)
                
                self.thread.start()

                return self.thread
            else:
                if not on_new_acquisition is None:
                    self.multiPointWorker.signal_new_acquisition.connect(on_new_acquisition)

                self.multiPointWorker.image_to_display.connect(self.slot_image_to_display)
                self.multiPointWorker.image_to_display_multi.connect(self.slot_image_to_display_multi)
                self.multiPointWorker.spectrum_to_display.connect(self.slot_spectrum_to_display)
                self.multiPointWorker.signal_current_configuration.connect(self.slot_current_configuration)#,type=Qt.BlockingQueuedConnection)
                self.multiPointWorker.signal_register_current_fov.connect(self.slot_register_current_fov)

                self.multiPointWorker.finished.connect(self._on_acquisition_completed)
                    
                self.multiPointWorker.run()

    def on_multipointworker_finished(self):
        self._on_acquisition_completed()
        self.multiPointWorker.deleteLater()
        self.thread.quit()

    def on_thread_finished(self):
        self.thread.quit()
        self.multiPointWorker=None
        self.thread=None

    def _on_acquisition_completed(self):
        # restore the previous selected mode
        if not self.configuration_before_running_multipoint is None:
            self.signal_current_configuration.emit(self.configuration_before_running_multipoint)
        
        # emit the acquisition finished signal to enable the UI
        self.acquisitionFinished.emit()

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

    @TypecheckFunction
    def grid_positions_for_well(self,well_center_x_mm:float,well_center_y_mm:float)->List[Tuple[float,float]]:
        """ get coordinates in mm of each grid position """

        coords=[]

        base_x=well_center_x_mm-self.deltaX*(self.NX-1)/2
        base_y=well_center_y_mm-self.deltaY*(self.NY-1)/2

        for i in range(self.NY):
            y=base_y+i*self.deltaY
            for j in range(self.NX):
                x=base_x+j*self.deltaX
                ##for k in range(self.NZ):
                    # todo z=???
                coords.append((x,y))

        return coords
