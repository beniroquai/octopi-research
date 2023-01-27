# qt libraries
from qtpy.QtCore import QObject, Signal, Qt # type: ignore
from qtpy.QtWidgets import QMainWindow, QWidget, QGridLayout, QDesktopWidget, QVBoxLayout, QLabel, QApplication

from control._def import *

from queue import Queue
from threading import Thread, Lock
import numpy
import pyqtgraph as pg

from typing import Optional, List, Union, Tuple

from control.core import ConfigurationManager

class ImageDisplay(QObject):

    image_to_display = Signal(numpy.ndarray)

    def __init__(self):
        QObject.__init__(self)
        self.queue = Queue(10) # max 10 items in the queue
        self.image_lock = Lock()
        self.stop_signal_received = False
        self.thread:Thread = Thread(target=self.process_queue)
        self.thread.start()
        
    def process_queue(self):
        while True:
            # stop the thread if stop signal is received
            if self.stop_signal_received:
                return
            # process the queue
            try:
                [image,] = self.queue.get(timeout=0.1)
                self.image_lock.acquire(True)
                self.image_to_display.emit(image)
                QApplication.processEvents()
                self.image_lock.release()
                self.queue.task_done()
            except:
                pass

    def enqueue(self,image):
        try:
            self.queue.put_nowait([image,])
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


class ImageDisplayWindow(QMainWindow):

    def __init__(self, window_title='', draw_crosshairs = False, show_LUT=False):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint) # type: ignore
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint) # type: ignore
        self.widget = QWidget()
        self.show_LUT = show_LUT

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
        else:
            self.graphics_widget.img = pg.ImageItem(border='w')
            self.graphics_widget.view.addItem(self.graphics_widget.img)
            max_state=[[-1011.6942184540692, 4011.694218454069], [-147.79939172464378, 3147.799391724644]] # furthest zoomed out
            min_state=[[1105.2084826209198, 1163.5473663736475], [1401.9018607761034, 1440.1751411998673]] # furthest zoomed in
            ((max_lowerx,max_upperx),(max_lowery,max_uppery))=max_state
            ((min_lowerx,min_upperx),(min_lowery,min_uppery))=min_state

            # restrict zooming and moving (part 2 of 2)
            self.graphics_widget.view.setLimits(
                xMin=max_lowerx,
                xMax=max_upperx,
                yMin=max_lowery,
                yMax=max_uppery,

                minXRange=min_upperx-min_lowerx,
                maxXRange=max_upperx-max_lowerx,
                minYRange=min_uppery-min_lowery,
                maxYRange=max_uppery-max_lowery,
            )

        ## Create ROI
        self.roi_pos = (500,500)
        self.roi_size = pg.Point(500,500)
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
        self.image_offset = numpy.array([0, 0])

        ## Layout
        layout = QGridLayout()
        if self.show_LUT:
            layout.addWidget(self.graphics_widget.view, 0, 0) 
        else:
            layout.addWidget(self.graphics_widget, 0, 0)
        self.widget.setLayout(layout)
        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget()
        width = int(min(desktopWidget.height()*0.9,1000)) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    def display_image(self,image):
        """ display image in the respective widget """
        kwargs={
            'autoLevels':False, # disable automatically scaling the image pixel values (scale so that the lowest pixel value is pure black, and the highest value if pure white)
        }
        if image.dtype==numpy.float32:
            self.graphics_widget.img.setImage(image,levels=(0.0,1.0),**kwargs)
        else:
            self.graphics_widget.img.setImage(image,**kwargs)

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
        return numpy.array([xmin, ymin, width, height])


class ImageArrayDisplayWindow(QMainWindow):

    def __init__(self, configurationManager:ConfigurationManager, window_title=''):
        super().__init__()
        self.setWindowTitle(window_title)
        self.setWindowFlags(self.windowFlags() | Qt.CustomizeWindowHint) # type: ignore
        self.setWindowFlags(self.windowFlags() & ~Qt.WindowCloseButtonHint) # type: ignore
        self.widget = QWidget()
        self.configurationManager=configurationManager

        # interpret image data as row-major instead of col-major
        pg.setConfigOptions(imageAxisOrder='row-major')

        self.set_image_displays({
            11:0,
            12:1,
            14:2,
            13:3,
            15:4,

            0:6,
            1:7,
            2:8,
        },num_rows=3,num_columns=3)

        self.setCentralWidget(self.widget)

        # set window size
        desktopWidget = QDesktopWidget()
        width = int(min(desktopWidget.height()*0.9,1000)) #@@@TO MOVE@@@#
        height = width
        self.setFixedSize(width,height)

    @TypecheckFunction
    def set_image_displays(self,channel_mappings:Dict[int,int],num_rows:int,num_columns:int):
        reverse_channel_mappings={
            value:key
            for key,value
            in channel_mappings.items()
        }
            
        self.num_image_displays=num_rows*num_columns
        self.channel_mappings=channel_mappings
        self.graphics_widgets=[]
        image_display_layout = QGridLayout()

        assert num_rows*num_columns>=self.num_image_displays

        # restrict zooming and moving range so that image is always in view  (part 1 of 2)
        max_state=[[-1011.6942184540692, 4011.694218454069], [-147.79939172464378, 3147.799391724644]] # furthest zoomed out
        min_state=[[1105.2084826209198, 1163.5473663736475], [1401.9018607761034, 1440.1751411998673]] # furthest zoomed in
        ((max_lowerx,max_upperx),(max_lowery,max_uppery))=max_state
        ((min_lowerx,min_upperx),(min_lowery,min_uppery))=min_state

        for i in range(self.num_image_displays):
            next_graphics_widget = pg.GraphicsLayoutWidget()
            next_graphics_widget.view = next_graphics_widget.addViewBox()
            next_graphics_widget.view.setAspectLocked(True)
            next_graphics_widget.img = pg.ImageItem(border='w')
            next_graphics_widget.view.addItem(next_graphics_widget.img)

            # restrict zooming and moving (part 2 of 2)
            next_graphics_widget.view.setLimits(
                xMin=max_lowerx,
                xMax=max_upperx,
                yMin=max_lowery,
                yMax=max_uppery,

                minXRange=min_upperx-min_lowerx,
                maxXRange=max_upperx-max_lowerx,
                minYRange=min_uppery-min_lowery,
                maxYRange=max_uppery-max_lowery,
            )

            # link all views together so that each image view shows the same region
            if i>0:
                next_graphics_widget.view.setXLink(self.graphics_widgets[0].view)
                next_graphics_widget.view.setYLink(self.graphics_widgets[0].view)

            next_graphics_widget_wrapper=QVBoxLayout()
            
            if i in reverse_channel_mappings:
                illumination_source_code=reverse_channel_mappings[i]

                for c in self.configurationManager.configurations:
                    if c.illumination_source==illumination_source_code:
                        channel_name=c.name

            else:
                channel_name="<intentionally empty>"

            next_graphics_widget_wrapper.addWidget(QLabel(channel_name))
            next_graphics_widget_wrapper.addWidget(next_graphics_widget)

            row=i//num_columns
            column=i%num_columns
            image_display_layout.addLayout(next_graphics_widget_wrapper, row, column)

            self.graphics_widgets.append(next_graphics_widget)

        # all views are linked, to it's enough to set the (initial) view range on a single view
        self.graphics_widgets[0].view.setRange(xRange=(max_lowerx,max_upperx),yRange=(max_lowery,max_uppery))

        self.widget.setLayout(image_display_layout)

    def display_image(self,image,channel_index:int):
        #print(f"{self.graphics_widgets[0].view.getState()['viewRange']=}")

        # display image, flipped across x (to counteract the displaying of the image as flipped across x)
        self.graphics_widgets[self.channel_mappings[channel_index]].img.setImage(image[::-1,:],autoLevels=False)
