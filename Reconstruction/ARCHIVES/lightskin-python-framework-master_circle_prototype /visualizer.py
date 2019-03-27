#!/usr/bin/python3

import csv

import tkinter as tk

from LightSkin.Algorithm.RayInfluenceModels.DirectSampledRayGridInfluenceModel import DirectSampledRayGridInfluenceModel
from LightSkin.Algorithm.Reconstruction.LogarithmicLinSysOptimize2 import LogarithmicLinSysOptimize2
from LightSkin.LightSkin import LightSkin
from LightSkin.GUI import Views

# from SimpleProportionalForwardModel import SimpleProportionalForwardModel
from LightSkin.Algorithm.ForwardModels.ArduinoConnectorForwardModel import ArduinoConnectorForwardModel
from LightSkin.Algorithm.SimpleCalibration import SimpleCalibration
import serial.tools.list_ports


# Source: https://code.activestate.com/recipes/410687-transposing-a-list-of-lists-with-different-lengths/
def transposed(lists):
    if not lists:
        return []
    return list(map(lambda *row: list(row), *lists))


# Main Code

"""Searching for the right port and storing object of type SysFS in port. """

ports = serial.tools.list_ports.comports()
port = None
for p in ports:
    print('Checking port %s / %s' % (p.device, p.description))
    if "uino" in p.description.lower():  # Find "ardUINO" and "genUINO" boards
        port = p
        break

if port is None:
    print('Could not find a connected Arduino')
    exit(0)

print('Using the Arduino connected on:')
print(port.description + ' / ' + port.device)


"""Instantiating object of class LightSkin. 
   Stores:
   1) Coordinates of the sensors in a list of tuples:   sensors=[]
   2) Coordinates of the leds in a list of tuples:      LEDs=[]
   3) An object of type ValueMap:                       translucencyMap=None
   4) An object of type ForwardModel:                   forwardModel=None
   5) An object of type BackwardModel:                  backwardModel=None
   6) The currently selected sensor as integer:         _selectedSensor=-1
   7) The currently selected led as integer:            _selectedLED=-1
   8) An object of type EventHook:                      onChange=EventHook()      """

ls = LightSkin()


"""LOAD Sensor and LED coordinates from CSV and writes them in ls.sensors and ls.LEDs"""

with open('sensors.csv', 'r') as csvfile:
    read = csv.reader(csvfile)
    for r in read:
        s = (float(r[0]), float(r[1]))
        ls.sensors.append(s)
#
with open('leds.csv', 'r') as csvfile:
    read = csv.reader(csvfile)
    for r in read:
        s = (float(r[0]), float(r[1]))
        ls.LEDs.append(s)


""" Determines the dimension of the matrix of the Top View """

recResolution = 5


"""Instantiating object of type SimpleCalibration. Inherits from Calibration.
   Stores:
   1) The already created LightSkin object:                             ls = ls
   2) Status variable telling if calibrations has already happened:     isCalibrated = False
   3) A matrix of floats taken from the forward model and treated as 
      ground truth (when there is no force applied on prototype):       _calibration = []"""

calibration = SimpleCalibration(ls)


"""Instantiating object of type ArduinoConnectorForwardModel. Inherits from ForwardModel, which inherits from 
   Measurable.
   Stores:
   1) Class variable.                                                   sampleDistance = 0.125
   2) Class variable. Sensor measurements by arduino are in the range 
      of 0 - 1023:                                                      MAX_VALUE = 1024
   3) The already created LightSkin object:                             ls = ls
   4) An object of type EventHook.                                      onUpdate = EventHook()
   5) Sensor*LEDs matrix, which holds the constantly updated sensor 
      values coming from the arduino:                                   _sensorValues = constantly updated matrix
   6) An object of type Thread, which starts the update loop:           _readerThread = Thread(...)
   7) An object of type Serial:                                         ser = serial.Serial(...)
   8) Boolean that controls if update loop is active:                   _readerThreadRun = True
   
   Function:
   -> Starts thread that keep updating a matrix with sensor values coming from the prototype with arduino."""

arduinoConnector = ArduinoConnectorForwardModel(ls, port.device, 1000000)


"""Instantiating object of type LogarithmicLinSysOptimize2. Inherits from LogarithmicLinSysOptimize, which inherits
   from BackwardModel, which inherits from ValueMap, which inherits from MeasurableGrid, which inherits from 
   Measurable.
   Stores:
   1) The already created LightSkin object:                             ls = ls
   2) The already created calibration object:                           calibration = calibration
   3) An object of type ValueGridDefinition. It is 
      instantiated to hold the following info:
      startX, startY, endX, endY, width, height, cellsX, 
      cellsY, cellWidth, cellHeight.                                    gridDefinition = ...
   4) ???                                                               _tmpGrid = []
   5) ???                                                               _tmpGridWeights = []
   6) ???                                                               _bufGrid: List[List[float]] = []
   7) An object of type DirectSampledRayGridInfluenceModel.    
      Inherits from RayGridInfluenceModel. Stores an object
      of type ValueGridDefinition.                                      rayModel = DirectSampledRayGridInfluenceModel()
   8) Initializes attribute of object rayModel with the already
      created object gridDefinition.                                    rayModel.gridDefinition = self.gridDefinition
   9) ???                                                               _construct_hash = None
   10) ???                                                              _lgs_A: sparse.csr_matrix = None
   11) ???                                                              _lgs_b: List[float] = []
   12) ???                                                              _lgs_sol: List[float] = []
   13) Matrix of floats initialized with ones. Reconstruction values
       between 0 and 1 will go here:                                    grid = ..."""

backwardModel = LogarithmicLinSysOptimize2(ls,
                                           recResolution, recResolution,
                                           calibration,
                                           DirectSampledRayGridInfluenceModel())


"""Initializing of ls' attributes ls.forwardModel and ls.backwardModel."""

ls.forwardModel = arduinoConnector
ls.backwardModel = backwardModel

# print(ls.sensors)
# print(ls.LEDs)
# print(flush=True)

"""Wo wird pressure_colormap verwendet?"""

pressure_colormap = 'nipy_spectral'


"""Builds window."""

window = tk.Tk()
window.title('Light Skin Visualization')
window.minsize(900, 300)


"""Instantiating object of class LightSkinGridView. Inherits from tk.Frame.
   Stores:
   1) The LightSkin object that was created at the beginning.                       skin = ls
   2) A display function, that takes a value between 0 and 1 as
      argument and returns an object of type Color:                                 displayFunction = ...
   3) A list of the sensor buttons of the grid view:                                _sensors = [...]
   4) A list of the LED buttons of the grid view:                                   _leds = [...]
   5) Matrix of tuples. The tuples include a tk.Frame
      and a ToolTip object:                                                         _measurements = [...]
      
   _build() method fills the lists _sensors and _leds with Button objects. When a button is clicked, 
   self.skin.selectedSensor or self.skin.selectedLED changes to the number of the clicked sensor/led. 
   Afterwards the buttons and LEDs are placed on a grid on our LightSkinGridView object. The grid view
   frames and the correspondent tooltips are also being created and put on the LightSkinGridView object
   as well as in the _measurement matrix.

   6) A callable is being added to the Callable list of the EventHook object of the LightSkin object.
      skin.onChange += lambda *a, **kwa: self.after_idle(self.updateVisuals)
      
   updateVisuals() changes the colors of buttons in the lists: _sensors and _leds. The method also 
   changes the colors of the frames in the matrix _measurements depending on the value of skin.selectedLED 
   and skin.selectedSensor, the string of the ToolTip object and the color of the frame depending on the 
   value received from the prototype with arduino.
   
   The LightSkinGridView object: gridView is packed to the left of the tk.TK object: window."""


gridView = Views.LightSkinGridView(window, ls, width=400, height=400, highlightbackground='#aaa', highlightthickness=1,
                                   display_function=Views.Colorscales.MPColorMap('inferno', lambda x: x ** 0.3)
                                   )
gridView.pack(side=tk.LEFT)

# Builds the top view visualising where there is pressure on the skin
"""Instantiating object of class LightSkinTopView. Inherits from tk.Canvas.
   Stores:
   1) The given display function. Receiving float between 0 - 1, returning 
      Color object:                                                                 displayFunction = display_function
   2) The LightSkin object created at the beginning of the code:                    skin = skin
   
   From the ls.backwardModel we get the following information:
   3) The number of cells of the Top View in x direction:                           gridWidth = ...
   4) The number of cells of the Top View in y direction:                           gridHeight = ...
   5) Callable, expects a coordinate and returns the float value
      of the reconstruction algorithm:                                              measureFunction = ...
   
   There is code responsible for resizing the canvas and it's containing widgets that does not work yet.
   What does update() do????
   
   6) List of LED circle objects of the top view:                                   _leds = [...]
   7) List of sensor rectangle objects of the top view:                             _sensors = [...]
   8) Matrix recResolution*recResolution of reconstruction rectangles:              _grid= [...] 
   
   _draw() creates the objects 6),7),8), places and scales them on the canvas. Also adds functionality that
   when the sensor or led objects are clicked, the is selected led and sensor changes value.
   
   9) skin.onChange += lambda *a, **kwa: self.after_idle(self.updateVisuals) 
   
   updateVisuals() updates visuals for the to view."""

topViewReconstructed = Views.LightSkinTopView(window, ls, highlightbackground='#aaa', highlightthickness=1,
                                              width=500, height=500,
                                              measurable_grid=ls.backwardModel,
                                              display_function=Views.Colorscales.Grayscale(lambda x: x ** 2)  # MPColorMap(pressure_colormap)
                                              )
topViewReconstructed.pack(side=tk.RIGHT)

""" Initializes the matrix _calibration of the SimpleCalibration object with current values from the arduinoConnector's
    constantly updated _sensorValues matrix.
    
    backwardModel.calculate() ??? calculates the grid of reconstruction values in the backwardModel.
    
    ls.onChange('values') calls the two Callables in the __delegates list of the EventHook object onChange with the
    argument 'values'. ????? """

def onUpdate():
    if not calibration.isCalibrated:
        calibration.calibrate()
    backwardModel.calculate()
    ls.onChange('values')


arduinoConnector.onUpdate += onUpdate

window.mainloop()

# Main program logic follows:
# if __name__ == '__main__':
#
