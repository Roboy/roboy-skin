# Forward models
Forward models describe the sensor values for the current setup. We implement two forward models:

 * `ArduinoConnectorForwardModel.py`: reads real values from the prototype through a Serial connection with an Arduino
 * `SimpleProportionalForwardModel.py`: uses simulated values taken from the file translucency.csv
 
Each of the forward models use an own calibrating system. The `SimpleProportionalForwardModel.py` uses SimpleIdealProportionalCalibration.
The `ArduinoConnectorForwardModel.py` uses SimpleCalibration that takes a snapshot of the current sensor values and uses these as calibration values.
