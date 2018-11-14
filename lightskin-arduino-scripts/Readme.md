# Arduino Scripts for the Light-Skin project
The scripts are designed for an Arduino Mega that is connected to multiple LEDs on digital pins and multiple Light-To-Voltage sensors, eg. the TSL-251R, on analog pins.
Their goal is to measure the amount of light received from the LEDs at the sensors.


## Measure Translucency

This script measures the amount of light received at one specific sensor from one specific LED by turning the LED off and on and calculating the difference between the values.
An average is taken over 40 * 40 iterations.


## Light-Skin Arduino Adapter

This script measures the amount of received light from each LED for each sensor.
This is done by individually turning on the LEDs and measuring all the analog ins.
An average is taken over n measurements once again.
Values are also taken without any LEDs turned on and subtracted from all subsequent measurements to reduce the influence of static light.
The results are send via serial at 1 000 000 baud.