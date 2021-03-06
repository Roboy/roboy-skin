import re
from threading import Thread

import serial
from ...LightSkin import ForwardModel, LightSkin, EventHook


class ArduinoConnectorForwardModel(ForwardModel):
    """
        This class enables the communication to an Arduino running the Arduino Connector Script
        on the given port with the given baudrate.
        It parses the input in a new thread and updates its values accordingly.
        After reading all LEDs x Sensors combinations, the onUpdate is triggered.
    """

    MAX_VALUE = 1023

    def __init__(self, ls: LightSkin, port: str, baudrate: int):
        super().__init__(ls)

        self.onUpdate: EventHook = EventHook()

        self._sensorValues = []
        for i in range(len(self.ls.LEDs)):
            self._sensorValues.append([1.0] * len(self.ls.sensors))

        self._readerThread = Thread(target=self._readLoop, daemon=True)
        self._readerThreadRun = False

        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=10)
        self._readerThreadRun = True
        self._readerThread.start()

    def __del__(self):
        if self._readerThreadRun:
            self._readerThreadRun = False
            self._readerThread.join()
        try:
            self.ser.close()
        except Exception:
            pass

    def _readLoop(self):
        """
            This method reads the Serial data input from the Arduino.
            The parsing is triggered when a line 'Snapshot x,x' is read.
            Every row corresponds to the measurements of each sensor when a specific LED is on.
            All sensor values are normalized on a scale from 0-1.
        """

        print('Read Loop started')
        while self._readerThreadRun:
            line = self.ser.readline()
            match = re.match(b'Snapshot: ([0-9]+),([0-9]+)', line)
            if match is not None:
                leds = int(match.group(1))
                sensors = int(match.group(2))
                if leds != len(self.ls.LEDs) or sensors != len(self.ls.sensors):
                    print("Received wring amount of sensor values: %i / %i; expected %i / %i" % (
                    leds, sensors, len(self.ls.LEDs), len(self.ls.sensors)))
                else:
                    try:
                        for l in range(leds):
                            line = self.ser.readline()
                            vals = line.split(b',')
                            for s in range(sensors):
                                val = float(vals[s]) / self.MAX_VALUE if s < len(vals) else 0.0
                                self._sensorValues[l][s] = min(1.0, max(0.0, val))
                        #print("received data")
                        self.onUpdate()
                    except Exception as e:
                        print(e)
        print('Read Loop finished')

    # TODO: not elegant
    def measureLEDAtPoint(self, x: float, y: float, led: int = -1) -> float:
        """
            This method simulates sensor values and is not needed in this class.
        """
        # No measurement possible
        return 0.0

    def getSensorValue(self, sensor: int, led: int = -1) -> float:
        """
            This inherited method returns the measured value for sensor and LED combination (one element in matrix).
        """
        if led < 0:
            led = self.ls.selectedLED
        return self._sensorValues[led][sensor]
