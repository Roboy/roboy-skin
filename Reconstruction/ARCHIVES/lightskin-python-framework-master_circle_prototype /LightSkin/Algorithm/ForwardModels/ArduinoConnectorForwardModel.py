import math

import re
from threading import Thread

import serial
from ...LightSkin import ForwardModel, LightSkin, EventHook


class ArduinoConnectorForwardModel(ForwardModel):
    """ Connects to an Arduino running the Arduino Connector Script on the given port with the given baudrate
        Parses the input in a new thread and updates its values accordingly.
        After each full received frame, the onUpdate is triggered. """

    sampleDistance = 0.125
    MAX_VALUE = 1024

    def __init__(self, ls: LightSkin, port: str, baudrate: int):
        super().__init__(ls)

        """ Instanzattribut onUpdate vom Typ EventHook. EventHook Objekte besitzen eine Liste aus Callablen und eine
           Methode um allen Callables der Liste die selben Parameter zu übergeben und die Returnwerte wiederzugeben."""
        self.onUpdate: EventHook = EventHook()

        """for: so oft wie es LEDs gibt. Erstellt _sensorValues: eine 12x12 Matrix initialisiert mit 1.0.
           Sensoren mal LEDs Matrix"""
        self._sensorValues = []
        for i in range(len(self.ls.LEDs)):
            self._sensorValues.append([1.0] * len(self.ls.sensors))

        """Thread objekt wird erstellt _readerThread. Target ist die Funktion, die parallel ausgeführt werden soll. Daemon
           bedeutet, dass der Thread aufhört zu laufen, wenn das hauptprogramm geschlossen wird. """
        self._readerThread = Thread(target=self._readLoop, daemon=True)


        """Was bringt _readerThreadRun? Warum PARITY_NONE? Warum STOPBITS_ONE und EIGHTBITS? timeout apparently needed
           for the readline method.
           serial.Serial(port=...) opens port, so that we can write (read) from it. Auf 45 wird thread gestartet.
           Das heißt _readLoop wird ausgeführt."""
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

    """bei readline werden erst die sensorwerte der ersten LED alle ausgelesen, dann der zweiten LED etc.
       In einer Schleife solange self._readerThreadRun war ist: Zuerst wird nach der Startzeile mit Snapshot gesurcht
       und die anzahl der Leds und Sensoren aus der serial verbindung mit der Anzahl der LEds und Sensoren aus
       dem CSV file abgeglichen. falls die jeweilige Anzahl übereinstimmt, werden die Sensorwerte durch 1024 geteilt, 
       und in die LED Sensor Matrix _sensorValues geschrieben. Dann wird self.onUpdate() aufgerufen, was macht das?"""

    def _readLoop(self):
        print('Read Loop started')
        while self._readerThreadRun:
            line = self.ser.readline()
            match = re.match(b'Snapshot: ([0-9]+),([0-9]+)', line)
            if match is not None:
                leds = int(match.group(1))
                sensors = int(match.group(2))
                if leds != len(self.ls.LEDs) or sensors != len(self.ls.sensors):
                    print("Received wrong amount of sensor values: %i / %i; expected %i / %i" % (
                    leds, sensors, len(self.ls.LEDs), len(self.ls.sensors)))
                else:
                    try:
                        for l in range(leds):
                            line = self.ser.readline()
                            vals = line.split(b',')
                            for s in range(sensors):
                                val = float(vals[s]) / self.MAX_VALUE if s < len(vals) else 0.0
                                self._sensorValues[l][s] = min(1.0, max(0.0, val))
                        print("received data")
                        self.onUpdate() # führt die Callables ohne Argumente in der Callableliste aus
                        # print("print self.onUpdate()", self.onUpdate())
                    except Exception as e:
                        print(e)
        print('Read Loop finished')

    def measureLEDAtPoint(self, x: float, y: float, led: int = -1) -> float:
        # No measurement possible
        return 0.0

    def getSensorValue(self, sensor: int, led: int = -1) -> float:
        # falls von visualizer, LightSkinGridView aufgerufen, if fall kommt nicht vor!
        if led < 0:
            led = self.ls.selectedLED
        return self._sensorValues[led][sensor]
