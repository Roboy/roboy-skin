from ....LightSkin import Calibration


class SimpleIdealProportionalCalibration(Calibration):
    """ The ideal calibration values for the SimpleProportionalForwardModel """

    def __hash__(self):
        return hash(self.__class__.__name__)  # all instances of this class are equivalent

    def expectedSensorValue(self, sensor: int, led: int) -> float:
        ray = self.ls.getRayFromLEDToSensor(sensor, led)

        dist = max(ray.length, 0.1)
        val = 4 / dist

        return max(0.0, min(1.0, val))
