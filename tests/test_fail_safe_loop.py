"""Exercise the actual controller loop with invalid live speed/sensor inputs."""
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('fault_loop',Path(__file__).resolve().parents[1]/'controllers/autonomous_car/autonomous_car.py')
car=importlib.util.module_from_spec(spec);spec.loader.exec_module(car)

class FailSafeLoop(unittest.TestCase):
    def test_invalid_speed_or_range_cannot_accelerate_and_keeps_brake(self):
        for speed,distance in [(float('nan'),20.),(0.,float('nan')),(float('inf'),20.)]:
            with self.subTest(speed=speed,distance=distance):
                class Sensor:
                    def enable(self,_):pass
                    def getValue(self):return distance
                class Driver:
                    def __init__(self):self.steps=0;self.commands=[];self.brakes=[]
                    def getDevice(self,name):return Sensor() if 'distance' in name else None
                    def setSteeringAngle(self,_):pass
                    def getCurrentSpeed(self):return speed
                    def getTime(self):return .032
                    def step(self):self.steps+=1;return 0 if self.steps==1 else -1
                    def setCruisingSpeed(self,value):self.commands.append(value)
                    def setBrakeIntensity(self,value):self.brakes.append(value)
                driver=Driver()
                with patch.dict(sys.modules,{'vehicle':types.SimpleNamespace(Driver=lambda:driver),'controller':types.SimpleNamespace(Emitter=object)}):
                    self.assertEqual(car.main(),0)
                self.assertEqual(driver.commands,[0.]);self.assertEqual(driver.brakes,[1.])

if __name__=='__main__':unittest.main()
