"""Exercise the actual controller loop with invalid live speed/sensor inputs."""
import importlib.util
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

spec=importlib.util.spec_from_file_location('fault_loop',Path(__file__).resolve().parents[1]/'controllers/autonomous_car/autonomous_car.py')
car=importlib.util.module_from_spec(spec);spec.loader.exec_module(car)

def run_frames(frames):
    """Run the real main loop with (speed_kmh, distance_m) observations."""
    class Sensor:
        def enable(self, _): pass
        def getValue(self): return frames[driver.steps - 1][1]
    class Emitter:
        def send(self, value): driver.packets.append(json.loads(value))
    class Driver:
        def __init__(self):
            self.steps = 0
            self.commands, self.brakes, self.packets = [], [], []
        def getDevice(self, name): return Sensor() if 'distance' in name else Emitter()
        def setSteeringAngle(self, _): pass
        def getCurrentSpeed(self): return frames[self.steps - 1][0]
        def getTime(self): return self.steps * .032
        def step(self):
            self.steps += 1
            return 0 if self.steps <= len(frames) else -1
        def setCruisingSpeed(self, value): self.commands.append(value)
        def setBrakeIntensity(self, value): self.brakes.append(value)
    driver = Driver()
    with patch.dict(sys.modules, {'vehicle': types.SimpleNamespace(Driver=lambda: driver),
                                 'controller': types.SimpleNamespace(Emitter=object)}):
        car.main()
    return driver

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

    def test_startup_unavailable_speed_holds_then_requires_three_valid_samples(self):
        driver = run_frames([(float('nan'), 20.)] * 2 + [(0., 20.)] * 4)
        self.assertEqual(driver.commands[:4], [0.] * 4)
        self.assertEqual(driver.brakes[:4], [1.] * 4)
        self.assertGreater(driver.commands[4], 0.)
        self.assertTrue(driver.packets[4]['initialization_complete'])
        self.assertFalse(driver.packets[0]['speed_valid'])

    def test_startup_valid_streak_resets_on_invalid_observation(self):
        driver = run_frames([(0., 20.)] * 2 + [(0., float('nan'))] + [(0., 20.)] * 3)
        self.assertEqual(driver.commands[:5], [0.] * 5)
        self.assertGreater(driver.commands[5], 0.)

    def test_initialization_timeout_cannot_recover_into_acceleration(self):
        driver = run_frames([(float('nan'), 20.)] * 32 + [(0., 20.)] * 5)
        self.assertEqual(driver.commands, [0.] * 37)
        self.assertEqual(driver.brakes, [1.] * 37)
        self.assertEqual(driver.packets[-1]['state'], car.STATE_SENSOR_FAULT)

    def test_fault_after_startup_latches_full_brake_even_after_readings_recover(self):
        for invalid in [(float('nan'), 20.), (0., float('nan')), (float('inf'), 20.)]:
            with self.subTest(invalid=invalid):
                driver = run_frames([(0., 20.)] * 4 + [invalid] + [(0., 20.)] * 4)
                self.assertGreater(driver.commands[3], 0.)
                self.assertEqual(driver.commands[4:], [0.] * 5)
                self.assertEqual(driver.brakes[4:], [1.] * 5)
                self.assertTrue(all(p['state'] == car.STATE_SENSOR_FAULT for p in driver.packets[4:]))

    def test_supervisor_startup_exemption_is_bounded_and_requires_braking(self):
        source = Path(__file__).resolve().parents[1] / 'controllers/experiment_supervisor/experiment_supervisor.py'
        module_spec = importlib.util.spec_from_file_location('startup_supervisor', source)
        supervisor = importlib.util.module_from_spec(module_spec)
        module_spec.loader.exec_module(supervisor)
        packet = {'state': 'INITIALIZING', 'initialization_complete': False,
                  'commanded_speed': 0., 'brake_command': 1.}
        self.assertTrue(supervisor.is_bounded_startup_hold(packet, .096, 5.001, 5.))
        self.assertFalse(supervisor.is_bounded_startup_hold(packet, 1.1, 5., 5.))
        self.assertFalse(supervisor.is_bounded_startup_hold(packet, .1, 5.1, 5.))
        for field, value in [('brake_command', 0.), ('commanded_speed', .1),
                             ('initialization_complete', True), ('state', 'SENSOR_FAULT')]:
            self.assertFalse(supervisor.is_bounded_startup_hold({**packet, field: value}, .1, 5., 5.))

if __name__=='__main__':unittest.main()
