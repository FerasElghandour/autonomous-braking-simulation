import importlib.util
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('stress',Path(__file__).resolve().parents[1]/'scripts/stress_validation.py')
stress=importlib.util.module_from_spec(spec);spec.loader.exec_module(stress)

class StressChecks(unittest.TestCase):
    def test_nominal_far_obstacle_stops_without_collision(self):
        r=stress.simulate(10,80,0,'nominal',100)
        self.assertTrue(r['safe_stop']);self.assertLess(r['final_speed_mps'],.05)

    def test_impossible_stopping_distance_is_failure(self):
        r=stress.simulate(15,1,0,'nominal',100)
        self.assertTrue(r['collision']);self.assertFalse(r['safe_stop'])

    def test_noise_is_seeded_and_changes_observation_path(self):
        a=stress.simulate(10,80,0,'noise',100);b=stress.simulate(10,80,0,'noise',100)
        self.assertEqual(a,b)
        c=stress.simulate(10,80,0,'noise',101)
        self.assertNotEqual(a['minimum_clearance_m'],c['minimum_clearance_m'])

if __name__=='__main__':unittest.main()
