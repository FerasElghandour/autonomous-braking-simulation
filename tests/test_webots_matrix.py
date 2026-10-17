import importlib.util,json,re,tempfile
from pathlib import Path
import unittest

spec=importlib.util.spec_from_file_location('matrix',Path(__file__).resolve().parents[1]/'scripts/webots_matrix.py')
matrix=importlib.util.module_from_spec(spec);spec.loader.exec_module(matrix)

class MatrixChecks(unittest.TestCase):
    def test_generated_world_geometry_matches_each_scenario(self):
        with tempfile.TemporaryDirectory() as temporary:
            out=Path(temporary)/'matrix';report=matrix.prepare(out)
            self.assertEqual(len(report['cases']),9)
            for job in report['cases']:
                project=out/job['project'];config=json.loads((project/'config/scenario.json').read_text())
                world=(project/'worlds/obstacle_braking.wbt').read_text()
                obstacle=float(re.search(r'DEF OBSTACLE Solid \{\s+translation ([\d.]+)',world).group(1))
                self.assertEqual(obstacle,config['obstacle_position_m'])
                self.assertEqual(obstacle-1-config['car_start_position_m'],job['initial_reference_clearance_m'])
                self.assertEqual(config['target_speed_mps'],job['speed_mps'])
                self.assertTrue((project/'controllers/autonomous_car/autonomous_car.py').is_file())

if __name__=='__main__':unittest.main()
