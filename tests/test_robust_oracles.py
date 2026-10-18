"""Hand-computed risk/kinematics oracles and adversarial result-audit checks."""
import copy,importlib.util,sys,unittest
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(name,path):
    s=importlib.util.spec_from_file_location(name,ROOT/path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
car=load('robust_car','controllers/robust_car/robust_car.py')
supervisor=load('robust_supervisor','controllers/robust_supervisor/robust_supervisor.py')
audit=load('audit_robust','scripts/audit_robust_matrix.py')

class RobustChecks(unittest.TestCase):
    def test_stopping_envelope_and_fault_latch(self):
        # At 10 m/s: 2 + 10*.35 + 100/6 = 22 1/6 metres.
        self.assertEqual(car.guarded_state(True,0.,22.,10.,None,120.),'BRAKING_EMERGENCY')
        self.assertEqual(car.guarded_state(True,0.,23.,10.,None,120.),'CRUISING')
        self.assertEqual(car.guarded_state(True,.192,120.,15.,None,120.),'SENSOR_FAULT')
        self.assertEqual(car.guarded_state(True,0.,120.,15.,'SENSOR_FAULT',120.),'SENSOR_FAULT')
        self.assertEqual(car.guarded_state(True,0.,120.,15.,None,120.),'CRUISING')
        self.assertEqual(car.guarded_state(True,0.,119.,15.,'BRAKING_EMERGENCY',120.),'BRAKING_EMERGENCY')

    def test_delay_uses_measurement_time(self):
        self.assertEqual(car.delayed_sample([(0.,120.),(.1,80.),(.2,40.)],.2,.1),(.1,80.))

    def test_lead_stopping_distance(self):
        self.assertAlmostEqual(supervisor.lead_displacement(.5,12.,4.,1.),6.)
        self.assertAlmostEqual(supervisor.lead_displacement(2.,12.,4.,1.),22.)
        self.assertAlmostEqual(supervisor.lead_displacement(10.,12.,4.,1.),30.)

    def test_hull_is_not_vehicle_reference(self):
        forward=supervisor.forward_extent([1,0,0,0,1,0,0,0,1])
        self.assertTrue(3.9<forward<3.91)

    def test_audit_rejects_hidden_contact_forged_margin_and_missing_row(self):
        orientation=[1.,0.,0.,0.,1.,0.,0.,0.,1.]
        extent=supervisor.forward_extent(orientation)
        row={'step':1.,'sim_time_s':.032,'active':True,'car_x_m':5.,'obstacle_x_m':10000.,'true_speed_mps':10.,
             'reference_distance_m':9994.,'hull_forward_extent_m':extent,'hull_clearance_m':9994.-extent,'contact':False,
             'brake_command':1.,**{f'orientation_{i}':v for i,v in enumerate(orientation)}}
        second={**row,'step':2.,'sim_time_s':.064,'car_x_m':5.2,'obstacle_x_m':36.,'true_speed_mps':0.,'reference_distance_m':29.8,'hull_clearance_m':29.8-extent}
        rows=[row,second];summary={'rows':2,'config':{'target_speed_mps':10.,'hazard_reference_gap_m':30.},'trigger_time_s':.032,'end_reason':'stopped',**supervisor.outcome(rows,.032,10.)}
        self.assertEqual(audit.verify_case(rows,summary),2)
        for key,value in [('contact',True),('hull_clearance_m',100.)]:
            fake=copy.deepcopy(rows);fake[0][key]=value
            with self.assertRaises(ValueError):audit.verify_case(fake,summary)
        with self.assertRaises(ValueError):audit.verify_case([],summary)

if __name__=='__main__':unittest.main()
