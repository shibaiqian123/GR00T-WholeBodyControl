import json,unittest
import numpy as np
from gear_sonic.utils.inference.observation_diagnostics import inspect_observation
class Limit:
    def contains(self,q):return 0<=q<=6
class Tests(unittest.TestCase):
    def setUp(self):
        self.c={'timestamps':{'ego_view':1000.},'images':{'ego_view':np.zeros((4,4,3),dtype=np.uint8)}}
        self.b={'sample_time':10.,'source_tick':7}
        self.d={'time':10.,'sides':{s:dict(feedback_time=10.,feedback_valid=True,q=2.) for s in ('left','right')}}
    def check(self):return inspect_observation(self.c,self.b,self.d,{s:Limit() for s in ('left','right')},10.01,1000.01)
    def test_valid(self):self.assertTrue(self.check()['ok'])
    def test_camera_age(self):
        self.c['timestamps']['ego_view']=999.;self.assertIn('camera_age_seconds:outside_0_250ms',self.check()['reasons'])
    def test_missing_body(self):
        self.b=None;self.assertIn('body_age_seconds:missing_or_nonfinite',self.check()['reasons'])
    def test_grip_invalid(self):
        self.d['sides']['left']['feedback_valid']=False;self.assertIn('left:feedback_invalid',self.check()['reasons'])
    def test_skew(self):
        self.b['sample_time']=9.8;self.assertIn('observation_skew:over_150ms',self.check()['reasons'])
    def test_nan_json_safe(self):
        self.d['sides']['right']['q']=float('nan');r=self.check();self.assertFalse(r['ok']);json.dumps(r,allow_nan=False)
    def test_bad_image(self):
        self.c['images']['ego_view']=np.zeros((3,4));self.assertIn('image:invalid_shape_or_dtype',self.check()['reasons'])
    def test_future_feedback(self):
        self.d['sides']['left']['feedback_time']=10.1;self.assertIn('left_feedback_age_seconds:outside_0_250ms',self.check()['reasons'])
if __name__=='__main__':unittest.main()
