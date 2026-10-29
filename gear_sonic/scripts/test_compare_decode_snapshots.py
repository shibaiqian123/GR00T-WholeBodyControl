import unittest
import numpy as np
from compare_decode_snapshots import prepare, targets
class Tests(unittest.TestCase):
 def row(self):
  return dict(schema_name='sonic_v1_1_prospective_control_v1',state='WAIT_FOR_CONTROL',decoder_input=list(range(994)),actual_q=list(range(29)),actual_dq=list(range(29)),mapping=list(range(28,-1,-1)),scale=[i+.5 for i in range(29)],default_angles=[i*.3 for i in range(29)],sample_time=1)
 def test_only_token_changes(self):
  r=self.row(); x=prepare(r,np.arange(64)*2); np.testing.assert_array_equal(x[64:],np.arange(64,994)); self.assertEqual(r['decoder_input'][1],1)
 def test_mapping_scale_bias_once(self):
  r=self.row(); raw=np.arange(29)*.7
  np.testing.assert_array_equal(targets(raw,r),np.array([raw[28-i]*(i+.5)+i*.3 for i in range(29)],dtype=np.float32))
 def test_invalid(self):
  for k,v in [('decoder_input',[0]*993),('actual_q',[float('nan')]*29),('mapping',[0]*29),('schema_name','wrong')]:
   r=self.row();r[k]=v
   with self.assertRaises(ValueError):prepare(r,np.zeros(64))
 def test_planner(self):
  r=self.row();r.update(schema_name='sonic_v1_1_planner_observation_v1',state='CONTROL_PLANNER');self.assertEqual(prepare(r,np.zeros(64)).size,994)
if __name__=='__main__':unittest.main()
