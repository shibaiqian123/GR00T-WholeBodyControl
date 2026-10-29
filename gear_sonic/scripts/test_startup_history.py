"""Offline history-transform regression tests; no DDS or control imports."""
import unittest
import numpy as np
from gear_sonic.scripts.compare_startup_history import measured_history,variants,SEGMENTS
class HistoryTest(unittest.TestCase):
 def fixture(self):
  mapping=np.roll(np.arange(29),7);q=np.arange(29)*.125;defaults=np.arange(29)*.01
  sim=[dict(kind='sim_state',time=1+i*.02,sim_time=1+i*.02,body_q=q.tolist(),body_dq=(q+1).tolist(),qpos=[0,0,1,1,0,0,0],qvel=[0,0,0,1,2,3]) for i in range(10)]
  first=dict(time=1.181,actual_q=q.tolist(),mapping=mapping.tolist(),default_angles=defaults.tolist(),decoder_input=np.arange(994).tolist())
  return first,sim
 def test_mapping_bias_and_invariants(self):
  first,sim=self.fixture();r,_=measured_history(first,sim)
  expected=(np.asarray(first['actual_q'])-first['default_angles'])[np.argsort(first['mapping'])]
  np.testing.assert_allclose(r['q'][0],expected)
  np.testing.assert_allclose(r['gravity'][0],[0,0,-1])
  for v in variants(first,r).values():
   np.testing.assert_array_equal(v[:64],first['decoder_input'][:64]);np.testing.assert_array_equal(v[674:964],first['decoder_input'][674:964])
   for start,end,width in SEGMENTS.values():np.testing.assert_array_equal(v[end-width:end],first['decoder_input'][end-width:end])
 def test_missing_history_fails(self):
  first,sim=self.fixture()
  with self.assertRaises(ValueError):measured_history(first,sim[-1:])
 def test_nonfinite_fails(self):
  first,sim=self.fixture();first['decoder_input'][0]=float('nan')
  with self.assertRaises(ValueError):measured_history(first,sim)
if __name__=='__main__':unittest.main()
