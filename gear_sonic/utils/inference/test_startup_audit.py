import unittest
import numpy as np
from gear_sonic.scripts.audit_model_startup import match_token
from gear_sonic.scripts.replay_sonic_startup import validated,map_action
class Tests(unittest.TestCase):
 def test_ambiguous_not_forced(self):
  p=dict(time=10,ok=True,request_id=4,generation=2,observed_at=9.8,actions=np.zeros((50,66)))
  self.assertEqual(len(match_token(np.zeros(64),[p],10.1)['candidates']),50)
  self.assertEqual(match_token(np.zeros(64),[p],10.1)['status'],'ambiguous')
  self.assertEqual(match_token(np.zeros(64),[p],9.)['status'],'unmatched')
 def test_unique_index_is_not_send_counter(self):
  a=np.arange(50*66).reshape(50,66)
  p=dict(time=10,ok=True,request_id=4,generation=2,observed_at=9.8,actions=a)
  r=match_token(a[23,:64],[p],10.3)
  self.assertEqual(r['status'],'unique');self.assertEqual(r['candidates'][0]['window_index'],23)
 def test_missing_full_input_rejected(self):
  with self.assertRaisesRegex(ValueError,'decoder_input'):validated(dict(token=[0]*64))
 def test_mapping_nonuniform(self):
  row=dict(mapping=list(reversed(range(29))),scale=np.arange(29)*.01+.1,default_angles=np.arange(29)*.02-.4)
  action=np.arange(29)*.03
  expected=np.array([row['default_angles'][i]+action[28-i]*row['scale'][i] for i in range(29)])
  np.testing.assert_allclose(map_action(action,row),expected)
if __name__=='__main__':unittest.main()
