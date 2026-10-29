import unittest
import numpy as np
from gear_sonic.utils.inference.synchronous_policy import SynchronousPolicy

class Tests(unittest.TestCase):
 def begin(self,n=10,delay=.3):
  s=SynchronousPolicy(n);r=s.request(10.,10.)
  a=np.arange(50*66).reshape(50,66)
  ok=s.accept(dict(r,ok=True,actions=a),10.+delay)
  return s,r,ok
 def test_serial_and_first_frame(self):
  s,r,ok=self.begin();self.assertTrue(ok)
  for i in range(10):
   t=10.3+i*.02;p=s.action(t);self.assertEqual(p['window_index'],i)
   self.assertIsNone(s.request(t,t));self.assertFalse(s.accept(dict(r,ok=True,actions=np.zeros((50,66))),t))
  self.assertIsNone(s.action(10.5));self.assertEqual(s.phase,'observe')
  self.assertIsNone(s.request(10.49,10.5));self.assertIsNotNone(s.request(10.5,10.5))
 def test_wait_does_not_emit(self):
  s=SynchronousPolicy(10);s.request(10.,10.)
  self.assertIsNone(s.action(10.2));self.assertIsNone(s.request(10.2,10.2))
 def test_stop_discards_late_result(self):
  s=SynchronousPolicy(10);r=s.request(10.,10.);s.stop('manual')
  self.assertFalse(s.accept(dict(r,ok=True,actions=np.zeros((50,66))),10.1))
  self.assertIsNone(s.request(10.2,10.2));self.assertIsNone(s.action(10.2))
 def test_horizon_not_extended(self):
  s,_,ok=self.begin(50);self.assertFalse(ok);self.assertEqual(s.reason,'segment_exceeds_action_horizon')
 def test_missed_frame_no_catchup(self):
  s,_,_=self.begin();s.action(10.3);self.assertIsNone(s.action(10.36));self.assertEqual(s.reason,'missed_execution_deadline')
 def test_invalid_feedback(self):
  s,_,_=self.begin();self.assertIsNone(s.action(10.3,False));self.assertEqual(s.phase,'stop')
 def test_timeout_and_clock(self):
  s=SynchronousPolicy(10);s.request(10.,10.);s.action(12.01);self.assertEqual(s.reason,'inference_timeout')
  s,_,_=self.begin();s.action(9.);self.assertEqual(s.reason,'invalid_clock')
 def test_nonfinite_prediction(self):
  s=SynchronousPolicy(10);r=s.request(10.,10.)
  self.assertFalse(s.accept(dict(r,ok=True,actions=np.full((50,66),np.nan)),10.1))
 def test_duplicate_tick_no_extra_frame(self):
  s,_,_=self.begin();self.assertIsNotNone(s.action(10.3));self.assertIsNone(s.action(10.3))
if __name__=='__main__':unittest.main()
