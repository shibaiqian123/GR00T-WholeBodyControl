import unittest
import numpy as np
from gear_sonic.utils.inference.sync_model_control import SyncModelControl

class Tests(unittest.TestCase):
 def heartbeat(self,c,t,state='CONTROL'):
  c.status(dict(time=t,state=state,stop=False),t)
 def armed(self):
  c=SyncModelControl(10);self.heartbeat(c,10.,'WAIT_FOR_CONTROL')
  self.assertIsNone(c.request_metadata(0,10.,10.))
  c.command('arm',10.,True,True)
  self.assertIsNone(c.request_metadata(0,10.,10.))
  c.command('model',10.,True,True);req=c.request_metadata(0,10.,10.)
  return c,req
 def start(self):
  c,r=self.armed();a=np.repeat(np.arange(50)[:,None],66,axis=1)
  self.assertTrue(c.accept(dict(r,ok=True,actions=a),10.2))
  self.heartbeat(c,10.21,'WAIT_FOR_CONTROL')
  np.testing.assert_array_equal(c.action(10.21,True,True),a[0]);self.assertEqual(c.action_info['phase'],'prefill')
  self.heartbeat(c,10.33);np.testing.assert_array_equal(c.action(10.33,True,True),a[0]);self.assertEqual(c.mode,'model')
  return c,a
 def finish_segment(self,c):
  for i in range(1,10):
   t=10.33+.02*i;self.heartbeat(c,t);c.action(t,True,True)
   self.assertEqual(c.action_info['window_index'],i)
   self.assertIsNone(c.request_metadata(1,t,t))
  self.heartbeat(c,10.53);out=c.action(10.53,True,True)
  self.assertEqual(c.scheduler.phase,'observe');self.assertEqual(c.action_info['phase'],'continue_last_token')
  return out
 def test_two_serial_cycles(self):
  c,a=self.start();out=self.finish_segment(c);np.testing.assert_array_equal(out,a[9])
  self.assertIsNone(c.request_metadata(1,10.52,10.53))
  r=c.request_metadata(1,10.53,10.53);self.assertIsNotNone(r)
  self.heartbeat(c,10.6);np.testing.assert_array_equal(c.action(10.6,True,True),a[9])
  self.assertEqual(c.action_info['observed_at'],10.)
  self.assertTrue(c.accept(dict(r,ok=True,actions=a+100),10.7))
  self.heartbeat(c,10.7);np.testing.assert_array_equal(c.action(10.7,True,True),a[0]+100)
 def test_wait_keeps_original_expiry(self):
  c,a=self.start();self.finish_segment(c);r=c.request_metadata(1,10.53,10.53)
  self.heartbeat(c,11.);self.assertIsNone(c.action(11.,True,True));self.assertEqual(c.reason,'action_expired')
  self.assertFalse(c.accept(dict(r,ok=True,actions=a),11.01))
 def test_bottom_stop_wins_and_reset_new_generation(self):
  c,a=self.start();c.status(dict(time=10.34,state='STOP',stop=True),10.34)
  self.assertIsNone(c.action(10.34,True,True))
  self.heartbeat(c,10.4,'WAIT_FOR_CONTROL');c.command('reset',10.4)
  self.assertEqual(c.mode,'readonly');self.assertIsNone(c.last_output)
  c.command('arm',10.4,True,True);c.command('model',10.4,True,True);r=c.request_metadata(3,10.4,10.4)
  self.assertEqual(r['generation'],c.generation)
 def test_sensor_loss_stops_wait(self):
  c,a=self.start();self.finish_segment(c)
  self.assertIsNone(c.action(10.54,False,True));self.assertEqual(c.mode,'stop')
 def test_stale_prediction_cannot_start(self):
  c,r=self.armed()
  self.assertFalse(c.accept(dict(r,ok=True,actions=np.zeros((50,66))),10.6))
  self.assertEqual(c.mode,'stop')
 def test_unexpected_result_cannot_replace(self):
  c,a=self.start();self.assertFalse(c.accept(dict(request_id=99,generation=c.generation,observed_at=10.3,ok=True,actions=a+200),10.34))
  self.assertEqual(c.scheduler.phase,'execute')
if __name__=='__main__':unittest.main()
