import json
import unittest
import numpy as np
from gear_sonic.utils.inference.dit4dit_control import (
    Arbiter, sonic_state, normalize_state, target_to_trigger, command_fields)
from gear_sonic.utils.teleop.dex1_trigger import Limits
from gear_sonic.utils.teleop.zmq.zmq_planner_sender import build_command_message

class ControlTests(unittest.TestCase):
    def ready(self):
        a=Arbiter();a.status(dict(time=10.,state='CONTROL',stop=False),10.)
        self.assertTrue(a.accept(dict(generation=0,observed_at=9.9,actions=np.zeros((50,66))),10.))
        return a
    def model(self):
        a=self.ready();a.select('model',10.,True,True,True);return a
    def test_state_order_matches_conversion(self):
        q=np.arange(43)
        self.assertEqual(sonic_state(q,[100,101]).tolist(),list(range(22))+list(range(29,36))+[100,101])
    def test_constant_normalization(self):
        s=normalize_state(np.ones(31),dict(min=[0]*30+[1],max=[2]*30+[1]))
        np.testing.assert_array_equal(s,np.zeros((1,31)))
    def test_shape_and_nan_rejected(self):
        for v in [np.zeros((50,78)),np.full((50,66),np.nan)]:
            with self.assertRaises(ValueError):
                self.ready().accept(dict(generation=0,observed_at=9.9,actions=v),10.)
    def test_default_is_readonly(self):
        a=self.ready();self.assertEqual(a.mode,'readonly');self.assertIsNone(a.frame(10.,True))
    def test_bottom_o_latches_stop(self):
        a=self.model();a.status(dict(time=10.01,state='STOP',stop=True),10.01)
        a.status(dict(time=10.02,state='CONTROL',stop=False),10.02)
        self.assertIsNone(a.frame(10.02,True));self.assertEqual(a.mode,'stop')
        self.assertFalse(a.accept(dict(generation=0,observed_at=10.02,actions=np.zeros((50,66))),10.02))
        with self.assertRaises(ValueError):a.select('model',10.02,True,True,True)
    def test_expired_chunk_is_not_repeated(self):
        a=self.model();a.status(dict(time=11.,state='CONTROL',stop=False),11.)
        self.assertIsNone(a.frame(11.,True));self.assertEqual(a.reason,'action_expired')
    def test_sensor_loss_stops(self):
        a=self.model();self.assertIsNone(a.frame(10.,False));self.assertEqual(a.mode,'stop')
    def test_takeover_discards_inflight(self):
        a=self.model();a.select('teleop',10.,pico_ok=True)
        self.assertIsNone(a.chunk)
        self.assertFalse(a.accept(dict(generation=0,observed_at=9.9,actions=np.ones((50,66))),10.))
    def test_missing_pico_takeover_stops(self):
        a=self.model()
        with self.assertRaises(ValueError):a.select('teleop',10.,pico_ok=False)
        self.assertEqual(a.mode,'stop')
    def test_reset_cannot_bypass_active_control(self):
        a=self.model()
        with self.assertRaises(ValueError):a.reset(10.)
        self.assertEqual(a.mode,'model')
    def test_reset_requires_live_bottom(self):
        a=self.model();a.stop('x')
        with self.assertRaises(ValueError):a.reset(12.)
        a.status(dict(time=12.,state='WAIT_FOR_CONTROL',stop=False),12.)
        a.reset(12.);self.assertEqual(a.mode,'readonly');self.assertIsNone(a.chunk)
    def test_action_index_from_observation(self):
        a=self.ready();arr=np.repeat(np.arange(50)[:,None],66,axis=1)
        a.accept(dict(generation=0,observed_at=9.9,actions=arr),10.)
        a.select('model',10.,True,True,True)
        self.assertEqual(a.frame(10.01,True)[0],5)
    def test_dex_absolute_mapping(self):
        limits=dict(left=Limits(0,5,.1),right=Limits(0,6,.1))
        p=dict(time=10.,sample_time=10.,valid=True,stop=False,left=2.45,right=5.9)
        out=target_to_trigger(p,limits,10.)
        self.assertAlmostEqual(out['left'],.5);self.assertAlmostEqual(out['right'],0.)
        for side in limits:
            l=limits[side]
            self.assertAlmostEqual(l.command_open+out[side]*(l.closed-l.command_open),p[side])
        with self.assertRaises(ValueError):target_to_trigger(dict(p,left=99),limits,10.)
        with self.assertRaises(ValueError):target_to_trigger(p,limits,11.)
    def test_command_decode(self):
        self.assertEqual(command_fields(build_command_message(True,False,True)),dict(start=True,stop=False,planner=True))
        self.assertIsNone(command_fields(b'planner'))
        with self.assertRaises(Exception):command_fields(b'commandbad')

if __name__=='__main__':unittest.main()
