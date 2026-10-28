import unittest
import numpy as np
from gear_sonic.utils.inference.model_only_control import ModelControl,ModelGripInput
from gear_sonic.utils.teleop.dex1_trigger import Limits,RampController

class Tests(unittest.TestCase):
    def ready(self):
        c=ModelControl();c.status(dict(time=10.,state='WAIT_FOR_CONTROL',stop=False),10.)
        c.accept(dict(generation=0,observed_at=9.9,actions=np.zeros((50,66))),10.)
        return c
    def test_manual_gates(self):
        c=self.ready()
        with self.assertRaises(ValueError):c.command('model',10.,True,True)
        c.command('arm',10.,True,False);self.assertEqual(c.mode,'armed')
        with self.assertRaises(ValueError):c.command('model',10.,True,False)
        c.command('model',10.,True,True);self.assertEqual(c.mode,'starting')
    def test_stop_ack_allows_restart_but_not_motion(self):
        c=self.ready();c.stop('test');c.status(dict(time=10.,state='STOP',stop=True),10.)
        self.assertTrue(c.stop_ack)
        c.status(dict(time=10.1,state='WAIT_FOR_CONTROL',stop=False),10.1)
        self.assertEqual(c.mode,'stop');self.assertTrue(c.stop_ack)
        self.assertFalse(c.accept(dict(generation=0,observed_at=10.,actions=np.zeros((50,66))),10.1))
        with self.assertRaises(ValueError):c.command('model',10.1,True,True)
        c.command('reset',10.1);self.assertEqual(c.mode,'readonly');self.assertIsNone(c.chunk)
    def test_recover_does_not_clear_latch(self):
        c=self.ready();c.stop('test');c.command('recover',10.)
        self.assertEqual(c.mode,'stop');self.assertTrue(c.stop_ack)
        c.status(dict(time=10.,state='CONTROL',stop=False),10.)
        with self.assertRaises(ValueError):c.command('reset',10.)
    def test_expiration_nonfinite_and_loss(self):
        c=self.ready();c.command('arm',10.,True);c.command('model',10.,True,True)
        self.assertIsNone(c.action(10.,False,True));self.assertEqual(c.mode,'stop')
        c=self.ready();c.command('arm',10.,True);c.command('model',10.,True,True)
        c.status(dict(time=11.,state='CONTROL',stop=False),11.)
        self.assertIsNone(c.action(11.,True,True));self.assertEqual(c.reason,'action_expired')
        c=self.ready()
        with self.assertRaises(ValueError):c.accept(dict(generation=0,observed_at=10.,actions=np.full((50,66),np.nan)),10.)
    def test_grippers_hold_current_no_auto_open(self):
        c=RampController({s:Limits(0.,5.,.05) for s in ('left','right')},obstruction_hold=True)
        g=ModelGripInput();m={'left':(.2,10.,0.),'right':(3.,10.,0.)};h=dict(time=10.,state='WAIT_FOR_CONTROL',stop=False)
        self.assertEqual(g.update(c,None,m,h,10.),{});self.assertEqual(c.phase,'WAIT_INPUTS')
        p=dict(time=10.,sample_time=10.,valid=True,stop=False,mode='arm',arm_id='one',left=0.,right=0.)
        result=g.update(c,p,m,h,10.);np.testing.assert_allclose(list(result.values()),[.2,3.],atol=1e-12);self.assertEqual(c.phase,'READY')
        # Expiration locks hold; replaying arm token cannot reset it.
        g.update(c,p,m,h,11.);self.assertEqual(c.phase,'FAULT_HOLD')
        m={s:(q,11.1,0.) for s,q in [('left',.2),('right',3.)]};h=dict(h,time=11.1)
        g.update(c,dict(p,time=11.1,sample_time=11.1),m,h,11.1);self.assertEqual(c.phase,'FAULT_HOLD')
        g.update(c,dict(p,time=11.1,sample_time=11.1,arm_id='two'),m,h,11.1);self.assertEqual(c.phase,'READY')
    def test_grip_bad_feedback_cannot_arm(self):
        c=RampController({s:Limits(0.,5.) for s in ('left','right')});g=ModelGripInput()
        p=dict(time=10.,sample_time=10.,valid=True,stop=False,mode='arm',arm_id='x',left=0.,right=0.)
        self.assertEqual(g.update(c,p,{},dict(time=10.,state='WAIT_FOR_CONTROL',stop=False),10.),{})
        self.assertEqual(c.phase,'WAIT_INPUTS')

if __name__=='__main__':unittest.main()
