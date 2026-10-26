import unittest
from gear_sonic.utils.teleop.dex1_trigger import Limits, ObstructionHold, RampController, SIDES

class GuardTests(unittest.TestCase):
    def run_guard(self, g, count=51, reverse=False, q=.6, dq=0., start=10.):
        lim=Limits(1,0) if reverse else Limits(0,1)
        for i in range(count):
            t=start+i*.02
            out=g.update(lim,t,(q,t,dq),1.,.95 if reverse else .05)
        return out
    def test_one_second_and_capture(self):
        g=ObstructionHold()
        self.run_guard(g,50)
        self.assertIsNone(g.hold_q)
        self.assertEqual(g.update(Limits(0,1),11.,(.6,11.,0.),1.,.05),.6)
        self.assertAlmostEqual(g.release_trigger,.4)
    def test_release_catchup_not_first_release(self):
        g=ObstructionHold();self.run_guard(g)
        self.assertEqual(g.update(Limits(0,1),11.02,(.58,11.02,0.),.6,.4),.6)
        self.assertEqual(g.update(Limits(0,1),11.04,(.58,11.04,0.),.4,.6),.6)
        self.assertIsNone(g.hold_q)
        self.assertEqual(g.update(Limits(0,1),11.06,(.6,11.06,0.),.2,.8),.8)
    def test_reverse(self):
        g=ObstructionHold();self.assertEqual(self.run_guard(g,reverse=True),.6)
        self.assertAlmostEqual(g.release_trigger,.6)
    def test_moving_velocity(self):
        g=ObstructionHold();self.run_guard(g,200,dq=.1);self.assertIsNone(g.hold_q)
    def test_moving_position(self):
        g=ObstructionHold()
        for i in range(200):
            t=10+i*.02;g.update(Limits(0,5),t,(4-i*.01,t,0.),1.,0.)
        self.assertIsNone(g.hold_q)
    def test_small_error(self):
        g=ObstructionHold();self.run_guard(g,200,q=.1);self.assertIsNone(g.hold_q)
    def test_missing_velocity(self):
        g=ObstructionHold()
        for i in range(100):
            t=10+i*.02;g.update(Limits(0,1),t,(.6,t),1.,.05)
        self.assertIsNone(g.hold_q)
    def test_repeated_sample(self):
        g=ObstructionHold()
        for i in range(100):g.update(Limits(0,1),10+i*.02,(.6,10,0.),1.,.05)
        self.assertIsNone(g.hold_q)
    def test_gap(self):
        g=ObstructionHold();self.run_guard(g,40)
        self.run_guard(g,40,start=12.);self.assertIsNone(g.hold_q)
    def test_open_request_resets(self):
        g=ObstructionHold();self.run_guard(g,40)
        g.update(Limits(0,1),10.8,(.6,10.8,0.),0.,1.)
        self.run_guard(g,40,start=10.82);self.assertIsNone(g.hold_q)
    def test_invalid_duration(self):
        for value in (0,-1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):ObstructionHold(value)

class IntegrationTests(unittest.TestCase):
    def controller(self, enabled=True):
        c=RampController({s:Limits(0,1) for s in SIDES},obstruction_hold=enabled)
        c.phase='READY';c.target=dict.fromkeys(SIDES,.6)
        return c
    def tick(self,c,t,left=1,right=.4,stop=False,resume=False):
        return c.update(t,{s:(.6,t,0.) for s in SIDES},
            dict(time=t,sample_time=t,valid=True,stop=stop,left=left,right=right),
            dict(time=t,state='CONTROL',stop=False),resume)
    def test_independent(self):
        c=self.controller()
        for i in range(60):self.tick(c,10+i*.02)
        self.assertEqual(c.obstruction_status['left']['state'],'holding')
        self.assertEqual(c.obstruction_status['right']['state'],'idle')
        out=self.tick(c,11.2,right=.2)
        self.assertEqual(out['left'],.6);self.assertEqual(out['right'],.8)
    def test_disabled_preserves_old_behavior(self):
        c=self.controller(False)
        for i in range(100):self.tick(c,10+i*.02)
        self.assertIsNone(c.obstruction_status);self.assertLess(c.target['left'],.1)
    def test_fault_overrides_hold_and_requires_resume(self):
        c=self.controller()
        for i in range(60):self.tick(c,10+i*.02)
        self.tick(c,11.2,stop=True)
        self.assertEqual(c.phase,'FAULT_HOLD');self.assertIsNone(c.obstruction['left'].hold_q)
        self.tick(c,11.22,left=0,right=0);self.assertEqual(c.phase,'FAULT_HOLD')
        self.tick(c,11.24,left=0,right=0,resume=True);self.assertEqual(c.phase,'READY')
    def test_startup_not_latched(self):
        c=self.controller();c.phase='OPENING'
        for i in range(200):self.tick(c,10+i*.02)
        self.assertEqual(c.phase,'OPENING');self.assertIsNone(c.obstruction['left'].hold_q)

if __name__=='__main__':unittest.main()
