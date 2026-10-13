import math
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock
import msgpack
import zmq
from gear_sonic.utils.teleop.dex1_trigger import Limits, RampController, SIDES, TriggerFrameClock
from gear_sonic.scripts.run_dex1_trigger_control import DdsGrippers, PacketReader

class Rig:
    def __init__(self, reversed_right=False):
        self.t=10.;self.q=dict.fromkeys(SIDES,0.5)
        self.c=RampController({'left':Limits(0,1),'right':Limits(1,0) if reversed_right else Limits(0,1)})
    def tick(self,left=0,right=0,delta=.02,follow=True,**overrides):
        self.t+=delta
        tr={'time':self.t,'sample_time':self.t,'valid':True,'stop':False,'left':left,'right':right}
        hb={'time':self.t,'state':'WAIT_FOR_CONTROL','stop':False}
        measured={s:(q,self.t) for s,q in self.q.items()}
        out=self.c.update(self.t,overrides.get('measured',measured),overrides.get('trigger',tr),
                          overrides.get('heartbeat',hb),overrides.get('resume',False))
        if follow:self.q.update(out)
        return out
    def ready(self):
        for _ in range(120):self.tick()
        assert self.c.phase=='READY'

class StateTests(unittest.TestCase):
    def test_start_requires_all_feedback_and_no_zero_fallback(self):
        r=Rig();self.assertEqual(r.tick(measured={}),{});self.assertEqual(r.c.phase,'WAIT_INPUTS')
        self.assertEqual(r.tick()['left'],.5)
    def test_open_ignores_pressed_trigger_and_requires_release(self):
        r=Rig()
        for _ in range(120):r.tick(1,1)
        self.assertEqual(r.c.phase,'RELEASE_TO_ARM');self.assertEqual(r.q['left'],1)
        r.tick(0,1);self.assertEqual(r.c.phase,'RELEASE_TO_ARM')
        r.tick();self.assertEqual(r.c.phase,'READY')
    def test_obstruction_does_not_arm(self):
        r=Rig()
        for _ in range(150):r.tick(follow=False)
        self.assertEqual(r.c.phase,'OPENING')
    def test_normal_tracking_has_no_slew_delay(self):
        r=Rig();r.ready()
        for value in [1.,.37,.81,.023,0.]:
            out=r.tick(value,0)
            self.assertAlmostEqual(out['left'],1-value)
            self.assertEqual(out['right'],1.)
    def test_release_reverses_without_reset(self):
        r=Rig();r.ready()
        r.tick(.8);self.assertAlmostEqual(r.q['left'],.2)
        r.tick(.3);self.assertAlmostEqual(r.q['left'],.7)
        r.tick(.6);self.assertAlmostEqual(r.q['left'],.4)
    def test_half_trigger_holds_half_position_indefinitely(self):
        r=Rig();r.ready()
        for _ in range(300):r.tick(.5,0)
        self.assertAlmostEqual(r.q['left'],.5)
        self.assertEqual(r.q['right'],1.)
        self.assertEqual(r.c.motion['left'],'holding')

    def test_deeper_and_shallower_trigger_follow_absolute_positions(self):
        r=Rig();r.ready()
        for trigger in [.25,.75,.5,1.,0.]:
            for _ in range(110):r.tick(trigger,1-trigger)
            self.assertAlmostEqual(r.q['left'],1-trigger)
            self.assertAlmostEqual(r.q['right'],trigger)

    def test_small_trigger_is_proportional_not_boolean(self):
        r=Rig();r.ready()
        for _ in range(100):r.tick(.05,.10)
        self.assertAlmostEqual(r.q['left'],.95)
        self.assertAlmostEqual(r.q['right'],.9)

    def test_reversed_endpoint_sign(self):
        r=Rig(True);r.ready();self.assertEqual(r.q['right'],0)
        r.tick(0,1);self.assertAlmostEqual(r.q['right'],1.)
    def test_startup_lag_does_not_catch_up(self):
        r=Rig();r.tick();r.tick(delta=2.)
        self.assertAlmostEqual(r.q['left'],.52)
    def test_only_startup_full_travel_takes_two_seconds(self):
        r=Rig();r.q=dict.fromkeys(SIDES,0.);r.tick()
        for _ in range(50):r.tick()
        self.assertAlmostEqual(r.q['left'],.5)
        for _ in range(50):r.tick()
        self.assertAlmostEqual(r.q['left'],1.)
    def test_stop_holds_measured_not_previous_target(self):
        r=Rig();r.ready();r.q['left']=.7
        hb={'time':r.t+.02,'state':'STOP','stop':True}
        r.tick(1,heartbeat=hb)
        self.assertEqual(r.c.phase,'FAULT_HOLD');self.assertEqual(r.q['left'],.7)
        for _ in range(50):r.tick(0)
        self.assertEqual(r.q['left'],.7)
    def test_resume_requires_release_and_healthy_status(self):
        r=Rig();r.ready();r.tick(heartbeat=None)
        r.tick(1,resume=True);self.assertEqual(r.c.phase,'FAULT_HOLD')
        r.tick(resume=True,heartbeat=None);self.assertEqual(r.c.phase,'FAULT_HOLD')
        a=r.q.copy();r.tick(resume=True);self.assertEqual(r.c.phase,'READY');self.assertEqual(r.q,a)
    def test_stale_headset_even_with_fresh_packets(self):
        r=Rig();r.ready()
        p={'time':r.t+.02,'sample_time':r.t-1,'valid':True,'stop':False,'left':1,'right':1}
        r.tick(trigger=p);self.assertEqual(r.c.phase,'FAULT_HOLD')
    def test_stale_controller_and_trigger_stop(self):
        for packet in [None,{'time':0,'state':'CONTROL','stop':False}]:
            r=Rig();r.ready();r.tick(heartbeat=packet);self.assertEqual(r.c.phase,'FAULT_HOLD')
        r=Rig();r.ready()
        p={'time':r.t+.02,'sample_time':r.t+.02,'valid':True,'stop':True,'left':0,'right':0}
        r.tick(trigger=p);self.assertEqual(r.c.phase,'FAULT_HOLD')
    def test_missing_side_stops_publishing_that_side(self):
        r=Rig();r.ready();out=r.tick(measured={'right':(.6,r.t+.02)})
        self.assertNotIn('left',out);self.assertEqual(out['right'],.6)
        r.q['left']=.4;out=r.tick();self.assertEqual(out['left'],.4)
        self.assertEqual(r.c.phase,'FAULT_HOLD')
    def test_nan_trigger_and_bad_feedback(self):
        r=Rig();r.ready()
        p={'time':r.t+.02,'sample_time':r.t+.02,'valid':True,'stop':False,'left':math.nan,'right':0}
        r.tick(trigger=p);self.assertEqual(r.c.phase,'FAULT_HOLD')
        r.q['left']=math.nan;self.assertNotIn('left',r.tick())
    def test_first_fault_keeps_exact_side_position_and_age(self):
        r=Rig();r.ready()
        r.tick(measured={'left':(-.03,r.t+.02),'right':(.9,r.t+.02)})
        self.assertEqual(r.c.phase,'FAULT_HOLD')
        self.assertIn('left: outside_calibrated_range',r.c.reason)
        first=r.c.first_fault
        self.assertEqual(first['feedback']['left']['q'],-.03)
        r.tick(heartbeat=None)
        self.assertEqual(r.c.first_fault,first)
        self.assertIn('left: outside_calibrated_range',r.c.reason)
        r.tick(resume=True);self.assertIsNone(r.c.first_fault)

    def test_stale_feedback_distinguished_from_range_error(self):
        r=Rig();r.ready()
        r.tick(measured={'left':(.5,r.t-1.),'right':(.5,r.t+.02)})
        self.assertIn('left: stale_or_invalid_time',r.c.reason)
        self.assertGreater(r.c.first_fault['feedback']['left']['age_ms'],1000)

    def test_both_endpoints_allow_small_feedback_error_and_reverse(self):
        r=Rig();r.ready()
        r.q.update(left=1.002,right=1.002)
        r.tick(1,1)
        self.assertEqual(r.c.phase,'READY')
        self.assertLess(r.q['left'],1)
        for _ in range(110):r.tick(1,1)
        r.q.update(left=-.002,right=-.002)
        out=r.tick(0,0)
        self.assertEqual(r.c.phase,'READY')
        self.assertGreater(out['left'],0)
        self.assertGreater(out['right'],0)
        self.assertTrue(all(0<=q<=1 for q in out.values()))

    def test_start_and_hold_never_command_outside_limits(self):
        r=Rig();r.q.update(left=-.002,right=1.002)
        out=r.tick();self.assertEqual(out,{'left':0,'right':1})
        out=r.tick(heartbeat=None)
        self.assertTrue(all(0<=q<=1 for q in out.values()))

    def test_large_endpoint_violation_still_latches(self):
        r=Rig();r.ready();r.q['left']=1.03
        out=r.tick()
        self.assertEqual(r.c.phase,'FAULT_HOLD');self.assertNotIn('left',out)

    def test_invalid_endpoints_and_periods(self):
        for v in [math.nan,math.inf,0]:
            with self.assertRaises(ValueError):Limits(0,v)
        for t in [0,-1,math.inf]:
            with self.assertRaises(ValueError):RampController(dict.fromkeys(SIDES,Limits(0,1)),t)

class TriggerClockTests(unittest.TestCase):
    def test_cached_frame_not_fresh_at_start(self):
        c=TriggerFrameClock()
        self.assertEqual(c.observe(100,10.),(0.,False))
        self.assertEqual(c.observe(100,10.2),(0.,False))
    def test_advancing_input_is_fresh_without_body_updates(self):
        c=TriggerFrameClock();c.observe(100,10.)
        for i in range(1,101):
            stamp,valid=c.observe(100+i,10+i*.02)
            self.assertTrue(valid);self.assertAlmostEqual(stamp,10+i*.02)
    def test_repeated_read_does_not_refresh_stale_frame(self):
        c=TriggerFrameClock();c.observe(100,10.);c.observe(101,10.02)
        self.assertTrue(c.observe(101,10.2)[1])
        self.assertFalse(c.observe(101,10.28)[1])
        self.assertAlmostEqual(c.observe(101,11.)[0],10.02)
    def test_reset_or_invalid_source_requires_fresh_advance(self):
        c=TriggerFrameClock();c.observe(100,10.);c.observe(101,10.02)
        self.assertFalse(c.observe(50,10.04)[1])
        self.assertTrue(c.observe(51,10.06)[1])
        self.assertFalse(c.observe(0,10.08)[1])
        self.assertFalse(c.observe(math.nan,10.1)[1])
    def test_real_sdk_stall_still_latches_gripper(self):
        r=Rig();r.ready();c=TriggerFrameClock()
        c.observe(100,r.t);stamp,valid=c.observe(101,r.t+.01)
        for _ in range(20):
            now=r.t+.02;stamp,valid=c.observe(101,now)
            r.tick(trigger={'time':now,'sample_time':stamp,'valid':valid,'stop':False,'left':0.,'right':0.})
        self.assertEqual(r.c.phase,'FAULT_HOLD')

class TransportTests(unittest.TestCase):
    def sdk(self):
        init=Mock();sub=Mock();pub=Mock()
        return (init,sub,pub,lambda **x:NS(**x),object,lambda **x:NS(**x))
    def test_dry_run_never_creates_publishers(self):
        sdk=self.sdk();d=DdsGrippers('lo',sdk=sdk);d.write({'left':.5})
        sdk[2].assert_not_called();self.assertEqual(sdk[1].call_count,2);d.close()
    def test_real_message_shape_and_separate_sides_using_mock(self):
        sdk=self.sdk();d=DdsGrippers('lo',execute=True,sdk=sdk)
        d.write({'left':.2,'right':.8})
        calls=sdk[2].return_value.Write.call_args_list
        self.assertEqual([x.args[0].cmds[0].q for x in calls],[.2,.8])
        self.assertEqual(calls[0].args[0].cmds[0].tau,0.)
        self.assertEqual(len(calls[0].args[0].cmds),1);d.close()
    def test_invalid_feedback_not_replaced_with_zero(self):
        d=DdsGrippers('lo',sdk=self.sdk());d.on_state('left',NS(states=[]))
        self.assertTrue(math.isnan(d.snapshot()['left'][0]));d.close()
    def test_stop_edge_not_hidden_by_later_healthy_packet(self):
        context=zmq.Context();tx=context.socket(zmq.PAIR);rx=context.socket(zmq.PAIR)
        tx.bind('inproc://stop-edge');rx.connect('inproc://stop-edge')
        reader=PacketReader(rx,b'dex1_control_status ')
        try:
            for seq,stop in [(1,True),(2,False)]:
                tx.send(b'dex1_control_status '+msgpack.packb({'seq':seq,'time':10.+seq*.01,
                    'state':'STOP' if stop else 'CONTROL','stop':stop}))
            self.assertTrue(rx.poll(1000))
            self.assertTrue(reader.poll(10.03)['stop'])
            self.assertFalse(reader.poll(10.04)['stop'])
        finally:tx.close();rx.close();context.term()

    def test_stale_backlog_is_drained_without_losing_fault_edge(self):
        ctx=zmq.Context();tx=ctx.socket(zmq.PAIR);rx=ctx.socket(zmq.PAIR)
        tx.bind('inproc://backlog');rx.connect('inproc://backlog')
        reader=PacketReader(rx,b'dex1_control_status ')
        try:
            for seq in range(20):
                tx.send(b'dex1_control_status '+msgpack.packb({'seq':seq,'time':9.+seq*.01,
                    'state':'CONTROL','stop':False}))
            tx.send(b'dex1_control_status '+msgpack.packb({'seq':20,'time':9.99,'state':'CONTROL','stop':False}))
            self.assertTrue(rx.poll(1000))
            self.assertTrue(reader.poll(10.)['stop'])
            self.assertEqual(reader.poll(10.01)['seq'],20)
        finally:tx.close();rx.close();ctx.term()

    def test_real_zmq_packet_validation_and_stale_queue(self):
        ctx=zmq.Context();tx=ctx.socket(zmq.PAIR);rx=ctx.socket(zmq.PAIR)
        tx.bind('inproc://dex1-test');rx.connect('inproc://dex1-test')
        reader=PacketReader(rx,b'dex1_trigger ',True)
        packet={'seq':1,'time':10.,'sample_time':10.,'left':.2,'right':.8,'valid':True,'stop':False}
        def send(p):
            tx.send(b'dex1_trigger '+msgpack.packb(p));self.assertTrue(rx.poll(1000))
        try:
            send(packet);self.assertEqual(reader.poll(10.01)['left'],.2)
            send(packet);self.assertEqual(reader.poll(10.02)['time'],10.)
            send(packet);self.assertTrue(reader.poll(10.5)['stop'])
            send({'seq':2,'time':10.6,'valid':True});self.assertFalse(reader.poll(10.61)['valid'])
        finally:tx.close();rx.close();ctx.term()

if __name__=='__main__':unittest.main()
