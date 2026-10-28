import unittest
from gear_sonic.scripts.run_dit4dit_model import poll_bottom_status
from gear_sonic.utils.inference.model_only_control import ModelControl

class ClockOrderTest(unittest.TestCase):
    def evaluate(self,stamp,stop=False):
        c=ModelControl();c.mode='armed'
        class Reader:
            last_error=''
            def poll(self):
                return dict(time=stamp,seq=1,state='STOP' if stop else 'WAIT_FOR_CONTROL',stop=stop)
        times=iter([100.,100.002])
        diag=poll_bottom_status(c,Reader(),lambda:next(times))
        return c,diag
    def test_packet_arriving_during_poll_is_fresh(self):
        c,d=self.evaluate(100.001)
        self.assertEqual(c.mode,'armed');self.assertGreater(d['age_seconds'],0)
    def test_previous_clock_reproduces_false_stop(self):
        c=ModelControl();c.mode='armed'
        c.status(dict(time=100.001,state='WAIT_FOR_CONTROL',stop=False),100.)
        self.assertEqual(c.reason,'bottom_stale')
    def test_actual_stale_still_stops(self):
        c,_=self.evaluate(99.7);self.assertEqual(c.reason,'bottom_stale')
    def test_actual_future_still_stops(self):
        c,_=self.evaluate(101.);self.assertEqual(c.reason,'bottom_stale')
    def test_stop_arriving_during_poll_latches(self):
        c,_=self.evaluate(100.001,True)
        self.assertEqual(c.reason,'bottom_stop');self.assertTrue(c.stop_ack)
        for command in ('model','arm'):
            with self.assertRaises(ValueError):c.command(command,100.002,True,True)
if __name__=='__main__':unittest.main()
