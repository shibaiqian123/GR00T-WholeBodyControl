import unittest
from gear_sonic.utils.inference.remote_policy_client import bind_reply

class TestRemoteClock(unittest.TestCase):
    def setUp(self):
        self.req=dict(request_id=7,generation=3,observed_at=100.)
    def test_foreign_clock_never_controls_action_age(self):
        for stamp in (-100000.,9000000.):
            r=bind_reply(self.req,dict(ok=True,request_id=7,generation=3,observed_at=stamp),100.1,100.4)
            self.assertTrue(r['ok']);self.assertEqual(r['observed_at'],100.)
            self.assertAlmostEqual(r['roundtrip_seconds'],.3)
    def test_expired_and_future_rejected(self):
        for now in (101.,99.):
            r=bind_reply(self.req,dict(ok=True,request_id=7,generation=3),100.1,now)
            self.assertFalse(r['ok'])
    def test_mismatch_rejected(self):
        for r in (None,{},dict(request_id=8,generation=3),dict(request_id=7,generation=2)):
            with self.assertRaises(ValueError):bind_reply(self.req,r,100.,100.3)

if __name__=='__main__':unittest.main()
