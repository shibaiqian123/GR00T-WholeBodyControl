import unittest
from gear_sonic.utils.vibe_pose_gate import sample_grid,status_allows
class GateTests(unittest.TestCase):
    def test_ready(self):
        self.assertTrue(status_allows(dict(prepare_id="a",session="s",ready=True,sent_monotonic=1),"a","s",1.1))
    def test_rejections(self):
        good=dict(prepare_id="a",session="s",ready=True,sent_monotonic=1)
        for change in (dict(prepare_id="b"),dict(session="b"),dict(ready=False),dict(sent_monotonic=.5),dict(sent_monotonic=2),dict(sent_monotonic="bad")):
            self.assertFalse(status_allows(dict(good,**change),"a","s",1.1))
        self.assertFalse(status_allows(None,"a","s",1.1))
    def test_grid(self):
        self.assertEqual(sample_grid(0,60_000_000,20_000_000),[20_000_000,40_000_000,60_000_000])
        self.assertEqual(sample_grid(60_000_000,111_000_000,80_000_000),[80_000_000,100_000_000])
        self.assertEqual(sample_grid(111_000_000,150_000_000,120_000_000),[120_000_000,140_000_000])
    def test_invalid_grid(self):
        for args in ((1,1,1),(0,300_000_000,0),(20,40,0)):
            with self.assertRaises(ValueError):sample_grid(*args)
if __name__=="__main__":unittest.main()
