"""Exercise the actual viewer key-dispatch AST without DDS or robot motion."""
import ast,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from gear_sonic.utils.vibe_sim_support import original_band
SOURCE=Path(__file__).resolve().parents[1]/"scripts/run_vibe_hybrid_sim.py"
TREE=ast.parse(SOURCE.read_text())
KEYS=next(n for n in ast.walk(TREE) if isinstance(n,ast.While) and isinstance(n.test,ast.Name) and n.test.id=="keys")
CODE=compile(ast.Module(body=[KEYS],type_ignores=[]),str(SOURCE),"exec")
class Socket:
    def close(self,*args):pass
    def setsockopt(self,*args):pass
    def connect(self,*args):pass
def run(keys,fault=None,paused=False):
    band,_=original_band()
    state=dict(keys=keys,band=band,fault_reason=fault,paused=paused,clock=NS(resume=lambda:None),
               initialize=lambda:{"mode":"standing"},epoch=2,tick=20,count=1,last_cmd=1,
               active_started=1,transport=Socket(),ctx=NS(socket=lambda *_:Socket()),
               zmq=NS(REQ=1,LINGER=2,RCVTIMEO=3,SNDTIMEO=4),endpoint="test")
    exec(CODE,state)
    return state
class KeyTests(unittest.TestCase):
    def test_band_keys(self):
        s=run([55,56,57]);self.assertFalse(s["band"].enable)
    def test_keypad(self):
        s=run([329]);self.assertFalse(s["band"].enable)
    def test_pause(self):
        self.assertTrue(run([32])["paused"])
        self.assertFalse(run([32,32])["paused"])
    def test_fault_space_cannot_resume(self):
        s=run([32],fault="test fault",paused=True)
        self.assertTrue(s["paused"]);self.assertEqual(s["fault_reason"],"test fault")
    def test_fault_reset_then_resume(self):
        s=run([82,32],fault="test fault",paused=True)
        self.assertIsNone(s["fault_reason"]);self.assertFalse(s["paused"])
        self.assertEqual(s["epoch"],3);self.assertEqual(s["tick"],0)
if __name__=="__main__":unittest.main()
