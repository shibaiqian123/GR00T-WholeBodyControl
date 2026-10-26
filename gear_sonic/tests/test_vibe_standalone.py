"""Independent deployment checks; no training package imports."""
import importlib.util,json,unittest
from pathlib import Path
import numpy as np
from gear_sonic.utils.standalone.native import NativeCase
from gear_sonic.utils.vibe_sim_support import standing_reset,original_band,apply_band
ROOT=Path("/home/cqu023/sonic-diagnostics/vibe-standalone-v1")
class StandaloneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from gear_sonic.utils.vibe_config import load
        cls.cfg=load(ROOT/"artifacts/observation_config.yaml")
        m=json.loads(Path(cls.cfg["bundle"]).with_name("visual_decoder.manifest.json").read_text())
        cls.case=NativeCase(cls.cfg["case"],m)
    def test_training_packages_absent(self):
        for name in ("mjlab","orcs","mocke","vibe"):
            self.assertIsNone(importlib.util.find_spec(name))
    def test_initial_observation_oracle(self):
        c=self.case;c.reset()
        for key,value in c.observations().items():
            np.testing.assert_allclose(value,c.initial["oracle__"+key],rtol=0,atol=1e-5)
    def test_assets_and_standing(self):
        c=self.case
        fields=("actuator_gainprm","actuator_biasprm","actuator_forcerange","cam_pos","cam_quat")
        before={k:getattr(c.model,k).copy() for k in fields}
        result=standing_reset(c)
        self.assertGreaterEqual(result["minimum_contact_distance"],-1e-5)
        for k,v in before.items():np.testing.assert_array_equal(getattr(c.model,k),v)
    def test_native_step_and_reset(self):
        c=self.case;standing_reset(c);band,_=original_band()
        band.enable=True;band.point=np.array([*c.data.xpos[c.root,:2],1.])
        for _ in range(40):
            apply_band(c,band);c.mujoco.mj_step(c.model,c.data)
        self.assertTrue(np.isfinite(c.data.qpos).all())
        self.assertAlmostEqual(c.data.time,.2)
        c.reset()
        np.testing.assert_array_equal(c.data.qpos,c.initial["qpos"])
    def test_window_guards(self):
        from gear_sonic.utils.vibe_hybrid import LiveWindow
        with self.assertRaisesRegex(ValueError,"warming"):LiveWindow().observations(None,None,None,0)
    def test_rgb_layout(self):
        from gear_sonic.utils.standalone.native import bgr_input
        rgb=np.zeros((63,112,3),np.uint8);rgb[:,:,0]=99
        value=bgr_input(rgb)
        self.assertEqual(value.shape,(1,63,112,3))
        self.assertTrue((value[0,:,:,2]==99).all())
if __name__=="__main__":unittest.main()
