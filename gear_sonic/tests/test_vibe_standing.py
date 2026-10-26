"""Training-scene initialization and original-band regression; no DDS or policy takeover."""
import json,unittest
from pathlib import Path
import numpy as np
import mjlab
from vibe.sim2sim.runtime import NativeCase
from gear_sonic.utils.vibe_sim_support import standing_reset,original_band,apply_band

class StandingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        manifest=json.loads(Path('/home/cqu023/orcs-exports/stairs_split_v1/focus/split_sonic/visual_decoder.manifest.json').read_text())
        cls.case=NativeCase('/home/cqu023/sonic-diagnostics/vibe-native-v1/case_stair_p2_6768415cac75882b',manifest)
    def test_standing_preserves_assets(self):
        c=self.case
        before={k:getattr(c.model,k).copy() for k in ('geom_group','actuator_gainprm','actuator_biasprm','actuator_forcerange','cam_pos','cam_quat')}
        result=standing_reset(c)
        self.assertEqual(result['floor_height'],0.)
        np.testing.assert_allclose(c.data.xquat[c.root][1:3],[0,0],atol=1e-12)
        self.assertAlmostEqual(float(c.data.xmat[c.root].reshape(3,3)[2,2]),1.)
        self.assertLess(result['heading_radians'],-1.)
        np.testing.assert_allclose(c.data.qpos[c.qids],c.default,atol=1e-7)
        self.assertGreaterEqual(result['minimum_contact_distance'],-1e-5)
        for k,v in before.items():np.testing.assert_array_equal(getattr(c.model,k),v)
    def test_band_can_clear_force(self):
        c=self.case;info=standing_reset(c);band,_=original_band()
        band.point=np.array([*info['root_position'][:2],1.]);band.reference_heading=info["heading_radians"]
        apply_band(c,band)
        self.assertGreater(c.data.xfrc_applied[c.root,2],0)
        np.testing.assert_allclose(c.data.xfrc_applied[c.root,3:],0,atol=1e-10)
        band.enable=False;apply_band(c,band)
        np.testing.assert_array_equal(c.data.xfrc_applied[c.root],np.zeros(6))
    def test_seed_diagnostic_unchanged(self):
        c=self.case;standing_reset(c);c.reset()
        for key in ('qpos','qvel','ctrl'):np.testing.assert_array_equal(getattr(c.data,key),c.initial[key])

if __name__=='__main__':unittest.main()
