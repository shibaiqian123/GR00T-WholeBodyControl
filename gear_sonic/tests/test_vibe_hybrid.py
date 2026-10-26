"""Offline gates for the PICO hybrid observation contract; no DDS or robot commands."""
import json
import sys
import unittest
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from gear_sonic.utils.vibe_hybrid import LiveWindow

def packet(i):
    return {"schema":"groot.vibe.pose.v1","session":"test","sample_ns":i*20_000_000,
            "received_monotonic":1.+i*.02,"joints_local":np.zeros((24,3)).tolist(),
            "root_quat":[1,0,0,0],"root_translation_zup":[0,0,1],"wrist":[0]*6}

class ContractTests(unittest.TestCase):
    def test_order(self):
        from gear_sonic.utils.vibe_hybrid import HARDWARE_JOINTS,validate_hardware_order
        validate_hardware_order({"action":{"joint_names":list(HARDWARE_JOINTS)}})
        with self.assertRaises(ValueError):
            validate_hardware_order({"action":{"joint_names":list(reversed(HARDWARE_JOINTS))}})
    def test_duplicate(self):
        w=LiveWindow();w.append(packet(0))
        with self.assertRaises(ValueError):w.append(packet(0))
    def test_future_stamp(self):
        w=LiveWindow()
        for i in range(11):w.append(packet(i))
        with self.assertRaisesRegex(ValueError,"future"):w.observations(None,None,None,0.)
    def test_warmup(self):
        w=LiveWindow()
        for i in range(10):w.append(packet(i))
        with self.assertRaisesRegex(ValueError,"warming"):w.observations(None,None,None,1.2)
    def test_gap(self):
        w=LiveWindow();w.append(packet(0))
        with self.assertRaisesRegex(ValueError,"discontinuity"):w.append(packet(2))
        self.assertEqual(len(w.rows),0)
    def test_session(self):
        w=LiveWindow();w.append(packet(0));p=packet(100);p["session"]="new";w.append(p)
        self.assertEqual(len(w.rows),1)
    def test_nonfinite(self):
        p=packet(0);p["wrist"][0]=float("nan")
        with self.assertRaises(ValueError):LiveWindow().append(p)
    def test_nonunit(self):
        p=packet(0);p["root_quat"]=[0,0,0,0]
        with self.assertRaisesRegex(ValueError,"Nonunit"):LiveWindow().append(p)
    def test_stale(self):
        w=LiveWindow()
        for i in range(11):w.append(packet(i))
        with self.assertRaisesRegex(ValueError,"stale"):w.observations(None,None,None,5.)
    def test_official_tokenizer(self):
        import torch
        import mjlab
        from mjlab.utils.lab_api.math import quat_apply_inverse, quat_mul, quat_conjugate, axis_angle_from_quat
        from mocke.mdp.joint_maps import MJ2IL
        from orcs.core.data.point_reference import scaled_smpl_targets, infer_morphology_scale
        from vibe.sim2sim.runtime import NativeCase
        config=json.loads(Path("/home/cqu023/orcs-exports/stairs_split_v1/focus/split_sonic/visual_decoder.manifest.json").read_text())
        case=NativeCase("/home/cqu023/sonic-diagnostics/vibe-native-v1/case_stair_p2_6768415cac75882b",config)
        # Give the live path the same true samples as the recorded official tokenizer.
        start=10
        w=LiveWindow()
        for i in range(11):
            f=start+i-1
            q=torch.tensor(case.reference["smpl_root_quat"][f])
            raw=torch.tensor(case.reference["smpl_joints"][f])
            local=quat_apply_inverse(q.expand(24,-1),raw)
            p=packet(i)
            p["root_quat"]=q.tolist();p["joints_local"]=local.tolist()
            p["root_translation_zup"]=[0,0,0]
            p["wrist"]=case.reference["joint_pos"][f][MJ2IL][23:29].tolist()
            w.append(p)
        case.frame=start
        t,q,_=w.observations(case.data.xquat[case.root].astype(np.float32),
                            case.data.xpos[case.points].astype(np.float32),
                            case.meta["point_body_names"],1.2)
        expected=case.observations()["tokenizer"]
        err=float(np.abs(t-expected).max())
        print("LIVE_TOKENIZER_MAX_ABS",err)
        self.assertLessEqual(err,1e-5)
        self.assertTrue(np.isfinite(q).all())
if __name__=="__main__":unittest.main()
