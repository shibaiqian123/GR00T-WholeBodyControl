import tempfile
import unittest
from pathlib import Path
from gear_sonic.scripts.prepare_sim_single_step import make_config

class ConfigTests(unittest.TestCase):
    def test_distinct_joint_values_preserve_hardware_order(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            header=root/"gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp"
            urdf=root/"gear_sonic/data/robot_model/model_data/g1/g1_29dof_with_hand.urdf"
            header.parent.mkdir(parents=True);urdf.parent.mkdir(parents=True)
            names=[f"test_{i}_joint" for i in range(29)]
            header.write_text("g1_action_scale = {\n"+"".join(f"1, // {n}\n" for n in names)+"};")
            # XML order deliberately differs from the hardware order.
            urdf.write_text("<robot>"+"".join(f'<joint name="{names[i]}"><limit lower="{-i-1}" upper="{i+2}" velocity="{i+3}"/></joint>' for i in reversed(range(29)))+"</robot>")
            c=make_config(root)
            self.assertTrue(c["simulation_only"])
            self.assertEqual(c["joint_names"],names)
            for i in range(29):
                self.assertEqual(c["q_min"][i],-i-1)
                self.assertEqual(c["q_max"][i],i+2)
                self.assertAlmostEqual(c["max_delta"][i],(i+3)*.02)
            urdf.write_text(urdf.read_text().replace('velocity="3"','velocity="0"'))
            with self.assertRaises(ValueError):make_config(root)

    def test_real_asset_layout(self):
        c=make_config()
        self.assertEqual(c["joint_names"][15],"left_shoulder_pitch_joint")
        self.assertEqual(len(c["q_min"]),29)
        self.assertEqual(len(c["assets"]),2)

if __name__=="__main__":unittest.main()
