import unittest
import subprocess
import sys
import re
import ast
from pathlib import Path
import numpy as np
from gear_sonic.utils.inference.readonly_body import *

class ReadOnlyTests(unittest.TestCase):
    def test_mapping_and_offset(self):
        from gear_sonic.data.robot_model.instantiation.g1 import instantiate_g1_robot_model
        from gear_sonic.utils.inference.dit4dit_control import sonic_state
        q=np.arange(29)*.031-.4; offsets=np.arange(29)*.017+.1
        logged=q[list(MUJOCO_TO_ISAACLAB)]-offsets[list(MUJOCO_TO_ISAACLAB)]
        published=logged[list(ISAACLAB_TO_MUJOCO)]+offsets
        np.testing.assert_allclose(dds_to_body_q(q),published,atol=1e-15)
        robot=instantiate_g1_robot_model(waist_location='lower_and_upper_body')
        whole=robot.get_configuration_from_actuated_joints(body_actuated_joint_values=published)
        independent=robot.get_configuration_from_actuated_joints(body_actuated_joint_values=dds_to_body_q(q))
        expected=np.concatenate([whole[:22],whole[29:36],[.12,.34]])
        np.testing.assert_allclose(sonic_state(independent,[.12,.34]),expected)
        self.assertEqual(expected.shape,(31,))
    def test_mapping_matches_cpp_and_no_motor_publisher(self):
        root=Path(__file__).resolve().parents[3]
        header=root/'gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/include/policy_parameters.hpp'
        text=header.read_text()
        for name,expected in [('isaaclab_to_mujoco',ISAACLAB_TO_MUJOCO),('mujoco_to_isaaclab',MUJOCO_TO_ISAACLAB)]:
            block=re.search(name+r'[^=]*=\s*\{([^}]+)',text).group(1)
            actual=tuple(map(int,re.findall(r'\b\d+\b',re.sub(r'//[^\n]*','',block))))
            self.assertEqual(actual,expected)
        script=root/'gear_sonic/scripts/run_dit4dit_body_observer.py'
        tree=ast.parse(script.read_text())
        names=[x.id for x in ast.walk(tree) if isinstance(x,ast.Name)]
        self.assertNotIn('ChannelPublisher',names)
        self.assertNotIn('MotorCmd_',names)
        self.assertIn('ChannelSubscriber',names)

    def test_source_duplicates_stale_nonfinite(self):
        c=LowStateCache();self.assertFalse(c.packet('x',0,1)['valid'])
        self.assertTrue(c.accept(1,np.arange(29),1))
        self.assertFalse(c.accept(1,np.arange(29),1.1))
        self.assertEqual(c.stamp,1)
        self.assertTrue(c.packet('x',1,1.1)['valid'])
        self.assertEqual(c.duplicates,1)
        self.assertFalse(c.accept(1,np.arange(29),1.3))
        self.assertFalse(c.packet('x',2,1.3)['valid'])
        self.assertEqual(c.stamp,1)
        self.assertTrue(c.accept(2,np.arange(29),1.2))
        self.assertFalse(c.packet('x',2,1.46)['valid'])
        self.assertFalse(c.accept(3,[float('nan')]*29,1.5))
        self.assertFalse(c.accept(0,np.arange(29),1.6))
        self.assertTrue(advancing_tick(2**32-1,0))
    def test_duplicate_content_ignored_and_fault_not_cleared(self):
        c=LowStateCache();q=np.arange(29);c.accept(1,q,1.)
        self.assertFalse(c.accept(1,q+1,1.02))
        self.assertTrue(c.packet('x',1,1.03)['valid'])
        np.testing.assert_array_equal(c.q,q)
        self.assertEqual(c.stamp,1.)
        self.assertFalse(c.accept(1,[float('nan')]*29,1.035))
        self.assertFalse(c.accept(1,q,1.04))
        self.assertFalse(c.packet('x',2,1.05)['valid'])
        self.assertTrue(c.accept(2,q,1.06))
        self.assertTrue(c.packet('x',3,1.07)['valid'])

    def test_receiver_duplicate_cannot_renew(self):
        c=LowStateCache();c.accept(1,np.arange(29),1)
        r=PacketCache();p=c.packet('x',1,1.01)
        self.assertTrue(r.accept(p,1.02))
        self.assertFalse(r.accept(p,1.03))
        self.assertIsNone(r.current(1.26))
        p=dict(p,seq=2,time=1.1,sample_time=1.1)
        self.assertFalse(r.accept(p,1.12))
        self.assertIsNone(r.current(1.12))
    def test_mutual_exclusion(self):
        p=subprocess.run([sys.executable,'-m','gear_sonic.scripts.run_dit4dit_gateway',
          '--execute','--readonly-body-port','5564','--limits','/missing',
          '--task-prompt','test','--log-dir','/missing'],capture_output=True,text=True)
        self.assertEqual(p.returncode,2)
        self.assertIn('cannot be combined',p.stderr)

if __name__=='__main__':unittest.main()
