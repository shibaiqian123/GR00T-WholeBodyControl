import unittest
from unittest.mock import patch
from gear_sonic.scripts.run_sim_startup_audit import load_cases,check_ports
from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig
import socket

class SimAuditTests(unittest.TestCase):
    def test_domain_override_preserves_default(self):
        default=SimLoopConfig().load_wbc_yaml()
        isolated=SimLoopConfig(interface='lo',dds_domain=38).load_wbc_yaml()
        self.assertEqual(isolated['DOMAIN_ID'],38)
        self.assertEqual(isolated['INTERFACE'],'lo')
        for key in ('MOTOR_KP','MOTOR_KD','DEFAULT_MOTOR_ANGLES','ENABLE_ELASTIC_BAND','SIMULATE_DT'):
            self.assertEqual(default[key],isolated[key])
        with self.assertRaises(ValueError):SimLoopConfig(dds_domain=-1).load_wbc_yaml()
    def test_original_order_and_prefill(self):
        cases=load_cases()
        self.assertEqual(len(cases),3)
        self.assertEqual([c['start'] for c in cases],sorted(c['start'] for c in cases))
        for c in cases:
            self.assertEqual(sum(e['kind']=='start' for e in c['schedule']),1)
            frames=[e for e in c['schedule'] if e['kind']=='frame']
            self.assertTrue(frames[0]['index']>0)
            self.assertLess(frames[0]['time'],c['start'])
            self.assertTrue(all(len(f['token'])==64 for f in frames))
    def test_port_conflict_fails(self):
        with socket.socket() as s:
            s.bind(('127.0.0.1',15556));s.listen()
            with self.assertRaises(OSError):check_ports()

if __name__=='__main__':unittest.main()
