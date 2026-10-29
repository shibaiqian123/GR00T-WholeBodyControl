"""Launcher tests only invoke --dry-run; synthetic bounds never reach motors."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

LAUNCH=Path(__file__).with_name('launch_dit4dit_field.py')
class LauncherLimitsTest(unittest.TestCase):
    def run_launcher(self, *args, extra=None):
        env=dict(os.environ)
        for k in ('SONIC_JOINT_TARGET_LIMITS','SONIC_ARM_TARGET_SMOOTHING','SONIC_SINGLE_STEP_CONFIG','VIBE_HYBRID_ENDPOINT'):
            env.pop(k,None)
        env.update(extra or {})
        return subprocess.run([sys.executable,str(LAUNCH),'--inference-mode','async','--dry-run',*args],env=env,text=True,capture_output=True)
    def test_disabled_explicitly(self):
        r=self.run_launcher()
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('env -u SONIC_JOINT_TARGET_LIMITS -u SONIC_ARM_TARGET_SMOOTHING ./deploy.sh',r.stdout)
    def test_configuration(self):
        with tempfile.TemporaryDirectory(prefix='limiter test ') as tmp:
            path=Path(tmp)/'synthetic.json'
            c=dict(units='rad',joint_order='hardware_29',constraint_source='SYNTHETIC DRY RUN ONLY',q_min=[-1]*29,q_max=[1]*29,max_velocity=[.1]*29,max_tracking_error=[.1]*29)
            path.write_text(json.dumps(c))
            for args,env in [(('--joint-target-limits',str(path)),{}),((),{'SONIC_JOINT_TARGET_LIMITS':str(path)})]:
                r=self.run_launcher(*args,extra=env)
                self.assertEqual(r.returncode,0,r.stderr)
                self.assertIn('JOINT TARGET LIMITER: enabled',r.stdout)
                self.assertIn('bottom_target_audit.jsonl',r.stdout)
                self.assertIn('joint_target_limits.json',r.stdout)
            for key,value in [('simulation_only',True),('max_velocity',[0]*29),('q_max',[1]*28),('max_tracking_error',[float('nan')]*29)]:
                bad=dict(c);bad[key]=value;path.write_text(json.dumps(bad))
                r=self.run_launcher('--joint-target-limits',str(path))
                self.assertNotEqual(r.returncode,0)
                self.assertNotIn('gateway\n',r.stdout)
    def test_experimental_arm_configuration(self):
        path=LAUNCH.parents[1]/'configs/arm_target_smoothing.experimental.json'
        r=self.run_launcher('--arm-target-smoothing',str(path))
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('ARM TARGET SMOOTHING: enabled',r.stdout)
        self.assertIn('T_blend=0.5 tau=0.1 v_max=0.5',r.stdout)
        self.assertIn('SONIC_ARM_TARGET_SMOOTHING=',r.stdout)
        self.assertIn('bottom_target_audit.jsonl',r.stdout)
        with tempfile.TemporaryDirectory() as tmp:
            bad=json.loads(path.read_text());bad['joint_names'][15]='wrong'
            file=Path(tmp)/'bad.json';file.write_text(json.dumps(bad))
            self.assertNotEqual(self.run_launcher('--arm-target-smoothing',str(file)).returncode,0)
            bad=json.loads(path.read_text());bad['simulation_only']=True;file.write_text(json.dumps(bad))
            self.assertNotEqual(self.run_launcher('--arm-target-smoothing',str(file)).returncode,0)
    def test_missing_file(self):
        r=self.run_launcher('--joint-target-limits','/nonexistent/limits.json')
        self.assertNotEqual(r.returncode,0)

if __name__=='__main__':unittest.main()
