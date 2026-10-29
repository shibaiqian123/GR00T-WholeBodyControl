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
        for k in ('SONIC_JOINT_TARGET_LIMITS','SONIC_ARM_TARGET_SMOOTHING','SONIC_TARGET_SMOOTHING','SONIC_SINGLE_STEP_CONFIG','VIBE_HYBRID_ENDPOINT'):
            env.pop(k,None)
        env.update(extra or {})
        return subprocess.run([sys.executable,str(LAUNCH),'--inference-mode','async','--dry-run',*args],env=env,text=True,capture_output=True)
    def test_disabled_explicitly(self):
        r=self.run_launcher()
        self.assertEqual(r.returncode,0,r.stderr)
        self.assertIn('env -u SONIC_JOINT_TARGET_LIMITS -u SONIC_ARM_TARGET_SMOOTHING -u SONIC_TARGET_SMOOTHING ./deploy.sh',r.stdout)
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
    def test_full_body_smoothing(self):
        path=LAUNCH.parents[1]/'configs/joint_target_smoothing.draft.json'
        rejected=self.run_launcher('--target-smoothing',str(path))
        self.assertNotEqual(rejected.returncode,0)
        self.assertIn('tracking_error_source',rejected.stderr)
        retired=self.run_launcher('--arm-target-smoothing',str(path))
        self.assertNotEqual(retired.returncode,0)
        with tempfile.TemporaryDirectory() as tmp:
            valid=json.loads(path.read_text())
            valid['max_start_tracking_error']=[.2]*29
            valid['tracking_error_source']='Synthetic dry-run test, not hardware limits'
            file=Path(tmp)/'synthetic.json';file.write_text(json.dumps(valid))
            result=self.run_launcher('--target-smoothing',str(file))
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertIn('TARGET SMOOTHING: full-body common takeover',result.stdout)
            self.assertIn('SONIC_TARGET_SMOOTHING=',result.stdout)
            for key,value in [('joint_names',['wrong']*29),('simulation_only',True),('max_start_tracking_error',[None]*29),('v_max',[0]*29),('tau',[-.1]*29)]:
                bad=dict(valid);bad[key]=value;file.write_text(json.dumps(bad))
                self.assertNotEqual(self.run_launcher('--target-smoothing',str(file)).returncode,0)
    def test_missing_file(self):
        r=self.run_launcher('--joint-target-limits','/nonexistent/limits.json')
        self.assertNotEqual(r.returncode,0)

if __name__=='__main__':unittest.main()
