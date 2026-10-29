import importlib.util,json,tempfile,unittest
from pathlib import Path
import numpy as np
spec=importlib.util.spec_from_file_location('steps',Path(__file__).with_name('dit_sonic_steps.py'));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
class Tests(unittest.TestCase):
 def snapshot(self):
  return dict(kind='readonly_decode_snapshot',schema_name='sonic_v1_1_prospective_control_v1',state='WAIT_FOR_CONTROL',decoder_input=[0]*994,actual_q=[0]*29,actual_dq=[0]*29,scale=[1]*29,default_angles=[0]*29,mapping=list(range(29)),sample_time=10.,padding_rows=list(range(9)))
 def test_complete(self):self.assertEqual(m.validate_snapshot(self.snapshot())['state'],'WAIT_FOR_CONTROL')
 def test_missing_history(self):
  s=self.snapshot();s['decoder_input']=[0]*993
  with self.assertRaises(ValueError):m.validate_snapshot(s)
 def test_invalid_mapping(self):
  s=self.snapshot();s['mapping']=[0]*29
  with self.assertRaises(ValueError):m.validate_snapshot(s)
 def test_nan(self):
  s=self.snapshot();s['actual_q'][0]=float('nan')
  with self.assertRaises(ValueError):m.validate_snapshot(s)
 def test_padding_provenance(self):
  s=self.snapshot();del s['padding_rows']
  with self.assertRaises(ValueError):m.validate_snapshot(s)
 def test_freshness(self):
  d=dict(camera_age_seconds=.01,left_feedback_age_seconds=.01,right_feedback_age_seconds=.01)
  m.paired(self.snapshot(),d,{'sample_time':10},10.02)
  with self.assertRaises(ValueError):m.paired(self.snapshot(),d,{'sample_time':10},10.3)
 def test_bundle_without_decoder(self):
  with tempfile.TemporaryDirectory() as t:
   p=Path(t);np.savez(p/'observation.npz',state=np.arange(31));m.save_json(p/'manifest.json',{'files':{'observation.npz':m.sha(p/'observation.npz')}})
   self.assertIsNone(m.load_bundle(p)[1]);(p/'observation.npz').write_bytes(b'changed')
   with self.assertRaises(ValueError):m.load_bundle(p)
if __name__=='__main__':unittest.main()
