import importlib.util,json,tempfile,unittest,sys
from argparse import Namespace
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
 def test_decode_31_with_mock_decoder(self):
  with tempfile.TemporaryDirectory() as t:
   root=Path(t);cap=root/'capture';pred=root/'prediction';cap.mkdir();pred.mkdir()
   snap=self.snapshot();snap['mapping']=list(range(28,-1,-1));snap['scale']=[2]*29;snap['default_angles']=[.5]*29
   np.savez(cap/'observation.npz',state=np.zeros(31));m.save_json(cap/'sonic_snapshot.json',snap)
   m.save_json(cap/'manifest.json',{'files':{k:m.sha(cap/k) for k in ('observation.npz','sonic_snapshot.json')}})
   m.save_json(pred/'source.json',{'manifest_hash':m.sha(cap/'manifest.json')})
   actions=np.zeros((50,66));actions[0,64:]=[4.2,4.3]
   m.save_json(pred/'request.json',{'request_id':'fixture'})
   m.save_json(pred/'response.json',{'ok':True,'request_id':'fixture','actions':actions.tolist()})
   np.save(pred/'token_step0.npy',actions[0,:64]);np.save(pred/'gripper_step0.npy',actions[0,64:])
   decoder=root/'mock_decoder';model=root/'model';model.write_text('synthetic fixture only')
   meta={k:snap[k] for k in ('mapping','scale','default_angles')}
   decoder.write_text('#!'+sys.executable+'\nimport sys,json,struct\nfrom pathlib import Path\np=Path(sys.argv[3]);p.write_bytes(struct.pack("<29f",*range(29)))\nPath(str(p)+".mapping.json").write_text('+repr(json.dumps(meta))+')\n')
   decoder.chmod(0o700)
   m.decode(Namespace(capture=cap,prediction=pred,sonic_snapshot=None,output=root/'decoded',decoder=decoder,model=model))
   action=np.load(root/'decoded/action31.npy');self.assertEqual(action.shape,(31,))
   np.testing.assert_array_equal(action[:29],np.arange(28,-1,-1)*2+.5)
   np.testing.assert_array_equal(action[29:],[4.2,4.3])
   self.assertFalse(json.loads((root/'decoded/result.json').read_text())['executed'])
if __name__=='__main__':unittest.main()
