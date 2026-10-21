import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from gear_sonic.utils.vibe_config import generate, digest, load

class ConfigTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.root=Path(self.tmp.name);self.bundle=self.root/'bundle';self.bundle.mkdir()
        self.case=self.root/'case';self.case.mkdir();self.vibe=self.root/'vibe'
        helper=self.vibe/'src/vibe/sim2sim/runtime.py';helper.parent.mkdir(parents=True);helper.write_text('fixture')
        self.vision=self.root/'theia.onnx';self.vision.write_bytes(b'vision')
        latent={'shape':[1,64]}
        ports={'latent':[64],'policy':[930],'augmentation':[6],'q_proprio':[64],'q_cls':[192],'kv_tokens__img_tokens':[21,192]}
        action={'type':'joint_position','joint_names':[str(i) for i in range(29)],
                **{k:[1.]*29 for k in ('scale','default_joint_pos','stiffness','damping')}}
        self.decoder={'latent_contract':latent,'action':action,'inputs':[{'name':k,'shape':v,'terms':[]} for k,v in ports.items()],
                      'control':{'sim_timestep':.005,'decimation':4,'step_dt':.02}}
        self.write(self.bundle/'visual_decoder.manifest.json',self.decoder)
        self.write(self.bundle/'motion_encoder.manifest.json',{'latent_contract':latent,'inputs':[]})
        for name in ('motion_encoder.onnx','visual_decoder.onnx'):(self.bundle/name).write_bytes(name.encode())
        self.write(self.vision.with_suffix('.manifest.json'),{'input':{'layout':'bgr_hwc_uint8','width':112,'height':63},'hf_id':'theia'})
        self.write(self.bundle/'bundle.json',{'schema':'vibe.split_sonic.v1','status':'passed','latent_contract':latent,
                     'files':{p.name:digest(p) for p in self.bundle.iterdir()}})
        for name in ('scene.mjb','initial.npz','reference.npz'):(self.case/name).write_bytes(name.encode())
        self.write(self.case/'case.json',{'files':{p.name:digest(p) for p in self.case.iterdir()},
                   'bundle_sha256':digest(self.bundle/'bundle.json'),'joint_names':action['joint_names'],
                   'physics_dt':.005,'control_dt':.02,'clip_actions':None,
                   'camera':{'width':112,'height':63,'fovy':42.5,'pos':[0,0,0],'quat':[1,0,0,0],
                             'geom_groups':[0,2],'shadows':False,'textures':True}})
    def write(self,p,obj):p.write_text(json.dumps(obj))
    def config(self):return generate(self.bundle,self.vision,self.case,self.vibe)
    def test_round_trip(self):
        cfg=self.config();p=self.root/'config.json';self.write(p,cfg);self.assertEqual(load(p),cfg)
    def test_weight_tampering(self):
        (self.bundle/'motion_encoder.onnx').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'hash mismatch'):self.config()
    def test_scene_tampering(self):
        (self.case/'scene.mjb').write_bytes(b'changed')
        with self.assertRaisesRegex(ValueError,'hash mismatch'):self.config()
    def test_modified_config(self):
        cfg=self.config();cfg['action']['scale'][0]=999;p=self.root/'config.json';self.write(p,cfg)
        with self.assertRaisesRegex(ValueError,'stale or edited'):load(p)
    def test_camera_mismatch(self):
        path=self.case/'case.json';c=json.loads(path.read_text());c['camera']['fovy']=58;self.write(path,c)
        with self.assertRaisesRegex(ValueError,'camera'):self.config()
    def test_missing_hash_entries(self):
        path=self.bundle/'bundle.json';b=json.loads(path.read_text());b['files'].pop('motion_encoder.onnx');self.write(path,b)
        with self.assertRaisesRegex(ValueError,'Incomplete'):self.config()
    def test_real_and_manager_rejected(self):
        script=Path(__file__).resolve().parents[1]/'scripts/run_vibe_sim.py'
        base=[sys.executable,str(script),'--policy-type','vibe','--bundle','x','--vision-encoder','x','--obs-config','x']
        for tail in (['real'],['--input-type','zmq_manager','sim']):
            result=subprocess.run(base+tail,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertIn('invalid choice',result.stderr)
if __name__=='__main__':unittest.main()
