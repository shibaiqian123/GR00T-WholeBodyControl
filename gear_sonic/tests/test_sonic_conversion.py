"""Converter boundary tests, entirely synthetic and offline."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import av
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
from gear_sonic.scripts import convert_sonic_to_dit4dit as c


def fixture(root):
    n=6;fps=50
    features={k:{'dtype':'float64','shape':[dim],'names':[k+str(i) for i in range(dim)]}
              for k,dim in c.REQUIRED.items()}
    for k in c.AUTO|{'dex1.session_id','dex1.control_state','dex1.feedback_valid','dex1.command_valid','dex1.trigger_valid'}:
        if k!='timestamp':features[k]['dtype']='int64'
    features['observation.images.ego_view']={'dtype':'video','shape':[16,16,3],'names':['height','width','channel'],
        'info':{'video.fps':50,'video.height':16,'video.width':16,'video.channels':3}}
    info={'codebase_version':'v2.1','fps':fps,'total_episodes':3,'total_frames':3*n,'total_tasks':1,
          'total_videos':3,'total_chunks':1,'chunks_size':1000,'splits':{'train':'0:3'},
          'data_path':'data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet',
          'video_path':'videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4',
          'features':features,'discarded_episode_indices':[0],
          'script_config':{'dex1_recording':{'version':1,'phase_codes':{'READY':4},'calibration':{}}}}
    eps=[];eps_stats=[]
    for eid in range(3):
        data={k:np.full((n,dim),.25).tolist() for k,dim in c.REQUIRED.items()}
        for k in ['dex1.feedback_valid','dex1.command_valid','dex1.trigger_valid']:data[k]=[[1,1]]*n
        data.update(timestamp=(np.arange(n)/fps).tolist(),frame_index=list(range(n)),episode_index=[eid]*n,
                    index=list(range(eid*n,(eid+1)*n)),task_index=[0]*n,
                    **{'dex1.session_id':[10]*n,'dex1.control_state':[4]*n})
        data['observation.dex1_position']=[[1+i*.1,2-i*.1] for i in range(n)]
        data['action.dex1_target_position']=[[1.5+i*.1,2.5-i*.1] for i in range(n)]
        if eid==2:data['dex1.command_valid']=[[0,1]]+[[1,1]]*(n-1)
        params={'episode_index':eid,'episode_chunk':0}
        p=root/info['data_path'].format(**params);p.parent.mkdir(parents=True,exist_ok=True)
        pq.write_table(pa.table(data),p)
        v=root/info['video_path'].format(**params,video_key='observation.images.ego_view')
        v.parent.mkdir(parents=True,exist_ok=True)
        with av.open(str(v),'w') as video:
            stream=video.add_stream('libx264',rate=fps);stream.width=16;stream.height=16;stream.pix_fmt='yuv420p'
            for i in range(n):
                for pkt in stream.encode(av.VideoFrame.from_ndarray(np.full((16,16,3),i*20,dtype=np.uint8),format='rgb24')):video.mux(pkt)
            for pkt in stream.encode():video.mux(pkt)
        eps.append({'episode_index':eid,'length':n,'tasks':['test task']})
        eps_stats.append({'episode_index':eid,'stats':{'observation.images.ego_view':{'count':[n]}}})
    c.write_json(root/'meta/info.json',info)
    c.write_json(root/'meta/modality.json',{})
    c.write_lines(root/'meta/episodes.jsonl',eps)
    c.write_lines(root/'meta/episodes_stats.jsonl',eps_stats)
    c.write_lines(root/'meta/tasks.jsonl',[{'task_index':0,'task':'test task'}])
    return info,eps


class ConversionTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)/'input';self.out=Path(self.tmp.name)/'output'
        self.info,self.eps=fixture(self.root)
    def tearDown(self):self.tmp.cleanup()
    def test_filter_preserve_and_metadata(self):
        before={p:c.sha256(p) for p in self.root.rglob('*') if p.is_file()}
        report=c.convert(self.root,self.out)
        self.assertEqual(report['accepted'],[{'source_episode':1,'output_episode':0,'frames':6}])
        self.assertEqual(len(report['excluded']),2)
        a=c.arrays(pq.read_table(self.root/'data/chunk-000/episode_000001.parquet'))
        b=c.arrays(pq.read_table(self.out/'data/chunk-000/episode_000000.parquet'))
        for k in a:
            if k not in ('episode_index','index'):np.testing.assert_array_equal(a[k],b[k])
        np.testing.assert_array_equal(b['observation.sonic_state'],np.concatenate([a['observation.state'][:,c.BODY_INDICES],a['observation.dex1_position']],1))
        self.assertEqual(c.read_json(self.out/'meta/info.json')['total_frames'],6)
        self.assertTrue((self.out/'meta/stats_gr00t.json').exists())
        self.assertEqual(before,{p:c.sha256(p) for p in before})
    def test_existing_output_refused(self):
        self.out.mkdir()
        with self.assertRaises(FileExistsError):c.convert(self.root,self.out)
    def test_source_changed(self):
        original=c.check_video
        def mutate(*args):
            original(*args)
            p=self.root/'meta/info.json';p.write_text(p.read_text()+' ')
        with patch.object(c,'check_video',side_effect=mutate):
            with self.assertRaisesRegex(RuntimeError,'Source changed'):c.convert(self.root,self.out)
        self.assertFalse(self.out.exists())
    def test_dry_run(self):
        self.assertEqual(c.convert(self.root,self.out,True)['frames'],6)
        self.assertFalse(self.out.exists())
    def test_no_eligible(self):
        self.info['discarded_episode_indices']=[0,1,2];c.write_json(self.root/'meta/info.json',self.info)
        with self.assertRaisesRegex(ValueError,'No eligible'):c.convert(self.root,self.out)
    def test_missing_dimension_nonfinite_session_state(self):
        p=self.root/'data/chunk-000/episode_000001.parquet';t=pq.read_table(p)
        def check(table):return c.inspect_episode(table,self.eps[1],50,{0:'test task'},4)
        self.assertEqual(check(t),[])
        self.assertIn('missing:action.motion_token',check(t.drop(['action.motion_token'])))
        for key,values,reason in [
            ('action.motion_token',[[.2]*63]*6,'shape:action.motion_token'),
            ('observation.dex1_position',[[float('nan'),1]]*6,'nonfinite:observation.dex1_position'),
            ('dex1.session_id',[1,1,1,2,2,2],'telemetry_session_change'),
            ('dex1.control_state',[4,4,4,5,4,4],'non_READY_control_state')]:
            changed=t.set_column(t.schema.get_field_index(key),key,pa.array(values))
            self.assertIn(reason,check(changed))


if __name__=='__main__':unittest.main()
