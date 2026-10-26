"""Offline end-to-end LeRobot save test; synthetic images and no hardware."""
import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
import pyarrow.parquet as pq

from gear_sonic.tests.test_dex1_recording import packet
from gear_sonic.utils.teleop.dex1_recording import SampleCache, metadata, check_dataset


class SaveTests(unittest.TestCase):
    def test_full_schema_video_parquet_and_resume(self):
        from gear_sonic.data.features_sonic_vla import (
            get_g1_robot_model, get_features_sonic_vla, get_modality_config_sonic_vla,
            add_dex1_recording_features)
        from gear_sonic.data.exporter import Gr00tDataExporter
        model=get_g1_robot_model()
        features=get_features_sonic_vla(model)
        modality=get_modality_config_sonic_vla(model)
        add_dex1_recording_features(features,modality)
        with tempfile.TemporaryDirectory(prefix='dex1-save-test-') as tmp:
            root=Path(tmp)/'dataset'
            def create():
                return Gr00tDataExporter.create(save_root=root,fps=50,features=features,
                    modality_config=modality,task='offline dex1 test',
                    script_config={'dex1_recording':metadata({})})
            exporter=create();cache=SampleCache()
            expected=[]
            for i in range(12):
                now=100.+i*.02
                if i<8:
                    p=packet(now,sequence=i)
                    p['sides']['left']['q']=.2+i*.13
                    p['sides']['right']['q']=2.-i*.07
                    cache.accept(p,now)
                # Force a stale final sample without compressing the frame sequence.
                f=cache.frame(now if i<11 else now+.2)
                frame={k:np.zeros(v['shape'],dtype='uint8' if v['dtype']=='video' else v['dtype'])
                       for k,v in features.items()}
                frame.update(f);expected.append(f)
                exporter.add_frame(frame)
            exporter.save_episode();exporter.stop_video_writers()
            p=root/'data/chunk-000/episode_000000.parquet'
            table=pq.read_table(p)
            self.assertEqual(table.num_rows,12)
            for key in expected[0]:
                np.testing.assert_allclose(np.asarray(table[key].to_pylist()).reshape(12,-1),
                                           np.stack([f[key] for f in expected]))
            self.assertFalse(expected[-1]['dex1.feedback_valid'].any())
            check_dataset(root,{})
            check_dataset(root,{},expected_features=features,expected_modalities=modality,fps=50)
            with self.assertRaises(ValueError):
                check_dataset(root,{},fps=30)
            info=json.loads((root/'meta/info.json').read_text())
            self.assertEqual(info['total_frames'],12)
            self.assertEqual(info['features']['observation.dex1_position']['shape'],[2])
            self.assertEqual(info['script_config']['dex1_recording']['position_units'],'motor_output_radians')
            stats=[json.loads(x) for x in (root/'meta/episodes_stats.jsonl').read_text().splitlines()]
            self.assertIn('observation.dex1_position',stats[0]['stats'])
            import av
            with av.open(str(root/'videos/chunk-000/observation.images.ego_view/episode_000000.mp4')) as video:
                self.assertEqual(sum(1 for _ in video.decode(video=0)),12)
            resumed=create()
            self.assertEqual(resumed.meta.total_frames,12)
            resumed.stop_video_writers()

    def test_collector_waits_then_preserves_stale_frames(self):
        from unittest.mock import Mock
        from gear_sonic.scripts.run_data_exporter import GrootDataCollector
        from gear_sonic.data.features_sonic_vla import get_g1_robot_model
        c=GrootDataCollector.__new__(GrootDataCollector)
        c._dex1=SampleCache();c._dex1_last_warning=-float('inf')
        c._dex1_counts={'frames':0,'feedback':[0,0],'command':[0,0],'trigger':[0,0]}
        c.latest_proprio_msg={'body_q':np.zeros(29),'left_hand_q':np.zeros(7),
            'right_hand_q':np.zeros(7),'last_action':np.zeros(29),
            'last_left_hand_action':np.zeros(7),'last_right_hand_action':np.zeros(7)}
        c.data_exporter=SimpleExporter()
        c.robot_model=get_g1_robot_model()
        c._add_cpp_state_features=Mock();c._add_sonic_pose_features=Mock(return_value=None)
        c._add_images_to_frame_data=Mock();c._log_latency_periodic=Mock()
        c._finalize_frame=Mock(return_value=True)
        self.assertFalse(c._add_data_frame_sonic(10.))
        self.assertEqual(len(c.data_exporter.frames),0)
        c._dex1.accept(packet(10.),10.)
        self.assertTrue(c._add_data_frame_sonic(10.01))
        self.assertTrue(c._add_data_frame_sonic(10.5))
        self.assertEqual(len(c.data_exporter.frames),2)
        self.assertTrue(c.data_exporter.frames[0]['dex1.feedback_valid'].all())
        self.assertFalse(c.data_exporter.frames[1]['dex1.feedback_valid'].any())
        self.assertEqual(c._dex1_counts['feedback'],[1,1])


class SimpleExporter:
    def __init__(self):
        self.episode_buffer={'size':0};self.frames=[]
    def add_frame(self,frame):
        self.frames.append(frame);self.episode_buffer['size']+=1


if __name__=='__main__':unittest.main()
