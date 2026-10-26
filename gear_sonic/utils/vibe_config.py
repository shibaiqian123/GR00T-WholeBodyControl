"""Generate a simulation observation contract from hashed exports and a native case."""
import hashlib
import json
import math
from pathlib import Path


def digest(path):
    with Path(path).open('rb') as stream:
        result = hashlib.sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
        return result.hexdigest()


def read(path):
    text = Path(path).read_text()
    if Path(path).suffix.lower() in ('.yaml', '.yml'):
        import yaml
        return yaml.safe_load(text)
    return json.loads(text)


def checked_files(root, files):
    root = Path(root).resolve()
    for name, expected in files.items():
        path = (root / name).resolve()
        if not path.is_relative_to(root):
            raise ValueError('File escapes artifact directory: ' + name)
        if digest(path) != expected:
            raise ValueError('Artifact hash mismatch: ' + str(path))


def generate(bundle, vision, case, vibe_root, model_root=None, standalone=False):
    bundle = Path(bundle).resolve()
    if bundle.is_dir():
        bundle /= 'bundle.json'
    vision, case, vibe_root = map(lambda p: Path(p).resolve(), (vision, case, vibe_root))
    b, c = read(bundle), read(case / 'case.json')
    if b['schema'] != 'vibe.split_sonic.v1' or b['status'] != 'passed':
        raise ValueError('Expected validated split SONIC bundle')
    required = {'motion_encoder.onnx', 'visual_decoder.onnx',
                'motion_encoder.manifest.json', 'visual_decoder.manifest.json'}
    if not required <= set(b['files']):
        raise ValueError('Incomplete bundle')
    checked_files(bundle.parent, b['files'])
    if not {'scene.mjb', 'initial.npz', 'reference.npz'} <= set(c['files']):
        raise ValueError('Incomplete native case')
    checked_files(case, c['files'])
    if c['bundle_sha256'] != digest(bundle):
        raise ValueError('Case belongs to another bundle')
    d = read(bundle.parent / 'visual_decoder.manifest.json')
    e = read(bundle.parent / 'motion_encoder.manifest.json')
    v = read(vision.with_suffix('.manifest.json'))
    if e['latent_contract'] != d['latent_contract'] or d['latent_contract'] != b['latent_contract']:
        raise ValueError('Latent contracts differ')
    if d['latent_contract']['shape'] != [1, 64]:
        raise ValueError('Expected 64D quantized latent')
    expected_ports = {'latent': [64], 'policy': [930], 'augmentation': [6],
                      'q_proprio': [64], 'q_cls': [192], 'kv_tokens__img_tokens': [21, 192]}
    if {p['name']: p['shape'] for p in d['inputs']} != expected_ports:
        raise ValueError('Unsupported decoder input contract')
    a = d['action']
    if a['type'] != 'joint_position' or len(a['joint_names']) != 29 or len(set(a['joint_names'])) != 29:
        raise ValueError('Expected 29 unique position-controlled joints')
    if a['joint_names'] != c['joint_names']:
        raise ValueError('Case joint order mismatch')
    for key in ('scale', 'default_joint_pos', 'stiffness', 'damping'):
        if len(a[key]) != 29 or not all(math.isfinite(x) for x in a[key]):
            raise ValueError('Invalid action field: ' + key)
    camera = c['camera']
    if (camera['width'], camera['height'], camera['fovy']) != (112, 63, 42.5):
        raise ValueError('Unexpected training camera')
    for key in ('pos', 'quat', 'geom_groups', 'shadows', 'textures'):
        if key not in camera:
            raise ValueError('Missing camera field: ' + key)
    if v['input']['layout'] != 'bgr_hwc_uint8' or (v['input']['width'], v['input']['height']) != (112, 63):
        raise ValueError('Unexpected Theia preprocessing')
    for port in d['inputs']:
        for term in port['terms']:
            src = term.get('source', {})
            if src.get('kind') == 'vision_encoder' and v['hf_id'].split('/')[-1] != src['model_name']:
                raise ValueError('Theia backbone mismatch')
    if c['physics_dt'] != .005 or c['control_dt'] != .02 or d['control'] != {
            'sim_timestep': .005, 'decimation': 4, 'step_dt': .02}:
        raise ValueError('Control clock mismatch')
    helper = (Path(__file__).parent / 'standalone/native.py') if standalone else vibe_root / 'src/vibe/sim2sim/runtime.py'
    result = {'schema': 'groot.vibe.recorded_sim.v1', 'simulation_only': True,
            'input_type': 'recorded_smpl', 'bundle': str(bundle), 'vision_encoder': str(vision),
            'case': str(case), 'vibe_root': str(vibe_root), 'action': a,
            'inputs': d['inputs'], 'tokenizer': e['inputs'], 'control': d['control'],
            'camera': camera, 'clip_actions': c['clip_actions'],
            'semantics': {'history': 'term-major, oldest-to-newest, 10 frames',
                'joint_position': 'q minus exported default_joint_pos',
                'last_action': 'previous clipped normalized decoder action',
                'angular_velocity': 'pelvis link frame', 'gravity': 'pelvis inverse rotation of world -Z',
                'augmentation': 'recorded reference anchor velocity in reference anchor frame',
                'vision': 'RGB converted to BGR uint8; normalization inside Theia ONNX'},
            'hashes': {'bundle': digest(bundle), 'vision_encoder': digest(vision),
                       'vision_manifest': digest(vision.with_suffix('.manifest.json')),
                       'case': digest(case / 'case.json'), 'native_helper': digest(helper)}}

    if model_root is not None:
        model_root = Path(model_root).resolve()
        aliases = {'motion_encoder': 'model_encoder.onnx', 'visual_decoder': 'model_decoder.onnx'}
        result['model_root'] = str(model_root)
        result['model_paths'] = {}
        for role, filename in aliases.items():
            path = model_root / filename
            if digest(path) != b['files'][role + '.onnx']:
                raise ValueError('Relocated model does not match bundle: ' + str(path))
            result['model_paths'][role] = str(path)
    if standalone:
        result['schema']='groot.vibe.standalone.v1'
        result.pop('vibe_root')
    return result


def load(path):
    cfg = read(path)
    independent=cfg['schema']=='groot.vibe.standalone.v1'
    fresh = generate(cfg['bundle'], cfg['vision_encoder'], cfg['case'], cfg.get('vibe_root','.'), cfg.get('model_root'),standalone=independent)
    if independent:
        derived=Path(cfg['vision_trt'])
        report=read(derived.with_suffix('.adaptation.json'))
        if report['status']!='passed' or report['source_sha256']!=digest(cfg['vision_encoder']) or report['derived_sha256']!=digest(derived):
            raise ValueError('Prepared TensorRT vision artifact mismatch')
        fresh['vision_trt']=str(derived)
        fresh['vision_trt_sha256']=digest(derived)
    if cfg != fresh:
        raise ValueError('Observation config is stale or edited; regenerate and review it')
    return cfg
