"""Derive a float pixel input for TRT parsers rejecting UINT8. Never edits source."""
import argparse,hashlib,json
from pathlib import Path
import numpy as np

def prepare(source,out):
    import onnx
    import onnxruntime as ort
    source=Path(source);out=Path(out)
    if out.exists():raise FileExistsError(out)
    graph=onnx.load(str(source))
    if len(graph.graph.input)!=1:raise ValueError('Expected one image input')
    port=graph.graph.input[0];name=port.name
    if port.type.tensor_type.elem_type!=onnx.TensorProto.UINT8:raise ValueError('Expected original UINT8 port')
    boundary={name};casts=0
    for n in graph.graph.node:
        if not boundary.intersection(n.input):continue
        if n.op_type=='Cast' and any(a.name=='to' and a.i==onnx.TensorProto.FLOAT for a in n.attribute):
            casts+=1;continue
        if n.op_type not in ('Slice','Transpose') or n.input[0] not in boundary:
            raise ValueError('Unsupported pixel operation before float cast: '+n.op_type)
        boundary.update(n.output)
    if casts!=1:raise ValueError('Expected one pixel float conversion')
    for value in graph.graph.value_info:
        if value.name in boundary:value.type.tensor_type.elem_type=onnx.TensorProto.FLOAT
    port.type.tensor_type.elem_type=onnx.TensorProto.FLOAT
    for prop in graph.metadata_props:
        if prop.key=='manifest':
            m=json.loads(prop.value);m['input']['layout']='bgr_hwc_float32_0_255';m['derived_for']='TensorRT pixel-input adapter';prop.value=json.dumps(m)
    onnx.checker.check_model(graph);out.parent.mkdir(parents=True,exist_ok=True);onnx.save(graph,str(out))
    opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
    old=ort.InferenceSession(str(source),sess_options=opts,providers=['CPUExecutionProvider'])
    new=ort.InferenceSession(str(out),sess_options=opts,providers=['CPUExecutionProvider'])
    shape=old.get_inputs()[0].shape;random=np.random.default_rng(0).integers(0,256,shape,dtype=np.uint8)
    errors=[]
    for pixels in (random,np.full(shape,127,np.uint8)):
        y0=old.run(None,{name:pixels});y1=new.run(None,{name:pixels.astype(np.float32)})
        errors.append(max(float(np.max(np.abs(a-b))) for a,b in zip(y0,y1,strict=True)))
    result={'status':'passed' if max(errors)==0 else 'blocked','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'derived_sha256':hashlib.sha256(out.read_bytes()).hexdigest(),'feature_max_abs':errors,'change':'input UINT8 to FLOAT only; same integer-valued 0..255 pixels, BGR, preprocessing and weights'}
    out.with_suffix('.adaptation.json').write_text(json.dumps(result,indent=2))
    if result['status']!='passed':raise ValueError(result)
    return result
if __name__=='__main__':
    p=argparse.ArgumentParser(__doc__);p.add_argument('--source',required=True);p.add_argument('--out',required=True);a=p.parse_args();print(json.dumps(prepare(a.source,a.out),indent=2))
