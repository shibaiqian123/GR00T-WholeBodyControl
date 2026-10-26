"""Strict, named, static single-environment binding to SONIC's C++ TensorRT engine."""
import ctypes as C
import hashlib
import shutil
from pathlib import Path
import numpy as np

class Session:
    def __init__(self, library, onnx, cache):
        self.lib=C.CDLL(str(Path(library).resolve()))
        signatures={'vibe_open':([C.c_char_p],C.c_void_p),'vibe_close':([C.c_void_p],None),
          'vibe_error':([],C.c_char_p),'vibe_names':([C.c_void_p,C.c_int],C.c_char_p),
          'vibe_shape':([C.c_void_p,C.c_char_p,C.POINTER(C.c_int64),C.c_int],C.c_int),
          'vibe_type':([C.c_void_p,C.c_char_p],C.c_int),
          'vibe_transfer':([C.c_void_p,C.c_char_p,C.c_void_p,C.c_size_t,C.c_int],C.c_int),
          'vibe_run':([C.c_void_p],C.c_int)}
        for n,(a,r) in signatures.items(): getattr(self.lib,n).argtypes=a;getattr(self.lib,n).restype=r
        onnx=Path(onnx); digest=hashlib.sha256(onnx.read_bytes()).hexdigest()
        dest=Path(cache)/digest/onnx.name;dest.parent.mkdir(parents=True,exist_ok=True)
        if not dest.exists(): shutil.copyfile(onnx,dest)
        if hashlib.sha256(dest.read_bytes()).hexdigest()!=digest:raise ValueError('Cache ONNX changed')
        self.handle=self.lib.vibe_open(str(dest).encode())
        if not self.handle:raise RuntimeError(self.lib.vibe_error().decode())
        self.inputs=self._ports(0);self.outputs=self._ports(1)
    def _ports(self,io):
        result={}
        types={0:np.float32,1:np.float16,2:np.int8,3:np.int32,4:np.bool_,5:np.uint8,6:np.int64}
        for name in self.lib.vibe_names(self.handle,io).decode().splitlines():
            dims=(C.c_int64*16)(); rank=self.lib.vibe_shape(self.handle,name.encode(),dims,16)
            shape=tuple(dims[:rank]);dtype=types[self.lib.vibe_type(self.handle,name.encode())]
            if rank<0 or any(d<1 for d in shape):raise ValueError(f'Unresolved shape {name}:{shape}')
            result[name]=(shape,np.dtype(dtype))
        return result
    def run(self,feeds):
        if set(feeds)!=set(self.inputs):raise ValueError(f'Wrong ports: expected {set(self.inputs)}, got {set(feeds)}')
        for n,(shape,dtype) in self.inputs.items():
            value=np.asarray(feeds[n])
            if value.shape!=shape or value.dtype!=dtype:raise ValueError(f'{n}: expected {shape}/{dtype}, got {value.shape}/{value.dtype}')
            if not np.isfinite(value).all():raise ValueError(f'Non-finite {n}')
            value=np.ascontiguousarray(value)
            self._check(self.lib.vibe_transfer(self.handle,n.encode(),value.ctypes.data,value.nbytes,0))
        self._check(self.lib.vibe_run(self.handle))
        outputs={}
        for n,(shape,dtype) in self.outputs.items():
            v=np.empty(shape,dtype=dtype)
            self._check(self.lib.vibe_transfer(self.handle,n.encode(),v.ctypes.data,v.nbytes,1))
            if not np.isfinite(v).all():raise FloatingPointError(f'Non-finite output {n}')
            outputs[n]=v
        return outputs
    def _check(self,r):
        if r:raise RuntimeError(self.lib.vibe_error().decode())
    def close(self):
        if self.handle:self.lib.vibe_close(self.handle);self.handle=None
