"""Offline deployment dependency copy, verified by RECORD/file hashes; no training packages."""
import argparse,hashlib,importlib.metadata as md,json,shutil,sys
from pathlib import Path
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name
def main():
    p=argparse.ArgumentParser(__doc__);p.add_argument("--target",required=True);p.add_argument("--report",required=True)
    a=p.parse_args();target=Path(a.target).resolve()
    site=target/"lib/python3.11/site-packages"
    if not (target/"pyvenv.cfg").is_file():raise ValueError("Create isolated Python 3.11 venv first")
    roots=["torch","mujoco","numpy","scipy","pillow","pyyaml","onnxruntime","pyzmq","onnx","packaging"]
    blocked={"mjlab","orcs","mocke","vibe"}
    chosen={};todo=[(name,set()) for name in roots]
    while todo:
        name,extras=todo.pop();name=canonicalize_name(name)
        if name in blocked:raise ValueError("Forbidden dependency "+name)
        if name in chosen and extras<=chosen[name][1]:continue
        dist=md.distribution(name)
        extras=extras|chosen.get(name,(None,set()))[1];chosen[name]=(dist,extras)
        for text in dist.requires or []:
            req=Requirement(text)
            if req.marker and not any(req.marker.evaluate({"extra":extra}) for extra in extras|{""}):continue
            child=md.distribution(req.name)
            if req.specifier and not req.specifier.contains(child.version):raise ValueError("Unsatisfied source dependency "+text)
            todo.append((req.name,set(req.extras)))
    manifest={}
    for name,(dist,_) in sorted(chosen.items()):
        copied=0
        for entry in dist.files or []:
            source=Path(dist.locate_file(entry)).resolve()
            if not source.is_file() or str(entry).endswith((".pyc",".pth")):continue
            relative=Path(str(entry))
            # Entry point scripts are not used; launch modules through the venv interpreter.
            if ".." in relative.parts:continue
            dest=site/relative
            dest.parent.mkdir(parents=True,exist_ok=True)
            if source!=dest.resolve():shutil.copyfile(source,dest)
            if hashlib.sha256(source.read_bytes()).digest()!=hashlib.sha256(dest.read_bytes()).digest():
                raise ValueError("Copy verification failed "+str(entry))
            copied+=1
        manifest[name]={"version":dist.version,"files_verified":copied}
        print(name,dist.version,copied,flush=True)
    Path(a.report).write_text(json.dumps(manifest,indent=2))
if __name__=="__main__":main()
