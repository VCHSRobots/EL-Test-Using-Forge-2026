"""Minimal WPILOG reader: read(path) -> {signal name: [(seconds, value), ...]} for double, float,
int64, boolean and string entries (arrays, raw and structs are skipped). Forge's log_read is easier;
use this when you have a .wpilog file and want every sample."""
import struct,sys,json
def read(path):
    d=open(path,'rb').read()
    assert d[:6]==b'WPILOG'
    hl=struct.unpack('<I',d[8:12])[0]; p=12+hl
    ents={}; out={}
    while p<len(d):
        b=d[p]; p+=1
        el=(b&3)+1; sl=((b>>2)&3)+1; tl=((b>>4)&7)+1
        e=int.from_bytes(d[p:p+el],'little');p+=el
        s=int.from_bytes(d[p:p+sl],'little');p+=sl
        t=int.from_bytes(d[p:p+tl],'little');p+=tl
        pl=d[p:p+s];p+=s
        if e==0:
            if pl[0]==0:
                eid=struct.unpack('<I',pl[1:5])[0];q=5
                n=struct.unpack('<I',pl[q:q+4])[0];q+=4;name=pl[q:q+n].decode();q+=n
                n=struct.unpack('<I',pl[q:q+4])[0];q+=4;typ=pl[q:q+n].decode()
                ents[eid]=(name,typ);out.setdefault(name,[])
            continue
        if e not in ents: continue
        name,typ=ents[e]
        if typ=='double': v=struct.unpack('<d',pl)[0]
        elif typ=='float': v=struct.unpack('<f',pl)[0]
        elif typ=='int64': v=struct.unpack('<q',pl)[0]
        elif typ=='boolean': v=bool(pl[0])
        elif typ=='string': v=pl.decode(errors='replace')
        else: v=None
        out[name].append((t/1e6,v))
    return out
if __name__=='__main__':
    o=read(sys.argv[1])
    for k,v in o.items(): print(k,len(v), v[:1], v[-1:] if v else '')
