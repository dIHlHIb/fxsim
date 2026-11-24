"""Typed binary streams owned by one batch session."""
import math
import struct

from ._marray import Array, LIMIT, integer, need, scalar

FORMATS = {"double":"d","float64":"d","single":"f","float32":"f","int64":"q","uint64":"Q",
           "int32":"i","uint32":"I","int16":"h","uint16":"H","int8":"b","uint8":"B"}
ORDER = {"n":"=", "native":"=", "l":"<", "ieee-le":"<", "b":">", "ieee-be":">"}


def io_call(files, name, args, outputs):
    if name == "fopen":
        need(1 <= len(args) <= 3 and isinstance(args[0],str) and outputs in (1,2),name)
        mode=args[1] if len(args)>1 else "r"
        need(mode in ("r","rb","w","wb","a","ab"),"fopen mode")
        order=args[2] if len(args)>2 else "native"
        need(order in ORDER,name)
        try:
            stream=open(args[0],mode[0]+"b")
        except OSError as e:
            return (scalar(-1),str(e)) if outputs==2 else scalar(-1)
        key=max(files,default=2)+1
        files[key]=(stream,ORDER[order])
        return (scalar(key),"") if outputs==2 else scalar(key)
    need(args and integer(args[0]) in files,name)
    key=integer(args[0])
    stream,order=files[key]
    if name=="fclose":
        need(len(args)==1 and outputs==1,name)
        stream.close()
        del files[key]
        return scalar(0)
    if name=="fwrite":
        need(stream.writable() and 2<=len(args)<=5 and outputs==1,name)
        x=args[1]; precision=args[2] if len(args)>2 else "uint8"
        need(isinstance(x,Array) and isinstance(x.kind,str) and x.kind in (*FORMATS,"logical") and precision in FORMATS,name)
        need(len(args)<4 or integer(args[3])==0,name)
        if len(args)==5:
            need(args[4] in ORDER,name)
            order=ORDER[args[4]]
        code=FORMATS[precision]
        item=struct.Struct(order+code)
        result=bytearray(item.size*len(x.data))
        for i,v in enumerate(x.data):
            need(isinstance(v,(int,float)),name)
            if code not in ("f","d"):
                need(v==int(v),"fwrite integer conversion")
                v=int(v)
            try:item.pack_into(result,i*item.size,v)
            except (struct.error,OverflowError):need(False,"fwrite range")
        return scalar(stream.write(result)//item.size)
    need(stream.readable() and len(args)<=5 and outputs in (1,2),name)
    size=args[1] if len(args)>=2 and not isinstance(args[1],str) else scalar(math.inf)
    other=args[1:] if len(args)>=2 and isinstance(args[1],str) else args[2:]
    precision=other[0] if other else "uint8"
    need(len(other)<=3 and isinstance(precision,str) and (len(other)<2 or integer(other[1])==0),name)
    if len(other)==3:
        need(other[2] in ORDER,name)
        order=ORDER[other[2]]
    source,arrow,target=precision.partition("=>")
    if source.startswith("*"):
        need(not arrow,name)
        source=target=source[1:]
    elif not arrow:
        target="double"
    need(source in FORMATS and (target in (source,"double") or FORMATS.get(target)==FORMATS[source]),name)
    cls={"float32":"single","float64":"double"}.get(target,target)
    need(isinstance(size,Array) and size.kind=="double" and (len(size.data)==1 or (size.rows,size.cols)==(1,2)),name)
    def dimension(v):
        need(v==math.inf or math.isfinite(v) and v>=0 and int(v)==v,name)
        return int(v) if math.isfinite(v) else math.inf
    dims=[dimension(v) for v in size.data]
    matrix=len(dims)==2
    if matrix:
        need(0<dims[0]<=LIMIT,name)
    maximum=math.prod(dims)
    count=LIMIT+1 if maximum==math.inf else min(maximum,LIMIT+1)
    fmt=order+FORMATS[source]
    width=struct.calcsize(fmt)
    block=stream.read(count*width)
    actual,rem=divmod(len(block),width)
    need(not rem and actual<=LIMIT,name+" size or partial value")
    values=tuple(float(v) if cls=="double" else v for (v,) in struct.iter_unpack(fmt,block))
    if matrix:
        rows=dims[0]
        need(actual and actual%rows==0 or maximum==0,name+" incomplete matrix")
        result=Array(rows,actual//rows,values,cls)
    else:
        result=Array(actual,1,values,cls)
    return (result,scalar(actual)) if outputs==2 else result


def typecast(x, target):
    allowed=("double","int8","uint8","int16","uint16","int32","uint32","int64","uint64")
    need(isinstance(x,Array) and x.kind in allowed and target in allowed,"typecast class")
    need(x.rows==1 or x.cols==1,"typecast vector")
    if x.kind==target:return x
    source=struct.Struct("="+FORMATS[x.kind])
    dest=struct.Struct("="+FORMATS[target])
    nbytes=source.size*len(x.data)
    need(nbytes%dest.size==0 and nbytes//dest.size<=LIMIT,"typecast size")
    buf=bytearray(nbytes)
    for i,v in enumerate(x.data):
        need(isinstance(v,(int,float)),"typecast real")
        source.pack_into(buf,i*source.size,v)
    values=tuple(v for (v,) in dest.iter_unpack(buf))
    return Array(1,len(values),values,target) if x.rows==1 else Array(len(values),1,values,target)
