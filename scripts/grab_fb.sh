#!/bin/bash
# 현재 DSI 패널 화면을 그대로 캡처해 /tmp/fb.png 로 저장
BPP=$(cat /sys/class/graphics/fb0/bits_per_pixel)
SIZE=$(cat /sys/class/graphics/fb0/virtual_size); W=${SIZE%,*}; H=${SIZE#*,}
dd if=/dev/fb0 of=/tmp/fb.raw bs=$((W*H*BPP/8)) count=1 2>/dev/null
python3 - "$W" "$H" "$BPP" <<'PY'
import sys, zlib, struct
W,H,BPP = int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
raw = open('/tmp/fb.raw','rb').read(); px = BPP//8
rows=[]
for y in range(H):
    off=y*W*px; line=bytearray(b'\x00')
    for x in range(W):
        p=off+x*px
        line += bytes((raw[p+2],raw[p+1],raw[p])) if p+2 < len(raw) else b'\x00\x00\x00'
    rows.append(bytes(line))
d=zlib.compress(b''.join(rows),6)
def ck(t,b):
    c=t+b; return struct.pack('>I',len(b))+c+struct.pack('>I',zlib.crc32(c)&0xffffffff)
open('/tmp/fb.png','wb').write(b'\x89PNG\r\n\x1a\n'
  + ck(b'IHDR',struct.pack('>IIBBBBB',W,H,8,2,0,0,0)) + ck(b'IDAT',d) + ck(b'IEND',b''))
print(f"  /tmp/fb.png ({W}x{H})")
PY
