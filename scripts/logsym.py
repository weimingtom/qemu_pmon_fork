import re
import sys

f=open('nm.txt' if len(sys.argv)==1 else sys.argv[1])
l =f.readline()
p={}

# ffffffff8f90a15c t wait_init_done
while l:
 m = re.findall(r'([a-f0-9]{8,16})\s+\S\s+(\S+)',l)
 if m:
   p[int(m[0][0],16)] = m[0][1]
 l =f.readline()
f.close()

sk=p.keys()
sk.sort()

def hexsym(a):
  i0 = None
  for i in sk:
   if i > a:
     break
   i0 = i
  if (i0 != None and a - i0 < 0x1000):
    return ("<%s+0x%x> 0x%x" % (p[i0], a-i0, a))
  else:
    return None


f=open('qemu.log' if len(sys.argv)<3 else sys.argv[2])
l =f.readline()
while l:
 try:
   m = re.match(r'^(0x[a-f0-9]{16}):(.+)$',l)
   if m:
    s = hexsym(int(m.groups()[0], 16))
    if s:
       l = s + ":" +  m.groups()[1] + "\n"
  
   m = re.match(r'^(.+:\s+)(jal|bal|j|b)(\s+)(0x[a-f0-9]+)\s*$',l)
   if m:
    s = hexsym(int(m.groups()[3], 16))
    if s:
       l = m.groups()[0]+m.groups()[1] + m.groups()[2] + s +  "\n"
    continue
  
   m = re.match(r'^pc=(0x[a-f0-9]+)(\s.+)$',l)
   if m:
    s = hexsym(int(m.groups()[0], 16))
    if s:
       l = "pc=" + s + m.groups()[1] + "\n"
    continue
  
   m = re.match(r'^(.+\s)(EPC|ra)(\s+)(\S+)$',l)
   if m:
    s = hexsym(int(m.groups()[3], 16))
    if s:
       l = m.groups()[0] + m.groups()[1] + m.groups()[2] + s + "\n"
 finally:
  sys.stdout.write(l)
  l =f.readline()

