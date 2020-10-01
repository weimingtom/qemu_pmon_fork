///opt/gcc-4.4-gnu/bin/mipsel-linux-gcc -mips64 -Wa,-al,-am,-an  -EL -c ls1a500.S |perl -ane 'if(/([\dA-F]{8})\s+(.*)/){$a=unpack(q(I),pack(q(H*),$1));printf qq(0x%08X, //%s\n),$a,$2;} elsif(/\d+\s+(.+)/){print qq(         //$1\n);}' > loongson1a500_rom.h
#include "regdef.h"
.macro PCI_CONFIG_WRITE_DEV dev func mem int
li t0, 0xb6800000|(\dev<<11)|(\func<<8); 
li t1, \mem; 
sw t1, 0x10(t0);
li t1, \int;
sh t1, 0x3c(t0); 
li t1, 7; 
sw t1, 0x4(t0);
.endm


.macro PCI_CONFIG_WRITE_BRG dev func memstart memend iostart ioend int pri sec sub
li t0, 0xb6800000|(\dev<<11)|(\func<<8);
li t1, ((\memstart>>16)&0xfff0)|(\memend&0xfff00000);
sw t1, 0x20(t0);
sw t1, 0x24(t0);
li t1, ((\iostart>>16)&0xff)|(\ioend&0xff0000);
sw t1, 0x30(t0);
li t1, ((\iostart>>8)&0xf0)|(\ioend&0xf0)|0x101;
sw t1, 0x1c(t0);
li t1, \pri |(\sec << 8)|(\sub << 16);
sw t1, 0x18(t0);
li t1, 7;
sw t1, 0x4(t0);
.endm

	
.text
.set noreorder
.global _start;
_start:
 bal	1f
 nop
 .dword 0
 .dword 0
1:
 mfc0 t0, c0_status
 li      t1, 0x00e0      # {cu3,cu2,cu1,cu0}<={0110, status_fr<=1
 or      t0, t0, t1
 mtc0    t0, c0_status
 
 mfc0    t0, c0_status
 lui     t1, 0x40        #bev
 or      t0, t0, t1
 mtc0    t0, c0_status
 
 mtc0    zero, c0_cause
 
 mfc0 t0, c0_config
 
 ori     t0, t0, 7
 xori t0, t0, 4
 mtc0 t0, c0_config
	
	
PCI_CONFIG_WRITE_DEV 7,0,0x1f060000,30
PCI_CONFIG_WRITE_BRG 0,0,0x40300000,0x4fffffff,0x16400000,0x164fffff,40,0,1,1
PCI_CONFIG_WRITE_BRG 1,0,0x50000000,0x53ffffff,0x16500000,0x165fffff,41,0,4,4
ld ra, (ra)
jr ra
nop
 
