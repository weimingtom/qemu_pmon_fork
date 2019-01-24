#include "regdef.h"
.macro PCI_CONFIG_WRITE_DEV dev func mem int
li t0, 0xba000000|(\dev<<11)|(\func<<8); 
li t1, \mem; 
sw t1, 0x10(t0);
li t1, \int;
sh t1, 0x3c(t0); 
li t1, 7; 
sw t1, 0x4(t0);
.endm


.macro PCI_CONFIG_WRITE_BRG dev func memstart memend iostart ioend int pri sec sub
li t0, 0xba000000|(\dev<<11)|(\func<<8);
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
	
	
.set mips64
 mfc0    t2, $15, 1
 andi    t2, 0x3
 beqz t2, core0_start
 nop
.set mips3
########
#define FN_OFF 0x020
#define SP_OFF 0x028
#define GP_OFF 0x030
#define A1_OFF 0x038
	
 sll     t2, 8
 li     t0, 0xbfe11000
 
 or      t0, t0, t2
 
 
 bal waitforinit
 nop
	
waitforinit:   
	
 lw      v0, FN_OFF(t0)
 bnez    v0, 1f
 
	
 nop
 jr ra
 nop
1:
	
 dli      t1, 0xffffffff00000000 
 
 or       v0, t1
 dli      t1, 0x9800000000000000 
 
 ld      sp, SP_OFF(t0)
 or      sp, t1
 ld      gp, GP_OFF(t0)
 or      gp, t1
 ld      a1, A1_OFF(t0)
 jalr    v0  #byebye 
 nop
 
####
 1:  b   1b
 
 nop
core0_start:
 
PCI_CONFIG_WRITE_DEV 2,0,0x1fe00000,0
PCI_CONFIG_WRITE_DEV 3,0,0x40000000,20
PCI_CONFIG_WRITE_DEV 3,1,0x40010000,22
PCI_CONFIG_WRITE_DEV 4,0,0x40020000,57
PCI_CONFIG_WRITE_DEV 4,1,0x40060000,58
PCI_CONFIG_WRITE_DEV 4,2,0x40070000,59
PCI_CONFIG_WRITE_DEV 5,0,0x40080000,37
PCI_CONFIG_WRITE_DEV 6,0,0x400c0000,36
PCI_CONFIG_WRITE_DEV 7,0,0x400d0000,12
PCI_CONFIG_WRITE_DEV 8,0,0x400e0000,27
PCI_CONFIG_WRITE_BRG 9,0,0x40100000,0x4fffffff,0x18000000,0x180fffff,40,0,1,1
PCI_CONFIG_WRITE_BRG 10,0,0x50000000,0x53ffffff,0x18100000,0x181fffff,41,0,4,4
PCI_CONFIG_WRITE_BRG 11,0,0x54000000,0x57ffffff,0x18200000,0x182fffff,42,0,8,8
PCI_CONFIG_WRITE_BRG 12,0,0x58000000,0x5fffffff,0x18300000,0x183fffff,43,0,12,12
PCI_CONFIG_WRITE_BRG 13,0,0x60000000,0x77ffffff,0x18040000,0x184fffff,44,0,16,16
PCI_CONFIG_WRITE_BRG 14,0,0x78000000,0x7fffffff,0x18040000,0x184fffff,45,0,16,16
ld ra, (ra)
jr ra
nop
 
// mipsel-linux-gcc -mips64 -Wa,-al -EL -c boot.S |perl -ane 'if(/([\dA-F]{8})\s+(.*)/){$a=unpack(q(I),pack(q(H*),$1));printf qq(0x%08X, //%s\n),$a,$2;} elsif(/\d+\s+(.+)/){print qq(         //$1\n);}'
