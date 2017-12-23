#include <stdlib.h>
#include <stdio.h>

extern void (*poweroff_pt)(void);
extern void (*reboot_pt)(void);
static void init_efi(struct efi *efi);
static void init_reset_system(struct efi_reset_system_t *reset);
static void init_smbios(struct smbios_tables *smbios);
static void init_loongson_params(struct loongson_params *lp);
static struct efi_memory_map_loongson * init_memory_map(void *);
static struct efi_cpuinfo_loongson *init_cpu_info(void *);
static struct system_loongson *init_system_loongson(void *);
static struct irq_source_routing_table *init_irq_source(void *);
static struct interface_info *init_interface_info(void *);
static struct board_devices *board_devices_info(void *);
static struct loongson_special_attribute *init_special_info(void *);

#ifdef RS780E
extern unsigned char vgarom[];
extern struct pci_device *vga_dev;
#endif

static int init_boot_param(struct boot_params *bp)
{
  
  init_efi(&(bp->efi));
  init_reset_system(&(bp->reset_system));


  return 0;
}

void init_efi(struct efi *efi)
{
    init_smbios(&(efi->smbios));
}

void init_reset_system(struct efi_reset_system_t *reset)
{
  reset->Shutdown = 0xffffffffbfc00000ULL;
  reset->ResetWarm = 0xffffffffbfc00000ULL;
}

void init_smbios(struct smbios_tables *smbios)
{
  
  smbios->vers = 0;
#ifdef RS780E
  if(vga_dev != NULL){
  smbios->vga_bios = vgarom;
 }
#else
  smbios->vga_bios = 0;
#endif
  init_loongson_params(&(smbios->lp)); 

}


void init_loongson_params(struct loongson_params *lp)
{
 void *p = boot_params_p;

  lp->memory_offset = (unsigned long long)init_memory_map(p) - (unsigned long long)lp;
  p += align(sizeof(struct efi_memory_map_loongson));
  lp->cpu_offset = (unsigned long long)init_cpu_info(p) - (unsigned long long)lp; 
  p += align(sizeof(struct efi_cpuinfo_loongson));
  lp->system_offset = (unsigned long long)init_system_loongson(p) - (unsigned long long)lp;
  p += align(sizeof(struct system_loongson));
  lp->irq_offset = (unsigned long long)init_irq_source(p) - (unsigned long long)lp; 
  p += align(sizeof(struct irq_source_routing_table));
  lp->interface_offset = (unsigned long long)init_interface_info(p) - (unsigned long long)lp;
  p += align(sizeof(struct interface_info));
  lp->boarddev_table_offset = (unsigned long long)board_devices_info(p) - (unsigned long long)lp;
  p+= align(sizeof(struct board_devices));
  lp->special_offset = (unsigned long long)init_special_info(p) - (unsigned long long)lp; 
  p+= align(sizeof(struct loongson_special_attribute));
  boot_params_p = p;
}


struct efi_memory_map_loongson * init_memory_map(void *g_map)
{
  struct efi_memory_map_loongson *emap = g_map;


  //map->mem_start_addr = 0x80000000;
#if (defined(LOONGSON_3BSERVER))
  emap->nr_map = 10; 
#else
  emap->nr_map = 6; 
#endif

  emap->mem_freq = 266000000; //300M
  //map->memsz_high = atoi(getenv("highmemsize"));
  //map->mem_size = atoi(getenv("memsize"));
  //map->memsz_reserved = 16;

#if 1
#ifdef LOONGSON_2H
  emap->map[0].node_id = 0;
  //strcpy(emap->map[0].mem_name, "node0_low");
  emap->map[0].mem_type = 1;
  emap->map[0].mem_start = 0x01000000;
  emap->map[0].mem_size = atoi(getenv("memsize")) - 112;
#else 
  emap->map[0].node_id = 0;
  //strcpy(emap->map[0].mem_name, "node0_low");
  emap->map[0].mem_type = 1;
  emap->map[0].mem_start = 0x01000000;
  emap->map[0].mem_size = atoi(getenv("memsize"));
#endif

  emap->map[1].node_id = 0;
  //strcpy(emap->map[1].mem_name, "node0_high");
  emap->map[1].mem_type = 2;
#if defined(LOONGSON_3A2H) || defined(LOONGSON_2H) || defined(LOONGSON_3C2H)
  emap->map[1].mem_start = 0x110000000;
#else
  emap->map[1].mem_start = 0x90000000;
#endif
  emap->map[1].mem_size = atoi(getenv("highmemsize"));
  
//support smbios
  emap->map[2].node_id = 0;
  emap->map[2].mem_type = 10;
  emap->map[2].mem_start = SMBIOS_PHYSICAL_ADDRESS;

#ifdef LOONGSON_2H 
  emap->map[3].node_id = 0;
  emap->map[3].mem_type = 3;
  emap->map[3].mem_start = 0x120000000;
  emap->map[3].mem_size = 256;
#else
  emap->map[3].node_id = 0;
  emap->map[3].mem_type = 3;
  emap->map[3].mem_start = SMBIOS_PHYSICAL_ADDRESS & 0x0fffffff;
  emap->map[3].mem_size = SMBIOS_SIZE_LIMIT >> 20;
#endif

#if (defined(MULTI_CHIP)) || (defined(LOONGSON_3BSINGLE))

  emap->map[5].mem_size = atoi(getenv("memorysize_high_n1"));
  if ( emap->map[5].mem_size != 0 )
  {
	emap->map[4].node_id = 1;
	//strcpy(emap->map[0].mem_name, "node0_low");
	emap->map[4].mem_type = 1;
	emap->map[4].mem_start = 0x01000000;
	emap->map[4].mem_size = atoi(getenv("memsize"));

	emap->map[5].node_id = 1;
	//strcpy(emap->map[1].mem_name, "node0_high");
	emap->map[5].mem_type = 2;
	emap->map[5].mem_start = 0x90000000;
  }

#endif

#if (defined(LOONGSON_3BSERVER))

  emap->map[5].mem_size = atoi(getenv("memorysize_high_n1"));
  if ( emap->map[5].mem_size != 0 )
  {
	emap->map[4].node_id = 1;
	//strcpy(emap->map[0].mem_name, "node0_low");
	emap->map[4].mem_type = 1;
	emap->map[4].mem_start = 0x01000000;
	emap->map[4].mem_size = atoi(getenv("memsize"));

	emap->map[5].node_id = 1;
	//strcpy(emap->map[1].mem_name, "node0_high");
	emap->map[5].mem_type = 2;
	emap->map[5].mem_start = 0x90000000;
  }

  emap->map[7].mem_size = atoi(getenv("memorysize_high_n2"));
  if ( emap->map[7].mem_size != 0 )
  {
	emap->map[6].node_id = 2;
	//strcpy(emap->map[0].mem_name, "node0_low");
	emap->map[6].mem_type = 1;
	emap->map[6].mem_start = 0x01000000;
	emap->map[6].mem_size = atoi(getenv("memsize"));

	emap->map[7].node_id = 2;
	//strcpy(emap->map[1].mem_name, "node0_high");
	emap->map[7].mem_type = 2;
	emap->map[7].mem_start = 0x90000000;
  }

  emap->map[9].mem_size = atoi(getenv("memorysize_high_n3"));
  if ( emap->map[9].mem_size != 0 )
  {
	emap->map[8].node_id = 3;
	//strcpy(emap->map[0].mem_name, "node0_low");
	emap->map[8].mem_type = 1;
	emap->map[8].mem_start = 0x01000000;
	emap->map[8].mem_size = atoi(getenv("memsize"));

	emap->map[9].node_id = 3;
	//strcpy(emap->map[1].mem_name, "node0_high");
	emap->map[9].mem_type = 2;
	emap->map[9].mem_start = 0x90000000;
  }

#endif
#endif


  return emap;
}

#ifdef LOONGSON_3BSINGLE
#ifdef LOONGSON_3B1500
  #define PRID_IMP_LOONGSON    0x6307
#else
  #define PRID_IMP_LOONGSON    0x6306
#endif
static enum loongson_cpu_type cputype = Loongson_3B;
#endif
#ifdef LOONGSON_3BSERVER
  #define PRID_IMP_LOONGSON    0x6306
static  enum loongson_cpu_type cputype = Loongson_3B;
#endif
#if  defined ( LOONGSON_3ASINGLE) || defined ( LOONGSON_3A2H) || defined (LOONGSON_2H)
  #define PRID_IMP_LOONGSON    0x6305
static  enum loongson_cpu_type cputype = Loongson_3A;
#endif
#ifdef LOONGSON_3ASERVER
  #define PRID_IMP_LOONGSON    0x6305
static  enum loongson_cpu_type cputype = Loongson_3A;
#endif

struct efi_cpuinfo_loongson *init_cpu_info(void *g_cpuinfo_loongson)
{
  struct efi_cpuinfo_loongson *c = g_cpuinfo_loongson;
  c->cputype  = cputype;

  c->cpu_clock_freq = atoi(getenv("ENV_cpuclock")?:"200000000");

  c->total_node = 1;
  c->nr_cpus = smp_cpus;
  c->cpu_startup_core_id = 0;

return c;
}
 
struct system_loongson *init_system_loongson(void *g_sysitem)
{
 struct system_loongson *s = g_sysitem;
  s->ccnuma_smp = 1;
#ifdef LOONGSON_3ASERVER
  s->ccnuma_smp = 1;
  s->sing_double_channel = 2;
#endif
#ifdef LOONGSON_3BSERVER
  s->ccnuma_smp = 1;
  s->sing_double_channel = 2; // means what?
#endif
#
#ifdef LOONGSON_3BSINGLE
  s->ccnuma_smp = 1;
  s->sing_double_channel = 2;
#endif
#if defined(LOONGSON_3ASINGLE) || defined(LOONGSON_2K)
  s->ccnuma_smp = 0;
  s->sing_double_channel = 1;
#endif
#ifdef LOONGSON_3A2H
  s->ccnuma_smp = 0;
  s->sing_double_channel = 1;
#endif
#ifdef LOONGSON_2H
  s->ccnuma_smp = 0;
  s->sing_double_channel = 1;
#endif

  return s;
}

enum loongson_irq_source_enum
{
  HT,I8259,UNKNOWN
};



struct irq_source_routing_table *init_irq_source(void *g_irq_source)
{

 struct irq_source_routing_table *irq_info = g_irq_source ;

	
	irq_info->PIC_type = HT;

 
#ifdef LOONGSON_3BSINGLE
	irq_info->ht_int_bit = 1<<16;
#else
	irq_info->ht_int_bit = 1<<24;
#endif
	irq_info->ht_enable = 0x0000d17b;

#ifdef LOONGSON_3BSINGLE
	irq_info->node_id = 1;
#else
	irq_info->node_id = 0;
#endif

	irq_info->pci_mem_start_addr = 0x40000000ul;
	irq_info->pci_mem_end_addr = 0x7ffffffful;

	return irq_info;
}

struct interface_info *init_interface_info(void *g_interface)
{
  
 struct interface_info *inter = g_interface;
 int flashsize = 0x80000;

  inter->vers = 0x0001;
  inter->size = flashsize/0x400;
  inter->flag = 1;

#ifdef LOONGSON_3A2H
  strcpy(inter->description,"Loongson-PMON-V3.3.1");
#else
  strcpy(inter->description,"Loongson-PMON-V3.3.0");
#endif
 
  return inter;
}

static struct board_devices *board_devices_info(void *g_board)
{
  
 struct board_devices *bd = g_board;
 
#ifdef LOONGSON_3ASINGLE
  strcpy(bd->name,"Loongson-3A-780E-1w-V1.03-demo");
#endif
#ifdef LOONGSON_2H
  strcpy(bd->name,"Loongson-2H-SOC-1w-V0.1-demo");
#endif
#ifdef LOONGSON_3A2H
  strcpy(bd->name,"Loongson-3A-2H-1w-V1.00-demo");
#endif
#ifdef LOONGSON_3BSINGLE
#ifdef LOONGSON_3B1500
        strcpy(bd->name, "Loongson-3B-780E-1w-V0.9-demo");
#else
        strcpy(bd->name, "Loongson-3B-780E-1w-V1.03-demo");
#endif
#endif
#ifdef LOONGSON_3BSERVER
  strcpy(bd->name,"Loongson-3B-780E-2w-V1.03-demo");
#endif
#
#ifdef LOONGSON_3ASERVER
#ifdef USE_BMC
  strcpy(bd->name,"Loongson-3A-780E-2wBMC-V1.02-demo");
#else
  strcpy(bd->name,"Loongson-3A-780E-2w-V1.02-demo");
#endif
#endif
#ifdef LEMOTE_3AITX
  strcpy(bd->name,"lemote-3a-itx-a1101");
#endif
#ifdef LEMOTE_3ANOTEBOOK
  strcpy(bd->name,"lemote-3a-notebook-a1004");
#endif
  bd->num_resources = 10;
 
  return bd;
}


struct loongson_special_attribute *init_special_info(void *g_special)
{
  
  struct loongson_special_attribute  *special = g_special;
  char update[11]="2013-01-01";
  int VRAM_SIZE=0x20000;
  
  strcpy(special->special_name,update);
  
#ifdef CONFIG_GFXUMA   
  special->resource[0].flags = 1;
  special->resource[0].start = 0;
  special->resource[0].end = VRAM_SIZE;
  strcpy(special->resource[0].name,"UMAMODULE");
#else
  special->resource[0].flags = 0;
  special->resource[0].start = 0;
  special->resource[0].end = VRAM_SIZE;
  strcpy(special->resource[0].name,"SPMODULE");
#endif

#ifndef LOONGSON_2H
  special->resource[0].flags |= DMA64_SUPPORT;
#endif
  return special;
}

