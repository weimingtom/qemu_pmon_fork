/*
 * QEMU loongson 1a ac97 emulation
 *
 * Copyright (c) 2013 qiaochong@loongson.cn
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "hw/hw.h"
#include "hw/audio/audio.h"
#include "audio/audio.h"
#include "hw/pci/pci.h"
#include "hw/sysbus.h"
#include "sysemu/dma.h"
#define DPRINTF(a...) //printf(a)
struct dma_desc{
	uint32_t ordered;
	uint32_t saddr;
	uint32_t daddr;
	uint32_t length;
	uint32_t step_length;
	uint32_t step_times;
	uint32_t cmd;
	/*used by logic only*/
	uint32_t left;
	uint32_t active;
	uint32_t nextaddr;
};


enum {
DMA_INT_MASK =  1,
DMA_INT = 2,
DMA_SINGLE_TRANS_OVER = 4,
DMA_TRANS_OVER = 8,
DMA_ORDER_EN = 1
};

typedef struct AC97State{
	MemoryRegion codec;
	QEMUSoundCard card;
	uint8_t mixer_data[256];
	QEMUTimer *timer;
	qemu_irq irq[2];
	struct dma_desc dma_write_desc;
	struct dma_desc dma_read_desc;
	uint32_t dma_o_mode;
	uint32_t dma_int_status;
	uint32_t dma_int_mask;
	union{
	AddressSpace *as;
	void *as_ptr;
	};


    uint32_t glob_cnt;
    uint32_t glob_sta;
    uint32_t cas;
    uint32_t last_samp;
    SWVoiceIn *voice_pi;
    SWVoiceOut *voice_po;
    SWVoiceIn *voice_mc;
    int invalid_freq[3];
	uint32_t voice_nch[3]; 
	uint32_t voice_fmt[3];
	uint32_t crac;
} AC97State;

#define next_ptr(x) ((x+1)&1)
#define next_wptr(x) ((x+1)&1)


#define DESC_TODEV_COMPELE	(0x1)           /**/
#define DESC_TODEV_EMPTY	(0x2)
#define DESC_TODEV_FULL		(0x4)
#define DESC_FRMDEV_COMPELE	(0x8)
#define DESC_FRMDEV_EMPTY	(0x10)
#define DESC_FRMDEV_FULL	(0x20)
#define DESC_EMPTY          (DESC_TODEV_EMPTY|DESC_FRMDEV_EMPTY)
#define DESC_FULL           (DESC_TODEV_FULL|DESC_FRMDEV_FULL)

typedef struct ac97_sysbus_state {
	SysBusDevice busdev;
	AC97State ac97;
} ac97_sysbus_state;

enum {
    AC97_Reset                     = 0x00,
    AC97_Master_Volume_Mute        = 0x02,
    AC97_Headphone_Volume_Mute     = 0x04,
    AC97_Master_Volume_Mono_Mute   = 0x06,
    AC97_Master_Tone_RL            = 0x08,
    AC97_PC_BEEP_Volume_Mute       = 0x0A,
    AC97_Phone_Volume_Mute         = 0x0C,
    AC97_Mic_Volume_Mute           = 0x0E,
    AC97_Line_In_Volume_Mute       = 0x10,
    AC97_CD_Volume_Mute            = 0x12,
    AC97_Video_Volume_Mute         = 0x14,
    AC97_Aux_Volume_Mute           = 0x16,
    AC97_PCM_Out_Volume_Mute       = 0x18,
    AC97_Record_Select             = 0x1A,
    AC97_Record_Gain_Mute          = 0x1C,
    AC97_Record_Gain_Mic_Mute      = 0x1E,
    AC97_General_Purpose           = 0x20,
    AC97_3D_Control                = 0x22,
    AC97_AC_97_RESERVED            = 0x24,
    AC97_Powerdown_Ctrl_Stat       = 0x26,
    AC97_Extended_Audio_ID         = 0x28,
    AC97_Extended_Audio_Ctrl_Stat  = 0x2A,
    AC97_PCM_Front_DAC_Rate        = 0x2C,
    AC97_PCM_Surround_DAC_Rate     = 0x2E,
    AC97_PCM_LFE_DAC_Rate          = 0x30,
    AC97_PCM_LR_ADC_Rate           = 0x32,
    AC97_MIC_ADC_Rate              = 0x34,
    AC97_6Ch_Vol_C_LFE_Mute        = 0x36,
    AC97_6Ch_Vol_L_R_Surround_Mute = 0x38,
    AC97_Vendor_Reserved           = 0x58,
    AC97_Vendor_ID1                = 0x7c,
    AC97_Vendor_ID2                = 0x7e
};

#define SOFT_VOLUME
#define SR_FIFOE 16             /* rwc */
#define SR_BCIS  8              /* rwc */
#define SR_LVBCI 4              /* rwc */
#define SR_CELV  2              /* ro */
#define SR_DCH   1              /* ro */
#define SR_VALID_MASK ((1 << 5) - 1)
#define SR_WCLEAR_MASK (SR_FIFOE | SR_BCIS | SR_LVBCI)
#define SR_RO_MASK (SR_DCH | SR_CELV)
#define SR_INT_MASK (SR_FIFOE | SR_BCIS | SR_LVBCI)

#define CR_IOCE  16             /* rw */
#define CR_FEIE  8              /* rw */
#define CR_LVBIE 4              /* rw */
#define CR_RR    2              /* rw */
#define CR_RPBM  1              /* rw */
#define CR_VALID_MASK ((1 << 5) - 1)
#define CR_DONT_CLEAR_MASK (CR_IOCE | CR_FEIE | CR_LVBIE)

#define GC_WR    4              /* rw */
#define GC_CR    2              /* rw */
#define GC_VALID_MASK ((1 << 6) - 1)

#define GS_MD3   (1<<17)        /* rw */
#define GS_AD3   (1<<16)        /* rw */
#define GS_RCS   (1<<15)        /* rwc */
#define GS_B3S12 (1<<14)        /* ro */
#define GS_B2S12 (1<<13)        /* ro */
#define GS_B1S12 (1<<12)        /* ro */
#define GS_S1R1  (1<<11)        /* rwc */
#define GS_S0R1  (1<<10)        /* rwc */
#define GS_S1CR  (1<<9)         /* ro */
#define GS_S0CR  (1<<8)         /* ro */
#define GS_MINT  (1<<7)         /* ro */
#define GS_POINT (1<<6)         /* ro */
#define GS_PIINT (1<<5)         /* ro */
#define GS_RSRVD ((1<<4)|(1<<3))
#define GS_MOINT (1<<2)         /* ro */
#define GS_MIINT (1<<1)         /* ro */
#define GS_GSCI  1              /* rwc */
#define GS_RO_MASK (GS_B3S12|                   \
                    GS_B2S12|                   \
                    GS_B1S12|                   \
                    GS_S1CR|                    \
                    GS_S0CR|                    \
                    GS_MINT|                    \
                    GS_POINT|                   \
                    GS_PIINT|                   \
                    GS_RSRVD|                   \
                    GS_MOINT|                   \
                    GS_MIINT)
#define GS_VALID_MASK ((1 << 18) - 1)
#define GS_WCLEAR_MASK (GS_RCS|GS_S1R1|GS_S0R1|GS_GSCI)

#define BD_IOC (1<<31)
#define BD_BUP (1<<30)

#define EACS_VRA 1
#define EACS_VRM 8

#define VOL_MASK 0x1f
#define MUTE_SHIFT 15

#define REC_MASK 7
enum {
    REC_MIC = 0,
    REC_CD,
    REC_VIDEO,
    REC_AUX,
    REC_LINE_IN,
    REC_STEREO_MIX,
    REC_MONO_MIX,
    REC_PHONE
};


#ifdef DEBUG_AC97
#define dolog(...) AUD_log ("ac97", __VA_ARGS__)
#else
#define dolog(...)
#endif


enum {
    PI_INDEX = 0,
    PO_INDEX,
    MC_INDEX,
    LAST_INDEX
};


enum {
    GLOB_CNT = 0x2c,
    GLOB_STA = 0x30,
    CAS      = 0x34
};

#define GET_BM(index) (((index) >> 4) & 3)

static void po_callback (void *opaque, int free);
static void pi_callback (void *opaque, int avail);
static void mc_callback (void *opaque, int avail);

static void mixer_store (AC97State *s, uint32_t i, uint16_t v)
{
    if (i + 2 > sizeof (s->mixer_data)) {
        dolog ("mixer_store: index %d out of bounds %zd\n",
               i, sizeof (s->mixer_data));
        return;
    }

    s->mixer_data[i + 0] = v & 0xff;
    s->mixer_data[i + 1] = v >> 8;
}

static uint16_t mixer_load (AC97State *s, uint32_t i)
{
    uint16_t val = 0xffff;

    if (i + 2 > sizeof (s->mixer_data)) {
        dolog ("mixer_store: index %d out of bounds %zd\n",
               i, sizeof (s->mixer_data));
    }
    else {
        val = s->mixer_data[i + 0] | (s->mixer_data[i + 1] << 8);
    }

    return val;
}

enum{
	FMT_CH_EN=1,
	FMT_VSR=2,
	FMT_SAMPLE_MASK=0xc,
	FMT_SAMPLE_16=0x8,
	FMT_SAMPLE_8=0,
	FMT_DMAEN=0x40
};

static void open_voice (AC97State *s, int index, int freq)
{
    struct audsettings as;

    as.freq = (s->voice_fmt[index]&FMT_VSR)?freq:48000;
    as.nchannels = s->voice_nch[index];
    as.fmt =((s->voice_fmt[index]&FMT_SAMPLE_MASK)==FMT_SAMPLE_16)?AUD_FMT_S16:((s->voice_fmt[index]&FMT_SAMPLE_MASK)==0)?AUD_FMT_U8:-1;
    as.endianness = 0;

    if (freq > 0 && (s->voice_fmt[index]&(FMT_DMAEN|FMT_CH_EN))==(FMT_DMAEN|FMT_CH_EN) && as.fmt != -1 ) {
        s->invalid_freq[index] = 0;
        switch (index) {
        case PI_INDEX:
            s->voice_pi = AUD_open_in (
                &s->card,
                s->voice_pi,
                "ac97.pi",
                s,
                pi_callback,
                &as
                );
            break;

        case PO_INDEX:
            s->voice_po = AUD_open_out (
                &s->card,
                s->voice_po,
                "ac97.po",
                s,
                po_callback,
                &as
                );
            break;

        case MC_INDEX:
            s->voice_mc = AUD_open_in (
                &s->card,
                s->voice_mc,
                "ac97.mc",
                s,
                mc_callback,
                &as
                );
            break;
        }
    }
    else {
        s->invalid_freq[index] = freq;
        switch (index) {
        case PI_INDEX:
            AUD_close_in (&s->card, s->voice_pi);
            s->voice_pi = NULL;
            break;

        case PO_INDEX:
            AUD_close_out (&s->card, s->voice_po);
            s->voice_po = NULL;
            break;

        case MC_INDEX:
            AUD_close_in (&s->card, s->voice_mc);
            s->voice_mc = NULL;
            break;
        }
    }
}

static void reset_voices (AC97State *s, uint8_t active[LAST_INDEX])
{
    uint16_t freq;

	s->voice_nch[PI_INDEX] = 1;
	s->voice_nch[PO_INDEX] = 2;
	s->voice_nch[MC_INDEX] = 1;

	s->voice_fmt[PI_INDEX] = FMT_CH_EN|FMT_DMAEN;
	s->voice_fmt[PO_INDEX] = FMT_CH_EN|FMT_DMAEN|FMT_SAMPLE_16;
	s->voice_fmt[MC_INDEX] = FMT_CH_EN|FMT_DMAEN;

    freq = mixer_load (s, AC97_PCM_LR_ADC_Rate);
    open_voice (s, PI_INDEX, freq);
    AUD_set_active_in (s->voice_pi, active[PI_INDEX]);

    freq = mixer_load (s, AC97_PCM_Front_DAC_Rate);
    open_voice (s, PO_INDEX, freq);
    AUD_set_active_out (s->voice_po, active[PO_INDEX]);

    freq = mixer_load (s, AC97_MIC_ADC_Rate);
    open_voice (s, MC_INDEX, freq);
    AUD_set_active_in (s->voice_mc, active[MC_INDEX]);
}

#ifdef USE_MIXER
static void set_volume (AC97State *s, int index,
                        audmixerctl_t mt, uint32_t val)
{
    int mute = (val >> MUTE_SHIFT) & 1;
    uint8_t rvol = VOL_MASK - (val & VOL_MASK);
    uint8_t lvol = VOL_MASK - ((val >> 8) & VOL_MASK);
    rvol = 255 * rvol / VOL_MASK;
    lvol = 255 * lvol / VOL_MASK;

#ifdef SOFT_VOLUME
    if (index == AC97_Master_Volume_Mute) {
        AUD_set_volume_out (s->voice_po, mute, lvol, rvol);
    }
    else {
        AUD_set_volume (mt, &mute, &lvol, &rvol);
    }
#else
    AUD_set_volume (mt, &mute, &lvol, &rvol);
#endif

    rvol = VOL_MASK - ((VOL_MASK * rvol) / 255);
    lvol = VOL_MASK - ((VOL_MASK * lvol) / 255);
    mixer_store (s, index, val);
}

static audrecsource_t ac97_to_aud_record_source (uint8_t i)
{
    switch (i) {
    case REC_MIC:
        return AUD_REC_MIC;

    case REC_CD:
        return AUD_REC_CD;

    case REC_VIDEO:
        return AUD_REC_VIDEO;

    case REC_AUX:
        return AUD_REC_AUX;

    case REC_LINE_IN:
        return AUD_REC_LINE_IN;

    case REC_PHONE:
        return AUD_REC_PHONE;

    default:
        dolog ("Unknown record source %d, using MIC\n", i);
        return AUD_REC_MIC;
    }
}

static uint8_t aud_to_ac97_record_source (audrecsource_t rs)
{
    switch (rs) {
    case AUD_REC_MIC:
        return REC_MIC;

    case AUD_REC_CD:
        return REC_CD;

    case AUD_REC_VIDEO:
        return REC_VIDEO;

    case AUD_REC_AUX:
        return REC_AUX;

    case AUD_REC_LINE_IN:
        return REC_LINE_IN;

    case AUD_REC_PHONE:
        return REC_PHONE;

    default:
        dolog ("Unknown audio recording source %d using MIC\n", rs);
        return REC_MIC;
    }
}

static void record_select (AC97State *s, uint32_t val)
{
    uint8_t rs = val & REC_MASK;
    uint8_t ls = (val >> 8) & REC_MASK;
    audrecsource_t ars = ac97_to_aud_record_source (rs);
    audrecsource_t als = ac97_to_aud_record_source (ls);
    AUD_set_record_source (&als, &ars);
    rs = aud_to_ac97_record_source (ars);
    ls = aud_to_ac97_record_source (als);
    mixer_store (s, AC97_Record_Select, rs | (ls << 8));
}
#endif

static void mixer_reset (AC97State *s)
{
    uint8_t active[LAST_INDEX];

    dolog ("mixer_reset\n");
    memset (s->mixer_data, 0, sizeof (s->mixer_data));
    memset (active, 0, sizeof (active));
    mixer_store (s, AC97_Reset                   , 0x0000); /* 6940 */
    mixer_store (s, AC97_Master_Volume_Mono_Mute , 0x8000);
    mixer_store (s, AC97_PC_BEEP_Volume_Mute     , 0x0000);

    mixer_store (s, AC97_Phone_Volume_Mute       , 0x8008);
    mixer_store (s, AC97_Mic_Volume_Mute         , 0x8008);
    mixer_store (s, AC97_CD_Volume_Mute          , 0x8808);
    mixer_store (s, AC97_Aux_Volume_Mute         , 0x8808);
    mixer_store (s, AC97_Record_Gain_Mic_Mute    , 0x8000);
    mixer_store (s, AC97_General_Purpose         , 0x0000);
    mixer_store (s, AC97_3D_Control              , 0x0000);
    mixer_store (s, AC97_Powerdown_Ctrl_Stat     , 0x000f);

    /*
     * Sigmatel 9700 (STAC9700)
     */
    mixer_store (s, AC97_Vendor_ID1              , 0x8384);
    mixer_store (s, AC97_Vendor_ID2              , 0x7600); /* 7608 */

    mixer_store (s, AC97_Extended_Audio_ID       , 0x0809);
    mixer_store (s, AC97_Extended_Audio_Ctrl_Stat, 0x0009);
    mixer_store (s, AC97_PCM_Front_DAC_Rate      , 0xbb80);
    mixer_store (s, AC97_PCM_Surround_DAC_Rate   , 0xbb80);
    mixer_store (s, AC97_PCM_LFE_DAC_Rate        , 0xbb80);
    mixer_store (s, AC97_PCM_LR_ADC_Rate         , 0xbb80);
    mixer_store (s, AC97_MIC_ADC_Rate            , 0xbb80);

#ifdef USE_MIXER
    record_select (s, 0);
    set_volume (s, AC97_Master_Volume_Mute, AUD_MIXER_VOLUME  , 0x8000);
    set_volume (s, AC97_PCM_Out_Volume_Mute, AUD_MIXER_PCM    , 0x8808);
    set_volume (s, AC97_Line_In_Volume_Mute, AUD_MIXER_LINE_IN, 0x8808);
#endif
    reset_voices (s, active);
}



static void nam_writew (void *opaque, uint32_t addr, uint32_t val)
{
    AC97State *s = opaque;
    uint32_t index = addr;
    s->cas = 0;
    switch (index) {
    case AC97_Reset:
        mixer_reset (s);
        break;
    case AC97_Powerdown_Ctrl_Stat:
        val &= ~0xf;
        val |= mixer_load (s, index) & 0xf;
        mixer_store (s, index, val);
        break;
#ifdef USE_MIXER
    case AC97_Master_Volume_Mute:
        set_volume (s, index, AUD_MIXER_VOLUME, val);
        break;
    case AC97_PCM_Out_Volume_Mute:
        set_volume (s, index, AUD_MIXER_PCM, val);
        break;
    case AC97_Line_In_Volume_Mute:
        set_volume (s, index, AUD_MIXER_LINE_IN, val);
        break;
    case AC97_Record_Select:
        record_select (s, val);
        break;
#endif
    case AC97_Vendor_ID1:
    case AC97_Vendor_ID2:
        dolog ("Attempt to write vendor ID to %#x\n", val);
        break;
    case AC97_Extended_Audio_ID:
        dolog ("Attempt to write extended audio ID to %#x\n", val);
        break;
    case AC97_Extended_Audio_Ctrl_Stat:
        if (!(val & EACS_VRA)) {
            mixer_store (s, AC97_PCM_Front_DAC_Rate, 0xbb80);
            mixer_store (s, AC97_PCM_LR_ADC_Rate,    0xbb80);
            open_voice (s, PI_INDEX, 48000);
            open_voice (s, PO_INDEX, 48000);
        }
        if (!(val & EACS_VRM)) {
            mixer_store (s, AC97_MIC_ADC_Rate, 0xbb80);
            open_voice (s, MC_INDEX, 48000);
        }
        dolog ("Setting extended audio control to %#x\n", val);
        mixer_store (s, AC97_Extended_Audio_Ctrl_Stat, val);
        break;
    case AC97_PCM_Front_DAC_Rate:
        if (mixer_load (s, AC97_Extended_Audio_Ctrl_Stat) & EACS_VRA) {
            mixer_store (s, index, val);
            dolog ("Set front DAC rate to %d\n", val);
            open_voice (s, PO_INDEX, val);
        }
        else {
            dolog ("Attempt to set front DAC rate to %d, "
                   "but VRA is not set\n",
                   val);
        }
        break;
    case AC97_MIC_ADC_Rate:
        if (mixer_load (s, AC97_Extended_Audio_Ctrl_Stat) & EACS_VRM) {
            mixer_store (s, index, val);
            dolog ("Set MIC ADC rate to %d\n", val);
            open_voice (s, MC_INDEX, val);
        }
        else {
            dolog ("Attempt to set MIC ADC rate to %d, "
                   "but VRM is not set\n",
                   val);
        }
        break;
    case AC97_PCM_LR_ADC_Rate:
        if (mixer_load (s, AC97_Extended_Audio_Ctrl_Stat) & EACS_VRA) {
            mixer_store (s, index, val);
            dolog ("Set front LR ADC rate to %d\n", val);
            open_voice (s, PI_INDEX, val);
        }
        else {
            dolog ("Attempt to set LR ADC rate to %d, but VRA is not set\n",
                    val);
        }
        break;
    default:
        dolog ("U nam writew %#x <- %#x\n", addr, val);
        mixer_store (s, index, val);
        break;
    }
}

static void ac97_check_irq(AC97State *s)
{
	/* edge interrupt */
	if(s->dma_int_status & 1)
	{
		qemu_irq_raise(s->irq[0]);
		qemu_irq_lower(s->irq[0]);
	}
	
	if(s->dma_int_status & 2)
	{
		qemu_irq_raise(s->irq[1]);
		qemu_irq_lower(s->irq[1]);
	}
	
	s->dma_int_status = 0;
}


static int dma_next(AC97State *s,struct dma_desc *desc,int irq)
{
	if(!desc->step_times)
	{
		if(desc->cmd & DMA_INT_MASK)
		{
			desc->cmd |= DMA_INT;
			s->dma_int_status = irq;
			ac97_check_irq(s);
		}

		if(desc->ordered & DMA_ORDER_EN)
		{
			dma_memory_read(s->as, desc->ordered & ~DMA_ORDER_EN ,(uint8_t *)desc,4*7);
			desc->left = desc->length * 4;
			desc->step_times--;
		}
		else if(desc->nextaddr)
		{
			dma_memory_read(s->as, desc->nextaddr, (uint8_t *)desc,4*7);
			desc->nextaddr = 0;
			desc->left = desc->length * 4;
		}
		else {
			desc->active = 0;
			desc->cmd |= DMA_TRANS_OVER;
		}
	}
	else
	{
		desc->step_times--;
		desc->saddr += desc->step_length * 4 ;
		desc->left = desc->length * 4;
	}

return desc->active;
}

static int write_audio (AC97State *s, int index,
                        int max, int *stop)
{
    uint8_t tmpbuf[4096];
    uint32_t addr;
    uint32_t temp,size;
    uint32_t written = 0;
    int to_copy = 0;


    do{
    if(!max)break;
	
    if(!s->dma_read_desc.active) break;

    if(!s->dma_read_desc.left && !dma_next(s,&s->dma_read_desc,1))
	break;

    addr = s->dma_read_desc.saddr;
    size = s->dma_read_desc.left;

    temp = audio_MIN (size, max);

    while (temp) {
        int copied;
        to_copy = audio_MIN (temp, sizeof (tmpbuf));
        dma_memory_read (s->as, addr, tmpbuf, to_copy);
        copied = AUD_write (s->voice_po, tmpbuf, to_copy);
        dolog ("write_audio max=%x to_copy=%x copied=%x\n",
               max, to_copy, copied);
        if (!copied) {
            *stop = 1;
            break;
        }
		temp -= copied;
		addr += copied;
		size -= copied;
		written += copied;
		max -= copied;
    }

	s->dma_read_desc.left = size;
	s->dma_read_desc.saddr = addr;

	
    } while(!*stop);

    if(!written) *stop=1;


    return written;
}


static int read_audio (AC97State *s, int index,
                       int max, int *stop)
{
    uint8_t tmpbuf[4096];
    uint32_t addr;
    uint32_t temp,size;
    uint32_t nread = 0;
    int to_copy = 0;
    SWVoiceIn *voice = index == MC_INDEX ? s->voice_mc : s->voice_pi;

    do{

    if(!max)break;

    if(!s->dma_write_desc.active) break;

    if(!s->dma_write_desc.left && !dma_next(s,&s->dma_write_desc,2))
     break;

    addr = s->dma_write_desc.saddr;
    size = s->dma_write_desc.left;

    temp = audio_MIN (size, max);


    while (temp) {
        int acquired;
        to_copy = audio_MIN (temp, sizeof (tmpbuf));
        acquired = AUD_read (voice, tmpbuf, to_copy);
        if (!acquired) {
            *stop = 1;
            break;
        }
        dma_memory_write (s->as, addr, tmpbuf, acquired);
        temp -= acquired;
        addr += acquired;
        nread += acquired;
		size -= acquired;
		max -= acquired;
    }

	s->dma_write_desc.left = size;
	s->dma_write_desc.saddr = addr;

    }while(!*stop);

    if (!nread) *stop = 1;
    return nread;
}

static void transfer_audio (AC97State *s, int index, int elapsed)
{
    int written = 0, stop = 0;

    if (s->invalid_freq[index]) {
        AUD_log ("ac97", "attempt to use voice %d with invalid frequency %d\n",
                 index, s->invalid_freq[index]);
        return;
    }


    while ((elapsed >> 1) && !stop) {
        int temp;


        switch (index) {
        case PO_INDEX:
            temp = write_audio (s, index, elapsed, &stop);
            written += temp;
            elapsed -= temp;
            break;

        case PI_INDEX:
        case MC_INDEX:
            temp = read_audio (s, index, elapsed, &stop);
            elapsed -= temp;
            break;
        }

    }
}

static void pi_callback (void *opaque, int avail)
{
    transfer_audio (opaque, PI_INDEX, avail);
}

static void mc_callback (void *opaque, int avail)
{
    transfer_audio (opaque, MC_INDEX, avail);
}

static void po_callback (void *opaque, int free)
{
    transfer_audio (opaque, PO_INDEX, free);
}






#define AC97_REG(x) x
#define DMA_REG(x) x

#define CSR			AC97_REG(0x00)
#define OCCR0		AC97_REG(0x04)
#define OCCR1		AC97_REG(0x08)
#define OCCR2		AC97_REG(0x0C)
#define ICCR		AC97_REG(0x10)
#define CDC_ID      AC97_REG(0x14)
#define CRAC		AC97_REG(0x18)

#define CRAR_READ  0x80000000
#define CRAR_WRITE 0x00000000
                                       /*Reserved 0x1C Reserved*/
#define OC0_DATA    AC97_REG(0x20)     /*20 bits WO Output Channel0 Tx buffer*/
#define OC1_DATA    AC97_REG(0x24)     /*20 bits WO Output Channel1 Tx buffer*/
#define OC2_DATA    AC97_REG(0x28)     /*20 bits WO Output Channel2 Tx buffer*/
#define OC3_DATA    AC97_REG(0x2C)     /*20 bits WO Output Channel3 Tx buffer*/
#define OC4_DATA    AC97_REG(0x30)     /*20 bits WO Output Channel4 Tx buffer*/
#define OC5_DATA    AC97_REG(0x34)     /*20 bits WO Output Channel5 Tx buffer*/
#define OC6_DATA    AC97_REG(0x38)     /*20 bits WO Output Channel6 Tx buffer*/
#define OC7_DATA    AC97_REG(0x3C)     /*20 bits WO Output Channel7 Tx buffer*/
#define OC8_DATA    AC97_REG(0x40)     /*20 bits WO Output Channel8 Tx buffer*/
#define IC0_DATA    AC97_REG(0x44)     /*20 bits RO Input Channel0 Rx buffer*/
#define IC1_DATA    AC97_REG(0x48)     /*20 bits RO Input Channel1 Rx buffer*/
#define IC2_DATA    AC97_REG(0x4C)     /*20 bits RO Input Channel2 Rx buffer*/
                                       /*Reserved AC97_REG(0x50 Reserved*/
#define INTRAW      AC97_REG(0x54)     /*32 bits RO Interrupt RAW status*/
#define INTM        AC97_REG(0x58)     /*32 bits R/W Interrupt Mask*/
#define INTS        AC97_REG(0x5C)     /*32 bits RO Interrupt Masked Status*/
#define CLR_INT     AC97_REG(0x60)     /*1 bit RO Clear Combined and Individual Interrupt*/
#define CLR_OC_INT  AC97_REG(0x64)     /*1 bit RO Clear Output Channel Reference Interrupt*/
#define CLR_IC_INT  AC97_REG(0x68)      /*1 bit RO Clear Input Channel Reference Interrupt*/
#define CLR_CDC_WR  AC97_REG(0x6C)      /*1 bit RO Clear Codec Write Done Interrupt*/
#define CLR_CDC_RD  AC97_REG(0x70)      /*1 bit RO Clear Codec Read Done Interrupt*/


/*control and status regs*/
#define DMA_STATUS  		 DMA_REG(0x2C)
#define DMA_INT_STATUS	 	 DMA_REG(0x2C)
#define DMA_INT_CLEAN		 DMA_REG(0x2C)
//#define DMA_CONTROL 		 DMA_REG(0x0)   /*NOT this reg*/
#define DMA_INT_MASK		 DMA_REG(0x28)  /*INT_MASK*/
/*channel control regs*/
#define DMA_ADDR_TODEV	 	 DMA_REG(0x0)   /*hardware define as RD_FIFO*/
#define DMA_SIZE_TODEV   	 DMA_REG(0x4)   /*RD_FIFO_SIZE*/
#define DMA_TODEV_RDPT       DMA_REG(0x8)   /*RD_FIFO_RD_PT*/               
#define DMA_TODEV_WRPT       DMA_REG(0xc)   /*RD_FIFO_WR_PT*/               
#define DMA_ADDR_FROMDEV 	 DMA_REG(0x10)  /*WR_FIFO*/
#define DMA_SIZE_FROMDEV 	 DMA_REG(0x14)  /*WR_FIFO_SIZE*/
#define DMA_FROMDEV_RDPT     DMA_REG(0x18)  /*RD_FIFO_RD_PT*/               
#define DMA_FROMDEV_WRPT     DMA_REG(0x1c)  /*RD_FIFO_WR_PT*/               
#define DMA_SET_TODEV        DMA_REG(0x20)  /*RD_MODE*/
#define DMA_SET_FRMDEV       DMA_REG(0x24)   /*WR_MODE*/




static uint64_t ac97_codec_readl(void *ptr, hwaddr addr, unsigned size)
{
	AC97State *s = ptr;
	uint32_t val;
	addr=addr&0x7f;
	switch(addr)
	{
	case CRAC:
		val=s->crac;
		break;
	case INTRAW:
		val=3;
		break;
	
	default:
		val=0;
		break;
	}
	DPRINTF("ac97_codec_readl:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
	return val;
}



static void ac97_codec_writel(void *ptr, hwaddr addr, uint64_t val, unsigned size)
{
	AC97State *s = ptr;
	uint32_t freq;
	addr=addr&0x7f;
	switch(addr)
	{
	case CRAC:
		s->crac = val;
		if(val&CRAR_READ)
		{
		 uint16_t data;
    	 data = mixer_load (s, (val>>16)&0x7fff);
		 s->crac = (s->crac&0xffff0000) | data;
		}
		else
		nam_writew(s,(val>>16)&0xffff,val&0xffff);
		break;

	case ICCR:
		s->voice_fmt[MC_INDEX] = val>>16;
    	freq = mixer_load (s, AC97_MIC_ADC_Rate);
    	open_voice (s, MC_INDEX, freq);
		break;

	case OCCR0:
		s->voice_fmt[PO_INDEX]= val;
		freq = mixer_load (s, AC97_PCM_Front_DAC_Rate);
		open_voice (s, PO_INDEX, freq);
		break;
	case INTRAW:
		break;
	
	default:
		break;
	}

	DPRINTF("ac97_codec_writel:  (addr 0x%08X), val 0x%08X\n", (unsigned) addr, val);
}

static const MemoryRegionOps ac97_codec_ops = {
    .read = ac97_codec_readl,
    .write = ac97_codec_writel,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static AC97State *ls1a_ac97;

static void ac97_new(AC97State *s)
{
	ls1a_ac97 = s;
	memset(s,0,sizeof(*s));

	memory_region_init_io(&s->codec, NULL, &ac97_codec_ops, (void *)s, "ac97", 0x80);

    	AUD_register_card ("sb2f_ac97", &s->card);
    	mixer_reset (s);
}

void ls1a_ac97_play_set_dmaaddr(uint32_t val);
void ls1a_ac97_rec_set_dmaaddr(uint32_t val);

void ls1a_ac97_play_set_dmaaddr(uint32_t val)
{
		AC97State *s = ls1a_ac97;

		uint32_t dmaaddr;
		dmaaddr = val & ~0xf;

		if(val & 0x8)
		{
		dma_memory_read(s->as, dmaaddr,(void *)&s->dma_read_desc,4*7);
		s->dma_read_desc.left = s->dma_read_desc.length * 4;
		s->dma_read_desc.step_times--;
		s->dma_read_desc.active = 1;
		s->dma_read_desc.nextaddr = 0;
		AUD_set_active_out (s->voice_po, 1);
		}

		if(val & 0x10)
		{
			s->dma_read_desc.active = 0;
			AUD_set_active_out (s->voice_po, 0);
		}

		if(val & 0x4)
		{
		dma_memory_write(s->as, dmaaddr,(void *)&s->dma_read_desc,4*7);
		}
}

void ls1a_ac97_rec_set_dmaaddr(uint32_t val)
{
		AC97State *s = ls1a_ac97;
		uint32_t dmaaddr;
		dmaaddr = val & ~0xf;

		if(val & 0x8)
		{
		dma_memory_read(s->as, dmaaddr,(void *)&s->dma_write_desc,4*7);
		s->dma_write_desc.left = s->dma_write_desc.length * 4;
		s->dma_write_desc.step_times--;
		s->dma_write_desc.active = 1;
		AUD_set_active_in (s->voice_pi, 1);
		}

		if(val & 0x10)
		{
			s->dma_write_desc.active = 0;
			AUD_set_active_in (s->voice_pi, 0);
		}
		
		if(val & 0x4)
		{
		dma_memory_write(s->as, dmaaddr,(void *)&s->dma_write_desc,4*7);
		}
}

#define TYPE_SYS_BUS_LS1AAC97 "ls1a_ac97"

#define SYS_BUS_LS1AAC97(obj) \
    OBJECT_CHECK(ac97_sysbus_state, (obj), TYPE_SYS_BUS_LS1AAC97)

static int ac97_sysbus_init(SysBusDevice *dev)
{
    ac97_sysbus_state *d = SYS_BUS_LS1AAC97(dev);

    ac97_new(&d->ac97);

    if(!d->ac97.as)
    d->ac97.as = &address_space_memory;

    sysbus_init_mmio(dev, &d->ac97.codec);
    sysbus_init_irq(dev, &d->ac97.irq[0]);
    sysbus_init_irq(dev, &d->ac97.irq[1]);

    return 0;
}

static Property ls1a_ac97_properties[] = {
    DEFINE_PROP_PTR("as", ac97_sysbus_state, ac97.as_ptr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ac97_sysbus_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = ac97_sysbus_init;
    dc->desc = "ls1a ac97";
    dc->props = ls1a_ac97_properties;
}

static const TypeInfo ac97_sysbus_info = {
    .name          = "ls1a_ac97",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(ac97_sysbus_state),
    .class_init    = ac97_sysbus_class_init,
};


static void ac97_sysbus_register_types(void)
{
    type_register_static(&ac97_sysbus_info);
}

type_init(ac97_sysbus_register_types)
