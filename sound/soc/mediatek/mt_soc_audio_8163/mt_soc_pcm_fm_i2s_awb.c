/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */



/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
#include "mt_soc_pcm_common.h"

/* information about */
static struct AFE_MEM_CONTROL_T *FM_I2S_AWB_Control_context;
static struct snd_dma_buffer *Awb_Capture_dma_buf;

static DEFINE_SPINLOCK(auddrv_AWBInCtl_lock);

/*
 *    function implementation
 */
static void StartAudioFMI2SAWBHardware(struct snd_pcm_substream *substream);
static void StopAudioFMI2SAWBHardware(struct snd_pcm_substream *substream);
static int mtk_fm_i2s_awb_probe(struct platform_device *pdev);
static int mtk_fm_i2s_awb_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_fm_i2s_awb_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_fm_i2s_awb_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_mgrrx_awb_hardware = {

	.info = (SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_RESUME | SNDRV_PCM_INFO_MMAP_VALID),
	.formats = SND_SOC_STD_MT_FMTS,
	.rates = SOC_NORMAL_USE_RATE,
	.rate_min = SOC_NORMAL_USE_RATE_MIN,
	.rate_max = SOC_NORMAL_USE_RATE_MAX,
	.channels_min = SOC_NORMAL_USE_CHANNELS_MIN,
	.channels_max = SOC_NORMAL_USE_CHANNELS_MAX,
	.buffer_bytes_max = FM_I2S_MAX_BUFFER_SIZE,
	.period_bytes_max = FM_I2S_MAX_BUFFER_SIZE,
	.periods_min = FM_I2S_MIN_PERIOD_SIZE,
	.periods_max = FM_I2S_MAX_PERIOD_SIZE,
	.fifo_size = 0,
};

static void StopAudioFMI2SAWBHardware(struct snd_pcm_substream *substream)
{
	pr_debug("StopAudioFMI2SAWBHardware\n");

	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, false);

	/* here to set interrupt */
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

	if (GetI2SDacEnable() == false)
		SetI2SDacEnable(false);

	/* here to turn off digital part */
	SetConnection(Soc_Aud_InterCon_DisConnect,
		Soc_Aud_InterConnectionInput_I00,
		Soc_Aud_InterConnectionOutput_O05);
	SetConnection(Soc_Aud_InterCon_DisConnect,
		Soc_Aud_InterConnectionInput_I01,
		Soc_Aud_InterConnectionOutput_O06);

	EnableAfe(false);
}

static void StartAudioFMI2SAWBHardware(struct snd_pcm_substream *substream)
{
	struct AudioDigtalI2S m2ndI2SInAttribute;

	pr_debug("StartAudioFMI2SAWBHardware\n");

	/* here to set interrupt */
	SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE,
		substream->runtime->period_size >> 1);
	SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE,
		substream->runtime->rate);
	SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);

	SetSampleRate(Soc_Aud_Digital_Block_MEM_AWB, substream->runtime->rate);
	SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, true);

	/* here to turn off digital part */
	SetConnection(Soc_Aud_InterCon_Connection,
		Soc_Aud_InterConnectionInput_I00,
		Soc_Aud_InterConnectionOutput_O05);
	SetConnection(Soc_Aud_InterCon_Connection,
		Soc_Aud_InterConnectionInput_I01,
		Soc_Aud_InterConnectionOutput_O06);


	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2) == false) {
		/* set merge interface */
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, true);

		/* Config 2nd I2S IN */
		memset((void *)&m2ndI2SInAttribute,
			0,
			sizeof(m2ndI2SInAttribute));

		m2ndI2SInAttribute.mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
		/* I2S_IN_FROM_CONNSYS */
		m2ndI2SInAttribute.mI2S_IN_PAD_SEL = false;
		m2ndI2SInAttribute.mI2S_SLAVE = Soc_Aud_I2S_SRC_SLAVE_MODE;
		m2ndI2SInAttribute.mI2S_SAMPLERATE = 32000;
		m2ndI2SInAttribute.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
		m2ndI2SInAttribute.mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
		m2ndI2SInAttribute.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
		Set2ndI2SIn(&m2ndI2SInAttribute);
		/* Covert from 32000 Hz to 44100 Hz */
		SetI2SASRCConfig(true, 44100);
		SetI2SASRCEnable(true);

		Set2ndI2SInEnable(true);
	} else
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, true);

	EnableAfe(true);
}

static int mtk_fm_i2s_awb_pcm_prepare(struct snd_pcm_substream *substream)
{
	pr_debug("%s, substream->rate = %d, substream->channels = %d\n",
		__func__,
		substream->runtime->rate,
		substream->runtime->channels);
	return 0;
}

static int mtk_fm_i2s_awb_alsa_stop(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_fm_i2s_awb_alsa_stop\n");
	StopAudioFMI2SAWBHardware(substream);
	RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB, substream);

	SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, false);
	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2) == false) {
		SetI2SASRCEnable(false);
		SetI2SASRCConfig(false, 0);	/* Setting to bypass ASRC */
		Set2ndI2SInEnable(false);
	}

	return 0;
}

static kal_int32 Previous_Hw_cur;
static snd_pcm_uframes_t mtk_awb_pcm_pointer(
	struct snd_pcm_substream *substream)
{
	kal_int32 HW_memory_index = 0;
	kal_int32 HW_Cur_ReadIdx = 0;
	kal_uint32 Frameidx = 0;
	struct AFE_BLOCK_T *Awb_Block = &(FM_I2S_AWB_Control_context->rBlock);

	pr_debug("mtk_awb_pcm_pointer, Awb_Block->u4WriteIdx = 0x%x\n",
		Awb_Block->u4WriteIdx);

	if (GetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB) == true) {
		/* get total bytes to copysinewavetohdmi */
		Frameidx = audio_bytes_to_frame(substream,
			Awb_Block->u4WriteIdx);
		return Frameidx;

		HW_Cur_ReadIdx = Align64ByteSize(Afe_Get_Reg(AFE_AWB_CUR));
		if (HW_Cur_ReadIdx == 0) {
			pr_debug("awb_pointer, HW_Cur_ReadIdx == 0\n");
			HW_Cur_ReadIdx = Awb_Block->pucPhysBufAddr;
		}
		HW_memory_index = (HW_Cur_ReadIdx - Awb_Block->pucPhysBufAddr);
		Previous_Hw_cur = HW_memory_index;
		pr_debug("awb_pointer = 0x%x, HW_memory_index = 0x%x\n",
			       HW_Cur_ReadIdx, HW_memory_index);
		return audio_bytes_to_frame(substream, Previous_Hw_cur);
	}
	return 0;
}


static void SetAWBBuffer(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params)
{
	struct AFE_BLOCK_T *pblock = &FM_I2S_AWB_Control_context->rBlock;
	struct snd_pcm_runtime *runtime = substream->runtime;

	pr_debug("SetAWBBuffer\n");
	pblock->pucPhysBufAddr = runtime->dma_addr;
	pblock->pucVirtBufAddr = runtime->dma_area;
	pblock->u4BufferSize = runtime->dma_bytes;
	pblock->u4SampleNumMask = 0x001f;	/* 32 byte align */
	pblock->u4WriteIdx = 0;
	pblock->u4DMAReadIdx = 0;
	pblock->u4DataRemained = 0;
	pblock->u4fsyncflag = false;
	pblock->uResetFlag = true;
	pr_debug("dma_bytes = %d, dma_area = %p, dma_addr = 0x%x\n",
		pblock->u4BufferSize,
		pblock->pucVirtBufAddr,
		pblock->pucPhysBufAddr);
	/* set sram address top hardware */
	Afe_Set_Reg(AFE_AWB_BASE, pblock->pucPhysBufAddr, 0xffffffff);
	Afe_Set_Reg(AFE_AWB_END,
		pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1),
		0xffffffff);

}

static int mtk_mgrrx_awb_pcm_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
	int ret = 0;

	pr_debug("mtk_mgrrx_awb_pcm_hw_params\n");

	dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
	dma_buf->dev.dev = substream->pcm->card->dev;
	dma_buf->private_data = NULL;

	if (Awb_Capture_dma_buf->area) {
		pr_debug("Awb_Capture_dma_buf->area\n");
		runtime->dma_bytes = params_buffer_bytes(hw_params);
		runtime->dma_area = Awb_Capture_dma_buf->area;
		runtime->dma_addr = Awb_Capture_dma_buf->addr;
	} else {
		pr_debug("snd_pcm_lib_malloc_pages\n");
		ret = snd_pcm_lib_malloc_pages(substream,
			params_buffer_bytes(hw_params));
	}
	pr_debug("%s dma_bytes = %zu, dma_area = %p, dma_addr = 0x%x\n",
		__func__, runtime->dma_bytes, runtime->dma_area,
		(uint32) runtime->dma_addr);

	pr_debug("runtime->hw.buffer_bytes_max = %zu\n",
		runtime->hw.buffer_bytes_max);
	SetAWBBuffer(substream, hw_params);

	pr_debug("dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
	       substream->runtime->dma_bytes, substream->runtime->dma_area,
	       (long)substream->runtime->dma_addr);
	return ret;
}

static int mtk_fm_i2s_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_fm_i2s_capture_pcm_hw_free\n");

	if (Awb_Capture_dma_buf->area)
		return 0;
	else
		return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_hw_constraint_list fm_i2s_awb_constraints_sample_rates = {
	.count = ARRAY_SIZE(soc_normal_supported_sample_rates),
	.list = soc_normal_supported_sample_rates,
};

static int mtk_fm_i2s_awb_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret = 0;

	pr_debug("mtk_fm_i2s_awb_pcm_open\n");
	FM_I2S_AWB_Control_context =
		Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_AWB);
	runtime->hw = mtk_mgrrx_awb_hardware;
	memcpy((void *)(&(runtime->hw)), (void *)&mtk_mgrrx_awb_hardware,
	       sizeof(struct snd_pcm_hardware));

	ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
		&fm_i2s_awb_constraints_sample_rates);
	ret = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);

	if (ret < 0)
		pr_debug("snd_pcm_hw_constraint_integer failed\n");

	pr_debug("runtime->rate = %d, channels = %d\n",
		runtime->rate, runtime->channels);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		pr_debug("SNDRV_PCM_STREAM_CAPTURE\n");
	else
		return -1;

	/* here open audio clocks */
	AudDrv_ANA_Clk_On();
	AudDrv_Clk_On();
	AudDrv_I2S_Clk_On();
	AudDrv_Emi_Clk_On();

	if (ret < 0) {
		pr_err("mtk_fm_i2s_awb_pcm_close\n");
		mtk_fm_i2s_awb_pcm_close(substream);
		return ret;
	}

	pr_debug("mtk_fm_i2s_awb_pcm_open return\n");
	return 0;
}

static int mtk_fm_i2s_awb_pcm_close(struct snd_pcm_substream *substream)
{
	AudDrv_Emi_Clk_Off();
	AudDrv_I2S_Clk_Off();
	AudDrv_Clk_Off();
	AudDrv_ANA_Clk_Off();
	return 0;
}

static int mtk_fm_i2s_awb_alsa_start(struct snd_pcm_substream *substream)
{
	pr_debug("mtk_fm_i2s_awb_alsa_start\n");
	SetMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB, substream);
	StartAudioFMI2SAWBHardware(substream);
	return 0;
}

static int mtk_capture_fm_i2s_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd)
{
	pr_debug("mtk_capture_fm_i2s_pcm_trigger cmd = %d\n", cmd);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		return mtk_fm_i2s_awb_alsa_start(substream);
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		return mtk_fm_i2s_awb_alsa_stop(substream);
	}
	return -EINVAL;
}

static bool CheckNullPointer(void *pointer)
{
	if (pointer == NULL) {
		pr_debug("CheckNullPointer pointer = NULL");
		return true;
	}
	return false;
}

static int mtk_fm_i2s_awb_pcm_copy(struct snd_pcm_substream *substream,
				   int channel, snd_pcm_uframes_t pos,
				   void __user *dst, snd_pcm_uframes_t count)
{
	struct AFE_MEM_CONTROL_T *pAWB_MEM_ConTrol = NULL;
	struct AFE_BLOCK_T *Awb_Block = NULL;
	char *Read_Data_Ptr = (char *)dst;
	ssize_t DMA_Read_Ptr = 0, read_size = 0, read_count = 0;
	unsigned long flags;

	/* get total bytes to copy */
	count = Align64ByteSize(audio_frame_to_bytes(substream, count));

	pr_debug("%s, pos = %lu, count = %lu\n", __func__, pos, count);

	/* check which memif nned to be write */
	pAWB_MEM_ConTrol = FM_I2S_AWB_Control_context;
	Awb_Block = &(pAWB_MEM_ConTrol->rBlock);

	if (pAWB_MEM_ConTrol == NULL) {
		pr_err("cannot find MEM control !!!!!!!\n");
		msleep(50);
		return 0;
	}

	if (Awb_Block->u4BufferSize <= 0) {
		msleep(50);
		return 0;
	}

	if (CheckNullPointer((void *)Awb_Block->pucVirtBufAddr)) {
		pr_err("CheckNullPointer pucVirtBufAddr = %p\n",
			Awb_Block->pucVirtBufAddr);
		return 0;
	}

	spin_lock_irqsave(&auddrv_AWBInCtl_lock, flags);
	if (Awb_Block->u4DataRemained > Awb_Block->u4BufferSize) {
		pr_debug("u4DataRemained=%x > u4BufferSize=%x\n",
			Awb_Block->u4DataRemained,
			Awb_Block->u4BufferSize);
		Awb_Block->u4DataRemained = 0;
		Awb_Block->u4DMAReadIdx = Awb_Block->u4WriteIdx;
	}

	if (count > Awb_Block->u4DataRemained)
		read_size = Awb_Block->u4DataRemained;
	else
		read_size = count;

	DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
	spin_unlock_irqrestore(&auddrv_AWBInCtl_lock, flags);

	if (DMA_Read_Ptr + read_size < Awb_Block->u4BufferSize) {
		if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx) {
			pr_debug
				("%s 1,size:%zu,remain:%x,RPtr:%zu,RIdx:%x\n",
				__func__,
				read_size,
				Awb_Block->u4DataRemained,
				DMA_Read_Ptr,
				Awb_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Awb_Block->pucVirtBufAddr + DMA_Read_Ptr),
				read_size)) {
			pr_err("%s Fail 1 copy to user, RDPtr:%p, BufAddr:%p,",
				__func__,
				Read_Data_Ptr,
				Awb_Block->pucVirtBufAddr);
			return 0;
		}

		read_count += read_size;
		spin_lock(&auddrv_AWBInCtl_lock);
		Awb_Block->u4DataRemained -= read_size;
		Awb_Block->u4DMAReadIdx += read_size;
		Awb_Block->u4DMAReadIdx %= Awb_Block->u4BufferSize;
		DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_AWBInCtl_lock);

		Read_Data_Ptr += read_size;
		count -= read_size;
	} else {
		uint32 size_1 = Awb_Block->u4BufferSize - DMA_Read_Ptr;
		uint32 size_2 = read_size - size_1;

		if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx) {
			pr_debug
				("%s 2,size1:%x,Remain:%x,RPtr:%zu,RIdx:%x\n",
				__func__,
				size_1,
				Awb_Block->u4DataRemained,
				DMA_Read_Ptr,
				Awb_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)Read_Data_Ptr,
			(Awb_Block->pucVirtBufAddr + DMA_Read_Ptr), size_1)) {
			pr_err("%s Fail 2 copy to user, RPtr:%p, BufAddr:%p,",
				__func__,
				Read_Data_Ptr,
				Awb_Block->pucVirtBufAddr);
			return 0;
		}

		read_count += size_1;
		spin_lock(&auddrv_AWBInCtl_lock);
		Awb_Block->u4DataRemained -= size_1;
		Awb_Block->u4DMAReadIdx += size_1;
		Awb_Block->u4DMAReadIdx %= Awb_Block->u4BufferSize;
		DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_AWBInCtl_lock);

		if (DMA_Read_Ptr != Awb_Block->u4DMAReadIdx) {
			pr_debug
				("%s 3,size2:%x,Remain:%x,RPtr:%zu,RIdx:%x\n",
				__func__,
				size_2,
				Awb_Block->u4DataRemained,
				DMA_Read_Ptr,
				Awb_Block->u4DMAReadIdx);
		}

		if (copy_to_user((void __user *)(Read_Data_Ptr + size_1),
			(Awb_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2)) {
			pr_err("%s Fail 3 copy to user, RPtr:%p, BufAddr:%p,",
				__func__,
				Read_Data_Ptr,
				Awb_Block->pucVirtBufAddr);
			return read_count << 2;
		}

		read_count += size_2;
		spin_lock(&auddrv_AWBInCtl_lock);
		Awb_Block->u4DataRemained -= size_2;
		Awb_Block->u4DMAReadIdx += size_2;
		DMA_Read_Ptr = Awb_Block->u4DMAReadIdx;
		spin_unlock(&auddrv_AWBInCtl_lock);

		count -= read_size;
		Read_Data_Ptr += read_size;

		pr_debug
			("%s finish3,size_2:%x,RIdx:%x,WIdx:%x,remained:%x\n",
			__func__,
			size_2,
			Awb_Block->u4DMAReadIdx,
			Awb_Block->u4WriteIdx,
			Awb_Block->u4DataRemained);
	}

	return read_count >> 2;
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
	int channel, snd_pcm_uframes_t pos, snd_pcm_uframes_t count)
{
	pr_debug("dummy_pcm_silence\n");
	/* do nothing */
	return 0;
}


static void *dummy_page[2];

static struct page *mtk_fm_i2s_capture_pcm_page(
	struct snd_pcm_substream *substream,
	unsigned long offset)
{
	pr_debug("dummy_pcm_page\n");
	return virt_to_page(dummy_page[substream->stream]);/* the same page */
}


static struct snd_pcm_ops mtk_fm_i2s_awb_ops = {

	.open = mtk_fm_i2s_awb_pcm_open,
	.close = mtk_fm_i2s_awb_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = mtk_mgrrx_awb_pcm_hw_params,
	.hw_free = mtk_fm_i2s_capture_pcm_hw_free,
	.prepare = mtk_fm_i2s_awb_pcm_prepare,
	.trigger = mtk_capture_fm_i2s_pcm_trigger,
	.pointer = mtk_awb_pcm_pointer,
	.copy = mtk_fm_i2s_awb_pcm_copy,
	.silence = mtk_capture_pcm_silence,
	.page = mtk_fm_i2s_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform = {

	.ops = &mtk_fm_i2s_awb_ops,
	.pcm_new = mtk_asoc_fm_i2s_awb_pcm_new,
	.probe = mtk_afe_fm_i2s_awb_probe,
};

static int mtk_fm_i2s_awb_probe(struct platform_device *pdev)
{
	pr_debug("mtk_fm_i2s_awb_probe\n");

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);

	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;

	if (pdev->dev.of_node)
		dev_set_name(&pdev->dev, "%s", MT_SOC_FM_I2S_AWB_PCM);

	pr_debug("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
	return snd_soc_register_platform(&pdev->dev, &mtk_soc_platform);
}

static int mtk_asoc_fm_i2s_awb_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	pr_debug("mtk_asoc_fm_i2s_awb_pcm_new\n");
	return 0;
}


static int mtk_afe_fm_i2s_awb_probe(struct snd_soc_platform *platform)
{
	pr_debug("mtk_afe_fm_i2s_awb_probe\n");
	AudDrv_Allocate_mem_Buffer(platform->dev,
		Soc_Aud_Digital_Block_MEM_AWB,
		FM_I2S_MAX_BUFFER_SIZE);
	Awb_Capture_dma_buf = Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_AWB);
	return 0;
}


static int mtk_fm_i2s_awb_remove(struct platform_device *pdev)
{
	pr_debug("%s\n", __func__);
	snd_soc_unregister_platform(&pdev->dev);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_fm_i2s_awb_of_ids[] = {

	{.compatible = "mediatek,mt8163-soc-pcm-fm-i2s-awb",},
	{}
};
#endif

static struct platform_driver mtk_fm_i2s_awb_capture_driver = {

	.driver = {
		   .name = MT_SOC_FM_I2S_AWB_PCM,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = mt_soc_pcm_fm_i2s_awb_of_ids,
#endif
		   },
	.probe = mtk_fm_i2s_awb_probe,
	.remove = mtk_fm_i2s_awb_remove,
};

#ifndef CONFIG_OF
static struct platform_device *soc_fm_i2s_capture_dev;
#endif

static int __init mtk_soc_fm_i2s_awb_platform_init(void)
{
	int ret = 0;

	pr_debug("%s\n", __func__);
#ifndef CONFIG_OF
	soc_fm_i2s_capture_dev =
		platform_device_alloc(MT_SOC_FM_I2S_AWB_PCM, -1);

	if (!soc_fm_i2s_capture_dev)
		return -ENOMEM;

	ret = platform_device_add(soc_fm_i2s_capture_dev);
	if (ret != 0) {
		platform_device_put(soc_fm_i2s_capture_dev);
		return ret;
	}
#endif
	ret = platform_driver_register(&mtk_fm_i2s_awb_capture_driver);
	return ret;
}

static void __exit mtk_soc_fm_i2s_awb_platform_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&mtk_fm_i2s_awb_capture_driver);
}
module_init(mtk_soc_fm_i2s_awb_platform_init);
module_exit(mtk_soc_fm_i2s_awb_platform_exit);

MODULE_DESCRIPTION("AFE PCM module platform driver");
MODULE_LICENSE("GPL");
