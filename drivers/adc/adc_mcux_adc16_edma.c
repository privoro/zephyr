/*
 * Copyright (c) 2017-2018, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_kinetis_adc16

#include <errno.h>

#include <drivers/adc.h>
#include <drivers/dma.h>
#include <drivers/adc_mcux_edma.h>

#include <fsl_adc16.h>
#include <fsl_sim.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_mcux_adc16);

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

struct mcux_adc16_config {
	ADC_Type *base;
};

struct mcux_adc16_data {
	struct device *dev;
	struct device *dev_dma;
	struct adc_context ctx;
	struct adc_edma_config adc_dma_config;
	u16_t *buffer;
	u16_t *repeat_buffer;
	u32_t channels;
	u8_t channel_id;
};

#define DEV_CFG(dev) ((const struct mcux_adc16_config *const)dev->config_info)
#define DEV_DATA(dev) ((struct mcux_adc16_data *)dev->driver_data)
#define DEV_BASE(dev) ((ADC_Type *)DEV_CFG(dev)->base)

static void adc_dma_callback(void *callback_arg, u32_t channel, int error_code)
{
	struct device *dev = (struct device *)callback_arg;
	struct mcux_adc16_data *data = DEV_DATA(dev);
	LOG_DBG("DMA done");
	adc_context_on_sampling_done(&data->ctx, dev);
}

static int mcux_adc16_channel_setup(struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	u8_t channel_id = channel_cfg->channel_id;

	if (channel_id > (ADC_SC1_ADCH_MASK >> ADC_SC1_ADCH_SHIFT)) {
		LOG_ERR("Channel %d is not valid", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	if (true) {
		/* enable ADC trigger channel */
		SIM->SOPT7 |=
			SIM_SOPT7_ADC0TRGSEL(CONFIG_ADC_HW_TRIGGER_CHANNEL) |
			SIM_SOPT7_ADC0ALTTRGEN(1);
	}

	return 0;
}

static int start_read(struct device *dev, const struct adc_sequence *sequence)
{
	const struct mcux_adc16_config *config = DEV_CFG(dev);
	struct mcux_adc16_data *data = DEV_DATA(dev);
	adc16_hardware_average_mode_t mode;
	adc16_resolution_t resolution;
	int error;
	u32_t tmp32;
	ADC_Type *base = config->base;

	switch (sequence->resolution) {
	case 8:
	case 9:
		resolution = kADC16_Resolution8or9Bit;
		break;
	case 10:
	case 11:
		resolution = kADC16_Resolution10or11Bit;
		break;
	case 12:
	case 13:
		resolution = kADC16_Resolution12or13Bit;
		break;
#if defined(FSL_FEATURE_ADC16_MAX_RESOLUTION) &&                               \
	(FSL_FEATURE_ADC16_MAX_RESOLUTION >= 16U)
	case 16:
		resolution = kADC16_Resolution16Bit;
		break;
#endif
	default:
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	tmp32 = base->CFG1 & ~(ADC_CFG1_MODE_MASK);
	tmp32 |= ADC_CFG1_MODE(resolution);
	base->CFG1 = tmp32;

	switch (sequence->oversampling) {
	case 0:
		mode = kADC16_HardwareAverageDisabled;
		break;
	case 2:
		mode = kADC16_HardwareAverageCount4;
		break;
	case 3:
		mode = kADC16_HardwareAverageCount8;
		break;
	case 4:
		mode = kADC16_HardwareAverageCount16;
		break;
	case 5:
		mode = kADC16_HardwareAverageCount32;
		break;
	default:
		LOG_ERR("Invalid oversampling");
		return -EINVAL;
	}
	ADC16_SetHardwareAverage(config->base, mode);

	adc_context_start_read(&data->ctx, sequence);

	error = adc_context_wait_for_completion(&data->ctx);
	dma_stop(data->dev_dma, data->adc_dma_config.dma_channel);
	return error;
}

static int mcux_adc16_read(struct device *dev,
			   const struct adc_sequence *sequence)
{
	struct mcux_adc16_data *data = dev->driver_data;
	int error;

	adc_context_lock(&data->ctx, false, NULL);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}

#ifdef CONFIG_ADC_ASYNC
static int mcux_adc16_read_async(struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct mcux_adc16_data *data = dev->driver_data;
	int error;

	adc_context_lock(&data->ctx, true, async);
	error = start_read(dev, sequence);
	adc_context_release(&data->ctx, error);

	return error;
}
#endif

static void mcux_adc16_start_channel(struct device *dev)
{
	const struct mcux_adc16_config *config = dev->config_info;
	struct mcux_adc16_data *data = DEV_DATA(dev);

	adc16_channel_config_t channel_config;
	u32_t channel_group = 0U;

	data->channel_id = find_lsb_set(data->channels) - 1;

	LOG_DBG("Starting channel %d", data->channel_id);

#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	channel_config.enableDifferentialConversion = false;
#endif
	channel_config.enableInterruptOnConversionCompleted = true;
	channel_config.channelNumber = data->channel_id;

	ADC16_SetChannelConfig(config->base, channel_group, &channel_config);
	dma_start(data->dev_dma, data->adc_dma_config.dma_channel);
	LOG_DBG("Starting channel done");
}

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct mcux_adc16_data *data =
		CONTAINER_OF(ctx, struct mcux_adc16_data, ctx);

	data->channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	LOG_DBG("config dma");
	data->buffer = ctx->sequence.buffer;
	data->adc_dma_config.dma_block.block_size = ctx->sequence.buffer_size;
	data->adc_dma_config.dma_block.dest_address = (u32_t)data->buffer;
	data->adc_dma_config.dma_cfg.head_block =
		&(data->adc_dma_config.dma_block);
	dma_config(data->dev_dma, data->adc_dma_config.dma_channel,
	   &data->adc_dma_config.dma_cfg);

	mcux_adc16_start_channel(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct mcux_adc16_data *data =
		CONTAINER_OF(ctx, struct mcux_adc16_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static int mcux_adc16_init(struct device *dev)
{
	const struct mcux_adc16_config *config = dev->config_info;
	struct mcux_adc16_data *data = dev->driver_data;
	ADC_Type *base = config->base;
	adc16_config_t adc_config;

	LOG_DBG("init adc");
	ADC16_GetDefaultConfig(&adc_config);

#if 1
	/*try the fast ADC convertor settings*/
	/* use bus clock*/
	adc_config.clockSource = kADC16_ClockSourceAlt0;
	adc_config.clockDivider = kADC16_ClockDivider1;
	adc_config.enableHighSpeed = true;
	adc_config.enableContinuousConversion = true;
#endif

#if CONFIG_ADC_MCUX_ADC16_VREF_DEFAULT
	adc_config.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
#else /* CONFIG_ADC_MCUX_ADC16_VREF_ALTERNATE */
	adc_config.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
#endif

#if CONFIG_ADC_MCUX_ADC16_CLK_DIV_RATIO_1
	adc_config.clockDivider = kADC16_ClockDivider1;
#elif CONFIG_ADC_MCUX_ADC16_CLK_DIV_RATIO_2
	adc_config.clockDivider = kADC16_ClockDivider2;
#elif CONFIG_ADC_MCUX_ADC16_CLK_DIV_RATIO_4
	adc_config.clockDivider = kADC16_ClockDivider4;
#else /* CONFIG_ADC_MCUX_ADC16_CLK_DIV_RATIO_8 */
	adc_config.clockDivider = kADC16_ClockDivider8;
#endif
    //adc_config.clockSource = kADC16_ClockSourceAlt2;
    adc_config.longSampleMode = kADC16_LongSampleCycle10;
    adc_config.enableHighSpeed = true;
    adc_config.enableContinuousConversion = true;

	ADC16_Init(base, &adc_config);
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) &&                              \
	FSL_FEATURE_ADC16_HAS_CALIBRATION
	ADC16_SetHardwareAverage(base, kADC16_HardwareAverageCount32);
	ADC16_DoAutoCalibration(base);
#endif

/* Enable HW trigger */
#if CONFIG_ADC_HW_TRIGGER
	ADC16_EnableHardwareTrigger(base, true);
#endif
	/* Enable DMA. */
	ADC16_EnableDMA(base, true);

	data->dev = dev;

	/* dma related init */

	data->adc_dma_config.dma_name = CONFIG_DMA_0_NAME;
	data->adc_dma_config.dma_channel = CONFIG_ADC_EDMA_CHANNEL;
	data->adc_dma_config.dma_cfg.block_count = 1U;
	data->adc_dma_config.dma_cfg.dma_slot =
		DT_DMAS_CELL_BY_IDX(DT_NODELABEL(adc0), 0, source);
	data->adc_dma_config.dma_cfg.channel_direction = PERIPHERAL_TO_MEMORY;
	data->adc_dma_config.dma_cfg.source_burst_length = 4U;
	data->adc_dma_config.dma_cfg.dest_burst_length = 4U;
	data->adc_dma_config.dma_cfg.channel_priority = 0U;
	data->adc_dma_config.dma_cfg.dma_callback = adc_dma_callback;
	data->adc_dma_config.dma_cfg.callback_arg = dev;

	data->adc_dma_config.dma_cfg.source_data_size = 4U;
	data->adc_dma_config.dma_cfg.dest_data_size = 4U;
	data->adc_dma_config.dma_block.source_address = (u32_t)&base->R[0];

	data->dev_dma = device_get_binding(data->adc_dma_config.dma_name);
	if (data->dev_dma == NULL) {
		LOG_INF("dma binding fail");
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static const struct adc_driver_api mcux_adc16_driver_api = {
	.channel_setup = mcux_adc16_channel_setup,
	.read = mcux_adc16_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = mcux_adc16_read_async,
#endif
};

#define ACD16_MCUX_INIT(n)                                                     \
	static const struct mcux_adc16_config mcux_adc16_config_##n = {        \
		.base = (ADC_Type *)DT_INST_REG_ADDR(n),                       \
	};                                                                     \
                                                                               \
	static struct mcux_adc16_data mcux_adc16_data_##n = {                  \
		ADC_CONTEXT_INIT_TIMER(mcux_adc16_data_##n, ctx),              \
		ADC_CONTEXT_INIT_LOCK(mcux_adc16_data_##n, ctx),               \
		ADC_CONTEXT_INIT_SYNC(mcux_adc16_data_##n, ctx),               \
	};                                                                     \
                                                                               \
	DEVICE_AND_API_INIT(mcux_adc16_##n, DT_INST_LABEL(n),                  \
			    &mcux_adc16_init, &mcux_adc16_data_##n,            \
			    &mcux_adc16_config_##n, POST_KERNEL,               \
			    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,                \
			    &mcux_adc16_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ACD16_MCUX_INIT)
