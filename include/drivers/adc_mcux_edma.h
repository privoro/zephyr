/**
 * @file
 * @brief ADC public API header file.
 */

/*
 * Copyright (c) 2020 , NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_ADC_MCUX_EDMA_H_
#define ZEPHYR_INCLUDE_DRIVERS_ADC_MCUX_EDMA_H_

#include <device.h>
#include <fsl_edma.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * This indicates the dma stream. Most members of the stream are
 * self-explanatory
 *
 * in_queue and out_queue are used as follows
 *   transmit stream:
 *      application provided buffer is queued to in_queue until loaded to DMA.
 *      when DMA channel is idle, buffer is retrieved from in_queue and loaded
 *      to DMA and queued to out_queue.
 *      when DMA completes, buffer is retrieved from out_queue and freed.
 *
 *   receive stream:
 *      driver allocates buffer from slab and loads DMA
 *      buffer is queued to in_queue
 *      when DMA completes, buffer is retrieved from in_queue and queued to
 *      out_queue
 *	when application reads, buffer is read (may optionally block) from
 *      out_queue and presented to application.
 */
struct adc_edma_config {
    const char *dma_name;
	s32_t state;
	u32_t dma_channel;
	void (*irq_call_back)(void);
	struct dma_config dma_cfg;
	struct dma_block_config dma_block;
};

#ifdef __cplusplus
}
#endif

#endif  /* ZEPHYR_INCLUDE_DRIVERS_ADC_H_ */
