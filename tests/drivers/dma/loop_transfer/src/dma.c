/* dma.c - DMA test source file */

/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>

#include <device.h>
#include <drivers/dma.h>
#include <sys/printk.h>
#include <string.h>

/* in millisecond */
#define SLEEPTIME 100

#define TRANSFER_LOOPS (5)
#define RX_BUFF_SIZE (64)

#if CONFIG_NOCACHE_MEMORY
static const char TX_DATA[] = "The quick brown fox jumps over the lazy dog";
static __aligned(16) char tx_data[64] __used
	__attribute__((__section__(".nocache")));
static __aligned(16) char rx_data[TRANSFER_LOOPS][RX_BUFF_SIZE] __used
	__attribute__((__section__(".nocache.dma")));
#else
/* pad to times of 8*/
static const __aligned(16) char tx_data[] =
	"The quick brown fox jumps over the lazy dog";
static __aligned(16) char rx_data[TRANSFER_LOOPS][RX_BUFF_SIZE] = { { 0 } };
#endif

#define DMA_DEVICE_NAME "DMA_0"

volatile u8_t transfer_count;
volatile u8_t transfer_done = 0;
static struct dma_config dma_cfg = { 0 };
static struct dma_block_config dma_block_cfg = { 0 };

static void test_transfer(struct device *dev, u32_t id)
{
	int ret;
	transfer_count++;
#if 1
	if (transfer_count < TRANSFER_LOOPS) {
		dma_block_cfg.block_size = sizeof(tx_data);
		dma_block_cfg.source_address = (u32_t)tx_data;
		dma_block_cfg.dest_address = (u32_t)rx_data[transfer_count];

		ret = dma_config(dev, id, &dma_cfg);
		if (ret == 0) {
        		transfer_done = 1;
			    //dma_start(dev, id);
		}
	}
#endif
}

static void test_error(void)
{
	printk("DMA could not proceed, an error occurred\n");
}

static void dma_user_callback(void *arg, u32_t id, int error_code)
{
	struct device *dev = (struct device *)arg;

	if (error_code == 0) {
		test_transfer(dev, id);
	} else {
		test_error();
	}
}

void main(void)
{
	struct device *dma;
	static u32_t chan_id;

	printk("DMA memory to memory transfer started on %s\n",
	       DMA_DEVICE_NAME);
	printk("Preparing DMA Controller\n");

#if CONFIG_NOCACHE_MEMORY
	memset(tx_data, 0, sizeof(tx_data));
	memcpy(tx_data, TX_DATA, sizeof(TX_DATA));
#endif

	memset(rx_data, 0, sizeof(rx_data));

	dma = device_get_binding(DMA_DEVICE_NAME);
	if (!dma) {
		printk("Cannot get dma controller\n");
		return;
	}

	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.source_burst_length = 1U;
	dma_cfg.dest_burst_length = 1U;
	dma_cfg.callback_arg = dma;
	dma_cfg.dma_callback = dma_user_callback;
	dma_cfg.block_count = 1U;
	dma_cfg.head_block = &dma_block_cfg;
#ifdef CONFIG_DMA_MCUX_TEST_SLOT_START
	dma_cfg.dma_slot = CONFIG_DMA_MCUX_TEST_SLOT_START;
#endif

	chan_id = 0U;
	transfer_count = 0;
	printk("Starting the transfer and waiting for 1 second\n");
	printk("TX data: %s\n", tx_data);
	printk("block_size %d\n", strlen(tx_data));
	dma_block_cfg.block_size = strlen(tx_data);
	dma_block_cfg.source_address = (u32_t)tx_data;
	dma_block_cfg.dest_address = (u32_t)rx_data[transfer_count];

	if (dma_config(dma, chan_id, &dma_cfg)) {
		printk("ERROR: transfer config\n");
		return;
	}

	if (dma_start(dma, chan_id)) {
		printk("ERROR: transfer start\n");
		return;
	}

	//k_sleep(K_MSEC(SLEEPTIME));
#if 1
        while (transfer_count < TRANSFER_LOOPS) {
             if (transfer_done) {
        	dma_start(dma, chan_id);
                transfer_done = 0;
             }
	     k_sleep(K_MSEC(SLEEPTIME));
	}
#endif
	if (transfer_count < TRANSFER_LOOPS) {
		transfer_count = TRANSFER_LOOPS;
		printk("ERROR: unfinished transfer\n");
		if (dma_stop(dma, chan_id)) {
			printk("ERROR: transfer stop\n");
		}
	}

	printk("Each RX buffer should contain the full TX buffer string.\n");
	printk("TX data: %s\n", tx_data);

	for (int i = 0; i < TRANSFER_LOOPS; i++) {
		printk("RX data Loop %d: %s\n", i, rx_data[i]);
	}

	printk("Finished: DMA\n");
}
