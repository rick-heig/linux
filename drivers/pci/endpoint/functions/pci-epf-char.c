// SPDX-License-Identifier: GPL-2.0
/*
 * Char device PCI endpoint function
 * Author : Rick Wertenbroek <rick.wertenbroek@gmail.com>
 *
 * Based on :
 * Test driver to test endpoint functionality
 *
 * Copyright (C) 2017 Texas Instruments
 * Author: Kishon Vijay Abraham I <kishon@ti.com>
 *
 */

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/module.h>
#include <linux/pci_ids.h>
#include <linux/random.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>

#define DEVICE_NAME "pci_epf_char"
#define CLASS_NAME  "pci_epf_char"

static int dev_major = 0;

#define TIMER_RESOLUTION		1

struct pci_epf_char;

struct pci_epf_char_dev_data {
	struct pci_epf_char		*epf_char;
	struct cdev			cdev;
};

struct pci_epf_char {
	void				*reg[PCI_STD_NUM_BARS];
	struct pci_epf			*epf;
	enum pci_barno			test_reg_bar;
	size_t				msix_table_offset;
	struct dma_chan			*dma_chan_tx;
	struct dma_chan			*dma_chan_rx;
	struct dma_chan			*transfer_chan;
	dma_cookie_t			transfer_cookie;
	enum dma_status			transfer_status;
	struct completion		transfer_complete;
	bool				dma_supported;
	bool				dma_private;
	bool				link_up;
	struct pci_epf_char_dev_data    chardev_data;
	struct class 			*char_class;
	const struct pci_epc_features   *epc_features;
};

static struct pci_epf_header test_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576 };

static void pci_epf_char_dma_callback(void *param)
{
	struct pci_epf_char *epf_char = param;
	struct dma_tx_state state;

	epf_char->transfer_status =
		dmaengine_tx_status(epf_char->transfer_chan,
				    epf_char->transfer_cookie, &state);
	if (epf_char->transfer_status == DMA_COMPLETE ||
	    epf_char->transfer_status == DMA_ERROR)
		complete(&epf_char->transfer_complete);
}

/**
 * pci_epf_char_data_transfer() - Function that uses dmaengine API to transfer
 *				  data between PCIe EP and remote PCIe RC
 * @epf_char: the EPF test device that performs the data transfer operation
 * @dma_dst: The destination address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @dma_src: The source address of the data transfer. It can be a physical
 *	     address given by pci_epc_mem_alloc_addr or DMA mapping APIs.
 * @len: The size of the data transfer
 * @dma_remote: remote RC physical address
 * @dir: DMA transfer direction
 *
 * Function that uses dmaengine API to transfer data between PCIe EP and remote
 * PCIe RC. The source and destination address can be a physical address given
 * by pci_epc_mem_alloc_addr or the one obtained using DMA mapping APIs.
 *
 * The function returns '0' on success and negative value on failure.
 */
static int pci_epf_char_data_transfer(struct pci_epf_char *epf_char,
				      dma_addr_t dma_dst, dma_addr_t dma_src,
				      size_t len, dma_addr_t dma_remote,
				      enum dma_transfer_direction dir)
{
	struct dma_chan *chan = (dir == DMA_MEM_TO_DEV) ?
				 epf_char->dma_chan_tx : epf_char->dma_chan_rx;
	dma_addr_t dma_local = (dir == DMA_MEM_TO_DEV) ? dma_src : dma_dst;
	enum dma_ctrl_flags flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	struct pci_epf *epf = epf_char->epf;
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config sconf = {};
	struct device *dev = &epf->dev;
	int ret;

	if (IS_ERR_OR_NULL(chan)) {
		dev_err(dev, "Invalid DMA memcpy channel\n");
		return -EINVAL;
	}

	if (epf_char->dma_private) {
		sconf.direction = dir;
		if (dir == DMA_MEM_TO_DEV)
			sconf.dst_addr = dma_remote;
		else
			sconf.src_addr = dma_remote;

		if (dmaengine_slave_config(chan, &sconf)) {
			dev_err(dev, "DMA slave config fail\n");
			return -EIO;
		}
		tx = dmaengine_prep_slave_single(chan, dma_local, len, dir,
						 flags);
	} else {
		tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len,
					       flags);
	}

	if (!tx) {
		dev_err(dev, "Failed to prepare DMA memcpy\n");
		return -EIO;
	}

	reinit_completion(&epf_char->transfer_complete);
	epf_char->transfer_chan = chan;
	tx->callback = pci_epf_char_dma_callback;
	tx->callback_param = epf_char;
	epf_char->transfer_cookie = dmaengine_submit(tx);

	ret = dma_submit_error(epf_char->transfer_cookie);
	if (ret) {
		dev_err(dev, "Failed to do DMA tx_submit %d\n", ret);
		goto terminate;
	}

	dma_async_issue_pending(chan);
	ret = wait_for_completion_interruptible(&epf_char->transfer_complete);
	if (ret < 0) {
		dev_err(dev, "DMA wait_for_completion interrupted\n");
		goto terminate;
	}

	if (epf_char->transfer_status == DMA_ERROR) {
		dev_err(dev, "DMA transfer failed\n");
		ret = -EIO;
	}

terminate:
	dmaengine_terminate_sync(chan);

	return ret;
}

struct epf_dma_filter {
	struct device *dev;
	u32 dma_mask;
};

static bool epf_dma_filter_fn(struct dma_chan *chan, void *node)
{
	struct epf_dma_filter *filter = node;
	struct dma_slave_caps caps;

	memset(&caps, 0, sizeof(caps));
	dma_get_slave_caps(chan, &caps);

	return chan->device->dev == filter->dev
		&& (filter->dma_mask & caps.directions);
}

/**
 * pci_epf_char_init_dma_chan() - Function to initialize EPF test DMA channel
 * @epf_char: the EPF test device that performs data transfer operation
 *
 * Function to initialize EPF test DMA channel.
 */
static int pci_epf_char_init_dma_chan(struct pci_epf_char *epf_char)
{
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	struct epf_dma_filter filter;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	filter.dev = epf->epc->dev.parent;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);
	if (!dma_chan) {
		dev_info(dev, "Failed to get private DMA rx channel. Falling back to generic one\n");
		goto fail_back_tx;
	}

	epf_char->dma_chan_rx = dma_chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	dma_chan = dma_request_channel(mask, epf_dma_filter_fn, &filter);

	if (!dma_chan) {
		dev_info(dev, "Failed to get private DMA tx channel. Falling back to generic one\n");
		goto fail_back_rx;
	}

	epf_char->dma_chan_tx = dma_chan;
	epf_char->dma_private = true;

	init_completion(&epf_char->transfer_complete);

	return 0;

fail_back_rx:
	dma_release_channel(epf_char->dma_chan_rx);
	epf_char->dma_chan_tx = NULL;

fail_back_tx:
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf_char->transfer_complete);

	epf_char->dma_chan_tx = epf_char->dma_chan_rx = dma_chan;

	return 0;
}

/**
 * pci_epf_char_clean_dma_chan() - Function to cleanup EPF test DMA channel
 * @epf_char: the EPF test device that performs data transfer operation
 *
 * Helper to cleanup EPF test DMA channel.
 */
static void pci_epf_char_clean_dma_chan(struct pci_epf_char *epf_char)
{
	if (!epf_char->dma_supported)
		return;

	dma_release_channel(epf_char->dma_chan_tx);
	if (epf_char->dma_chan_tx == epf_char->dma_chan_rx) {
		epf_char->dma_chan_tx = NULL;
		epf_char->dma_chan_rx = NULL;
		return;
	}

	dma_release_channel(epf_char->dma_chan_rx);
	epf_char->dma_chan_rx = NULL;

	return;
}

static void pci_epf_char_print_rate(struct pci_epf_char *epf_char,
				    const char *op, u64 size,
				    struct timespec64 *start,
				    struct timespec64 *end, bool dma)
{
	struct timespec64 ts = timespec64_sub(*end, *start);
	u64 rate = 0, ns;

	/* calculate the rate */
	ns = timespec64_to_ns(&ts);
	if (ns)
		rate = div64_u64(size * NSEC_PER_SEC, ns * 1000);

	dev_info(&epf_char->epf->dev,
		 "%s => Size: %llu B, DMA: %s, Time: %llu.%09u s, Rate: %llu KB/s\n",
		 op, size, dma ? "YES" : "NO",
		 (u64)ts.tv_sec, (u32)ts.tv_nsec, rate);
}

static int pci_epf_char_read(struct pci_epf_char *epf_char,
			     void *buffer,
			     u64 src_addr,
			     size_t len,
			     bool use_dma)
{
	int ret = 0;
	struct pci_epc_map map;
	struct timespec64 start, end;
	phys_addr_t dst_phys_addr;
	size_t src_size = len;
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	struct device *dma_dev = epf->epc->dev.parent;
	ssize_t map_size = 0;

	while (src_size) {
		map_size = pci_epf_mem_map(epf, src_addr, src_size, &map);
		if (map_size < 0) {
			dev_err(dev, "Failed to map address\n");
			ret = map_size;
			goto set_status;
		}

		if (use_dma) {
			dst_phys_addr = dma_map_single(dma_dev, buffer, map_size,
						       DMA_FROM_DEVICE);
			if (dma_mapping_error(dma_dev, dst_phys_addr)) {
				dev_err(dev,
					"Failed to map destination buffer addr\n");
				ret = -ENOMEM;
				goto unmap;
			}

			ktime_get_ts64(&start);
			ret = pci_epf_char_data_transfer(epf_char,
					dst_phys_addr, map.phys_addr,
					map_size, src_addr, DMA_DEV_TO_MEM);
			if (ret)
				dev_err(dev, "Data transfer failed\n");
			ktime_get_ts64(&end);

			dma_unmap_single(dma_dev, dst_phys_addr, map_size,
					 DMA_FROM_DEVICE);

			if (ret)
				goto unmap;
		} else {
			ktime_get_ts64(&start);
			memcpy_fromio(buffer, map.virt_addr, map_size);
			ktime_get_ts64(&end);
		}

		src_size -= map_size;
		src_addr += map_size;
		buffer += map_size;

		pci_epf_mem_unmap(epf, &map);
		map_size = 0;
	}

	pci_epf_char_print_rate(epf_char, "READ", len, &start, &end, use_dma);

unmap:
	if (map_size)
		pci_epf_mem_unmap(epf, &map);

set_status:
	return ret;
}

static int pci_epf_char_write(struct pci_epf_char *epf_char,
			      u64 dst_addr,
			      void *buffer,
			      size_t len,
			      bool use_dma)
{
	int ret = 0;
	struct pci_epc_map map;
	struct timespec64 start, end;
	phys_addr_t src_phys_addr;
	size_t dst_size = len;
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	struct device *dma_dev = epf->epc->dev.parent;
	ssize_t map_size = 0;

	while (dst_size) {
		map_size = pci_epf_mem_map(epf, dst_addr, dst_size, &map);
		if (map_size < 0) {
			dev_err(dev, "Failed to map address\n");
			ret = map_size;
			goto set_status;
		}

		if (use_dma) {
			src_phys_addr = dma_map_single(dma_dev, buffer, map_size,
						       DMA_TO_DEVICE);
			if (dma_mapping_error(dma_dev, src_phys_addr)) {
				dev_err(dev,
					"Failed to map source buffer addr\n");
				ret = -ENOMEM;
				goto unmap;
			}

			ktime_get_ts64(&start);

			ret = pci_epf_char_data_transfer(epf_char,
						map.phys_addr, src_phys_addr,
						map_size, dst_addr,
						DMA_MEM_TO_DEV);
			if (ret)
				dev_err(dev, "Data transfer failed\n");
			ktime_get_ts64(&end);

			dma_unmap_single(dma_dev, src_phys_addr, map_size,
					 DMA_TO_DEVICE);

			if (ret)
				goto unmap;
		} else {
			ktime_get_ts64(&start);
			memcpy_toio(map.virt_addr, buffer, map_size);
			ktime_get_ts64(&end);
		}

		dst_size -= map_size;
		dst_addr += map_size;
		buffer += map_size;

		pci_epf_mem_unmap(epf, &map);
		map_size = 0;
	}

	pci_epf_char_print_rate(epf_char, "WRITE", len, &start, &end, use_dma);

	/*
	 * wait 1ms inorder for the write to complete. Without this delay L3
	 * error in observed in the host system.
	 */
	usleep_range(1000, 2000);

unmap:
	if (map_size)
		pci_epf_mem_unmap(epf, &map);
set_status:
	return ret;
}

#if 0
static void pci_epf_char_cmd_handler(struct work_struct *work)
{
	u32 command;
	struct pci_epf_char *epf_char = container_of(work, struct pci_epf_char,
						     cmd_handler.work);
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	enum pci_barno test_reg_bar = epf_char->test_reg_bar;
	struct pci_epf_char_reg *reg = epf_char->reg[test_reg_bar];

	command = READ_ONCE(reg->command);
	if (!command)
		goto reset_handler;

	WRITE_ONCE(reg->command, 0);
	WRITE_ONCE(reg->status, 0);

	if ((READ_ONCE(reg->flags) & FLAG_USE_DMA) &&
	    !epf_char->dma_supported) {
		dev_err(dev, "Cannot transfer data using DMA\n");
		goto reset_handler;
	}

	if (reg->irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	switch (command) {
	case COMMAND_RAISE_LEGACY_IRQ:
	case COMMAND_RAISE_MSI_IRQ:
	case COMMAND_RAISE_MSIX_IRQ:
		pci_epf_char_raise_irq(epf_char, reg);
		break;
	case COMMAND_WRITE:
		pci_epf_char_write(epf_char, reg);
		pci_epf_char_raise_irq(epf_char, reg);
		break;
	case COMMAND_READ:
		pci_epf_char_read(epf_char, reg);
		pci_epf_char_raise_irq(epf_char, reg);
		break;
	case COMMAND_COPY:
		pci_epf_char_copy(epf_char, reg);
		pci_epf_char_raise_irq(epf_char, reg);
		break;
	default:
		dev_err(dev, "Invalid command 0x%x\n", command);
		break;
	}

reset_handler:
	queue_delayed_work(kpcitest_workqueue, &epf_char->cmd_handler,
			   msecs_to_jiffies(1));
}
#endif

static int dev_open(struct inode* inode, struct file* file) {
	struct pci_epf_char_dev_data *pecdd = container_of(inode->i_cdev,
		struct pci_epf_char_dev_data, cdev);
	file->private_data = pecdd;
	return 0;
}

static ssize_t dev_read(struct file* file, char* buffer, size_t len, loff_t* offset) {
	struct pci_epf_char_dev_data *pecdd = file->private_data;
	struct pci_epf_char *epf_char = pecdd->epf_char;
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	void* local_buffer = NULL;
	int error_count = 0;
	if (!offset)
		return -EINVAL;

	if (!epf_char->link_up) {
		dev_warn(dev, "Link is down cannot read\n");
		return -EFAULT;
	}

	local_buffer = kzalloc(len, GFP_KERNEL);
	if (!local_buffer)
		return -ENOMEM;

	dev_info(dev, "Request to read %zu bytes from offset %llu\n", len, *offset);

	pci_epf_char_read(epf_char, local_buffer, *offset, len, (len > SZ_4K));

	/* Maybe this is possible in zero-copy */
	error_count = copy_to_user(buffer, local_buffer, len);
	kfree(local_buffer);

	if (error_count != 0) {
		dev_err(dev, "Failed to send %d characters to the user\n", error_count);
		return -EFAULT;
	}

	return len;
}

static ssize_t dev_write(struct file* file, const char* buffer, size_t len, loff_t* offset) {
	struct pci_epf_char_dev_data *pecdd = file->private_data;
	struct pci_epf_char *epf_char = pecdd->epf_char;
	struct pci_epf *epf = epf_char->epf;
	struct device *dev = &epf->dev;
	void* local_buffer = NULL;
	if (!offset)
		return -EINVAL;

	dev_info(dev, "Request to write %zu bytes at offset %llu\n", len, *offset);

	if (!epf_char->link_up) {
		dev_warn(dev, "Link is down cannot write\n");
		return -EFAULT;
	}

	local_buffer = kzalloc(len, GFP_KERNEL);
	if (!local_buffer)
		return -ENOMEM;

	/* Maybe this is possible in zero-copy */
	if (copy_from_user(local_buffer, buffer, len)) {
		dev_err(dev, "Failed to copy data from user\n");
		return -EFAULT;
	}

	pci_epf_char_write(epf_char, *offset, local_buffer, len, (len > SZ_4K));
	kfree(local_buffer);
	return len;
}

static int dev_release(struct inode* inode, struct file* file) {
	return 0;
}

static struct file_operations fops = {
	.open = dev_open,
	.read = dev_read,
	.write = dev_write,
	.release = dev_release,
};

static void pci_epf_char_unbind(struct pci_epf *epf)
{
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);
	struct pci_epf_bar *epf_bar;
	const struct pci_epc_features *epc_features;
	int bar;

	epc_features = epf_char->epc_features;

	pci_epf_char_clean_dma_chan(epf_char);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (epf_char->reg[bar] &&
		    !(epc_features->fixed_bar & (1 << bar))) {
			pci_epf_clear_bar(epf, epf_bar);
			pci_epf_free_space(epf, epf_char->reg[bar], bar,
					   PRIMARY_INTERFACE);
		}
	}
}

static int pci_epf_char_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct device *dev = &epf->dev;
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);
	enum pci_barno test_reg_bar = epf_char->test_reg_bar;
	const struct pci_epc_features *epc_features;

	epc_features = epf_char->epc_features;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		if (!!(epc_features->fixed_bar & (1 << bar)))
			continue;

		ret = pci_epf_set_bar(epf, epf_bar);

		if (ret) {
			pci_epf_free_space(epf, epf_char->reg[bar], bar,
					   PRIMARY_INTERFACE);
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == test_reg_bar)
				return ret;
		}
	}

	return 0;
}

static int pci_epf_char_core_init(struct pci_epf *epf)
{
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	struct device *dev = &epf->dev;
	bool msix_capable = false;
	bool msi_capable = true;
	int ret;

	epc_features = pci_epf_get_features(epf);
	if (epc_features) {
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
	}

	if (epf->vfunc_no <= 1) {
		ret = pci_epf_write_header(epf, header);
		if (ret) {
			dev_err(dev, "Configuration header write failed\n");
			return ret;
		}
	}

	ret = pci_epf_char_set_bar(epf);
	if (ret)
		return ret;

	if (msi_capable) {
		ret = pci_epf_set_msi(epf, epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		ret = pci_epf_set_msix(epf, epf->msix_interrupts,
				       epf_char->test_reg_bar,
				       epf_char->msix_table_offset);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	/* If there is no notifier assume the link is up */
	if(!epc_features->linkup_notifier) {
		epf_char->link_up = true;
	}

	return 0;
}

static int pci_epf_char_link_up(struct pci_epf *epf)
{
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);

	epf_char->link_up = true;

	return 0;
}

static int pci_epf_char_link_down(struct pci_epf *epf)
{
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);

	epf_char->link_up = false;

	return 0;
}

static const struct pci_epf_event_ops pci_epf_char_event_ops = {
	.core_init = pci_epf_char_core_init,
	.link_up = pci_epf_char_link_up,
	.link_down = pci_epf_char_link_down,
};

static int pci_epf_char_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	size_t msix_table_size = 0;
	size_t test_reg_bar_size;
	size_t pba_size = 0;
	bool msix_capable;
	void *base;
	int bar, add, ret;
	enum pci_barno test_reg_bar = epf_char->test_reg_bar;
	const struct pci_epc_features *epc_features;
	size_t test_reg_size;

	epc_features = epf_char->epc_features;

	//test_reg_bar_size = ALIGN(sizeof(struct pci_epf_char_reg), 128);
	test_reg_bar_size = SZ_4K;

	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_char->msix_table_offset = test_reg_bar_size;
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	test_reg_size = test_reg_bar_size + msix_table_size + pba_size;

	if (epc_features->bar_fixed_size[test_reg_bar]) {
		if (test_reg_size > bar_size[test_reg_bar])
			return -ENOMEM;
		test_reg_size = bar_size[test_reg_bar];
	}

	if (!!(epc_features->fixed_bar & (1 << test_reg_bar))) {
		ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
					    epf->vfunc_no, test_reg_bar,
					    &epf->bar[test_reg_bar]);
		if (ret < 0) {
			dev_err(dev, "Failed to get fixed bar");
			return ret;
		}
		base = epf->bar[test_reg_bar].addr;
	} else {
		base = pci_epf_alloc_space(epf, test_reg_size, test_reg_bar,
					   epc_features->align, PRIMARY_INTERFACE);
		if (!base) {
			dev_err(dev, "Failed to allocated register space\n");
			return -ENOMEM;
		}
	}
	epf_char->reg[test_reg_bar] = base;

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;

		if (bar == test_reg_bar)
			continue;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		if (!!(epc_features->fixed_bar & (1 << bar))) {
			ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
						    epf->vfunc_no, bar,
						    epf_bar);
			if (ret < 0)
				base = NULL;
			else
				base = epf->bar[bar].addr;
		} else {
			base = pci_epf_alloc_space(epf, bar_size[bar], bar,
						   epc_features->align,
						   PRIMARY_INTERFACE);
		}

		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n",
				bar);
		epf_char->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_configure_bar(struct pci_epf *epf,
				  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			bar_size[i] = epc_features->bar_fixed_size[i];
	}
}

static int pci_epf_char_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_char *epf_char = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno test_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epf_get_features(epf);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	linkup_notifier = epc_features->linkup_notifier;
	core_init_notifier = epc_features->core_init_notifier;
	test_reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (test_reg_bar < 0)
		return -EINVAL;
	pci_epf_configure_bar(epf, epc_features);

	epf_char->test_reg_bar = test_reg_bar;
	epf_char->epc_features = epc_features;

	ret = pci_epf_char_alloc_space(epf);
	if (ret)
		return ret;

	if (!core_init_notifier) {
		ret = pci_epf_char_core_init(epf);
		if (ret)
			return ret;
	}

	epf_char->dma_supported = true;

	ret = pci_epf_char_init_dma_chan(epf_char);
	if (ret)
		epf_char->dma_supported = false;

	/* If there is no notifier at all, assume link is up */
	if (!linkup_notifier && !core_init_notifier)
		epf_char->link_up = true;

	return 0;
}

static const struct pci_epf_device_id pci_epf_char_ids[] = {
	{
		.name = "pci_epf_char",
	},
	{},
};

static int pci_epf_char_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	struct pci_epf_char *epf_char;
	struct device *dev = &epf->dev;
	int ret = 0;

	epf_char = devm_kzalloc(dev, sizeof(*epf_char), GFP_KERNEL);
	if (!epf_char)
		return -ENOMEM;

	epf->header = &test_header;
	epf_char->epf = epf;

	epf->event_ops = &pci_epf_char_event_ops;

	epf_set_drvdata(epf, epf_char);

	cdev_init(&epf_char->chardev_data.cdev, &fops);
	epf_char->chardev_data.cdev.owner = THIS_MODULE;

	ret = cdev_add(&epf_char->chardev_data.cdev,
	MKDEV(dev_major, 0), 1);
	if (ret < 0) {
		dev_err(&epf->dev, "Could not add character device\n");
		return ret;
	}

	device_create(epf_char->char_class, NULL, MKDEV(dev_major, 0), NULL,
		      "pci-io");

	epf_char->chardev_data.epf_char = epf_char;

	return 0;
}

static struct pci_epf_ops ops = {
	.unbind	= pci_epf_char_unbind,
	.bind	= pci_epf_char_bind,
};

static struct pci_epf_driver char_driver = {
	.driver.name	= "pci_epf_char",
	.probe		= pci_epf_char_probe,
	.id_table	= pci_epf_char_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_char_init(void)
{
	int ret;

	ret = pci_epf_register_driver(&char_driver);
	if (ret) {
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_char_init);

#if 0 /* XXX Should be moved to missing remove function */
static void pci_epf_chardev_exit(void) {
	device_destroy(char_class, MKDEV(major_number, 0));
	class_unregister(char_class);
	class_destroy(char_class);
	unregister_chrdev(major_number, DEVICE_NAME);
	printk(KERN_INFO "Goodbye from the char device!\n");
}
#endif

static void __exit pci_epf_char_exit(void)
{
	pci_epf_unregister_driver(&char_driver);
	//pci_epf_chardev_exit();
}
module_exit(pci_epf_char_exit);

MODULE_DESCRIPTION("PCI EPF CHAR DRIVER");
MODULE_AUTHOR("Rick Wertenbroek <rick.wertenbroek@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.0");
