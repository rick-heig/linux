// SPDX-License-Identifier: GPL-2.0+
/*
 * PCIe endpoint controller driver for Xilinx AXI PCIe Bridge
 *
 * Copyright (c) 2023 Rick Wertenbroek
 *
 */

#include <linux/configfs.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/pci-epc.h>
#include <linux/platform_device.h>
#include <linux/pci-epf.h>
#include <linux/sizes.h>

/*
 * Notes :
 *
 * Requirements for windows :
 * - AXI to PCI windows must be contiguous in memory map
 * - AXI to PCI windows must be all of the same size (a power of two)
 *
 */

#define XILINX_PCIE_EP_VID_OFFSET		0x0
#define XILINX_PCIE_EP_DEVID_SUBSYSID_OFFSET	0x8

#define XILINX_PCIE_EP_AXI_ADDRESS_TRANSLATION_REGS_OFFSET 0x208

struct xilinx_pcie {
	void	__iomem		*reg_base;	/* DT ctrl */
	void	__iomem		*hdr_conf;	/* DT hdr-conf */
	void	__iomem		*irq_reg;	/* irq-reg */
	struct	device		*dev;
	struct 	resource	*mem_res;	/* DT windows */
	u32			num_windows;
	u32			window_size;
};

struct xilinx_pcie_ep {
	struct xilinx_pcie	xilinx;
	struct pci_epc		*epc;
	u32			max_regions;
	unsigned long		ob_region_map;
	phys_addr_t		*ob_addr;
	phys_addr_t		irq_phys_addr;
	void __iomem		*irq_cpu_addr; /* here to write the MSI IRQ ? */
	u64			irq_pci_addr;
	u8			irq_pci_fn;
	u8			irq_pending;
};

static struct pci_epc_features xilinx_pcie_epc_features = {
	.msi_capable = true,
	/* Reserved BARs */
	.reserved_bar = GENMASK(5, 3),
	/* Fixed BARs - All are fixed */
	.fixed_bar = GENMASK(5, 0),
	/* Fixed sizes */ /** @todo get from DT */
	.bar_fixed_size = {
		SZ_128K,
		SZ_4K,
		SZ_16M,
	},
	//.align = XILINX_PCIE_AT_SIZE_ALIGN, /** @todo get from DT */
#if 0
	/* BARs with fixed address translations */ /** @todo get from DT */
	.bar_fixed_addr = {
		0x480000000ULL,
		0x480020000ULL,
		0x481000000ULL,
	},
#endif
};

static int xilinx_pcie_ep_write_header(struct pci_epc *epc, u8 fn, u8 vfn,
				       struct pci_epf_header *hdr)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	struct xilinx_pcie *xilinx = &ep->xilinx;
	u32 reg;

	if (xilinx->hdr_conf) {
		/* Other fields cannot be modified, they are set in Vivado */
		reg = hdr->vendorid | hdr->subsys_vendor_id << 16;
		writel(reg, xilinx->hdr_conf + XILINX_PCIE_EP_VID_OFFSET);

		reg = hdr->deviceid | hdr->subsys_id << 16;
		writel(reg, xilinx->hdr_conf + XILINX_PCIE_EP_DEVID_SUBSYSID_OFFSET);
	} else {
		dev_warn(xilinx->dev, "Cannot modify header, interface not provided\n");
	}

	dev_warn(xilinx->dev, "Cannot set class-code (baseclass, subclass, prog-if codes)"
		              ", must be set in IP before gnerating bitstream\n");

	return 0;
}

static int xilinx_pcie_ep_get_fixed_bar(struct pci_epc *epc, u8 fn, u8 vfn,
					struct pci_epf_bar *epf_bar)
{
	//struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	//struct xilinx_pcie *xilinx = &ep->xilinx;

	/// @todo implement this, get info from device tree, because it is fixed

	if (!epf_bar)
		return -EINVAL;

	switch (epf_bar->barno) {
	case BAR_0:
		/* This is fixed */ /** @todo get from DT */
		epf_bar->phys_addr = 0x480000000ULL;
		break;
	case BAR_1:
		/* This is fixed */ /** @todo get from DT */
		epf_bar->phys_addr = 0x480020000ULL;
		break;
	case BAR_2:
		/* This is fixed */ /** @todo get from DT */
		epf_bar->phys_addr = 0x481000000ULL;
		break;
	default: return -EINVAL;
	}

	/* This is fixed */
	epf_bar->size = xilinx_pcie_epc_features.bar_fixed_size[epf_bar->barno];
	/* Here we explicitely cast to a void* and drop the __iomem qualifier
	 * this is to be compatible with the current API, but actually it's
	 * still iomem and should be used at such, e.g., unaligned 64-bit access
	 * isn't possible */
	epf_bar->addr = (void *)ioremap(epf_bar->phys_addr, epf_bar->size);
	/* This is fixed */
	epf_bar->flags = PCI_BASE_ADDRESS_MEM_TYPE_32 |
			 PCI_BASE_ADDRESS_SPACE_MEMORY;

	return 0;
}

static int xilinx_pcie_ep_map_info(struct pci_epc *epc, u8 fn, u8 vfn,
				   struct pci_epc_map *map)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	phys_addr_t ofst, mask = ep->xilinx.window_size - 1;

	ofst = map->pci_addr & mask;
	if (ofst + map->size > ep->xilinx.window_size)
		map->size = ep->xilinx.window_size - ofst;

	map->phys_size = ALIGN(ofst + map->size, ep->xilinx.window_size);
	map->phys_ofst = ofst;

	return 0;
}

static int xilinx_pcie_ep_map_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				   phys_addr_t addr, u64 pci_addr,
				   size_t size)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	struct xilinx_pcie *xilinx = &ep->xilinx;

	u32 addr0, addr1, r;

	if (addr < xilinx->mem_res->start || addr >= xilinx->mem_res->start +
	    xilinx->num_windows * xilinx->window_size)
		return -EINVAL;

	/* The window size is a power of two */
	addr0 = lower_32_bits(pci_addr) & ~(xilinx->window_size - 1);
	addr1 = upper_32_bits(pci_addr);
	r = (addr - xilinx->mem_res->start) >> ilog2(xilinx->window_size);

	//dev_info(xilinx->dev, "Mapping phys addr : %pa\n", &addr);
	//dev_info(xilinx->dev, "pci addr : %#llx\n", pci_addr);
	//dev_info(xilinx->dev, "addr0 : %#x, addr1 : %#x\n", addr0, addr1);
	//dev_info(xilinx->dev, "r : %d\n", r);

	/* PCI bus address region, upper comes first, see Xilinx PG194 */
	writel(addr1, xilinx->reg_base +
			XILINX_PCIE_EP_AXI_ADDRESS_TRANSLATION_REGS_OFFSET +
			sizeof(u64) * r);
	writel(addr0, xilinx->reg_base +
			XILINX_PCIE_EP_AXI_ADDRESS_TRANSLATION_REGS_OFFSET +
			sizeof(u64) * r + sizeof(u32));

	set_bit(r, &ep->ob_region_map);
	ep->ob_addr[r] = addr;

	return 0;
}

static void xilinx_pcie_ep_unmap_addr(struct pci_epc *epc, u8 fn, u8 vfn,
				      phys_addr_t addr)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	struct xilinx_pcie *xilinx = &ep->xilinx;
	u32 r;

	for (r = 0; r < ep->max_regions; r++)
		if (ep->ob_addr[r] == addr)
			break;

	if (r == ep->max_regions)
		return;

	/* PCI bus address region */
	writel(0, xilinx->reg_base +
			XILINX_PCIE_EP_AXI_ADDRESS_TRANSLATION_REGS_OFFSET +
			sizeof(u64) * r);
	writel(0, xilinx->reg_base +
			XILINX_PCIE_EP_AXI_ADDRESS_TRANSLATION_REGS_OFFSET +
			sizeof(u64) * r + sizeof(u32));

	ep->ob_addr[r] = 0;
	clear_bit(r, &ep->ob_region_map);
}

static int xilinx_pcie_ep_set_msi(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				  u8 interrupts)
{
	/** @todo Note that the number of MSI IRQs is fixed in IP ... */
	return 0;
}

static int xilinx_pcie_ep_get_msi(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	/** @todo Note that the number of MSI IRQs is fixed in IP ... */
	return 1;
}

static int xilinx_pcie_ep_send_irq(struct xilinx_pcie_ep *ep, u8 fn,
				   u8 intx)
{
	struct xilinx_pcie *xilinx = &ep->xilinx;

	writel(1, xilinx->irq_reg);
	udelay(1);
	writel(0, xilinx->irq_reg);
	return 0;
}

static int xilinx_pcie_ep_raise_irq(struct pci_epc *epc, u8 fn, u8 vfn,
				    unsigned int type,
				    u16 interrupt_num)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);

	/* Note (from PG194) :
	 * Multiples interrupt modes can be configured during IP configuration
	 * (in Vivado, fixed from Linux point of view), however only one mode
	 * is used at runtime. If multiple interrupt modes are enabled by the
	 * host avec PCI but enumration at runtime, the core uses the MSI
	 * interrupt over legacy interrupt. Both MSI and Legacy interrupt modes
	 * are sent using the same int_msi_* interface, and the core
	 * automatically picks the best available interrupt mode at runtime.
	 * MSI-X is implemented externally to the code and uses a separate
	 * cfg_interrupt_msix_* interface. The core does not prevent the use of
	 * MSI-X at any time even when other interrupt modes are enabled,
	 * however Xilinx recommends the use of MSI-X interrupt solely if
	 * enabled over other interrupt modes even if they are all enabled at
	 * runtime. */

	/* So for the moment support only one MSI IRQ and legacy if MSI is
	   not enabled */

	switch (type) {
	case PCI_IRQ_LEGACY:
		return xilinx_pcie_ep_send_irq(ep, fn, 0);
	/* Do not support MSI / MSI-X for the moment */
	case PCI_IRQ_MSI:
		return xilinx_pcie_ep_send_irq(ep, fn, 0);
	default:
		return -EINVAL;
	}
}

static int xilinx_pcie_ep_start(struct pci_epc *epc)
{
	/* There is nothing to do, it is autonomous */

	return 0;
}

static const struct pci_epc_features*
xilinx_pcie_ep_get_features(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	return &xilinx_pcie_epc_features;
}

static const struct pci_epc_ops xilinx_pcie_epc_ops = {
	.write_header	= xilinx_pcie_ep_write_header,
	/* This has fixed BARs */
	.set_bar	= NULL,
	.clear_bar	= NULL,
	.get_fixed_bar  = xilinx_pcie_ep_get_fixed_bar,
	.map_info	= xilinx_pcie_ep_map_info,
	.map_addr	= xilinx_pcie_ep_map_addr,
	.unmap_addr	= xilinx_pcie_ep_unmap_addr,
	/* Don't support MSI or MSI-X for the moment */
	.set_msi	= xilinx_pcie_ep_set_msi,
	.get_msi	= xilinx_pcie_ep_get_msi,
	.raise_irq	= xilinx_pcie_ep_raise_irq,
	.start		= xilinx_pcie_ep_start,
	.get_features	= xilinx_pcie_ep_get_features,
};

static int xilinx_pcie_parse_ep_dt(struct xilinx_pcie *xilinx,
				   struct xilinx_pcie_ep *ep)
{
	struct device *dev = xilinx->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct device_node *node = dev->of_node;
	struct resource *regs;
	int err;

	/**
	 * @todo implement this
	 * DONE This will need to get the address for the translations
	 * DONE The number of windows
	 * Informations about the BARs
	 * DONE Information about the IDs and how to get/set them
	 */

	/* Get the header configuration registers */
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "hdr-conf");
	xilinx->hdr_conf = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(xilinx->hdr_conf)) {
		dev_warn(dev, "Header configuration interface not provided"
			 ", PCI IDs will be constant\n");
		xilinx->hdr_conf = NULL;
	}

	/* Get access to the controller register space */
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ctrl");
	xilinx->reg_base = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(xilinx->reg_base))
		return PTR_ERR(xilinx->reg_base);

	/* Get access to the register to fire IRQs */
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "irq-reg");
	xilinx->irq_reg = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(xilinx->irq_reg))
		return PTR_ERR(xilinx->irq_reg);

	/* Get the window memory ressource */
	xilinx->mem_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						       "windows");
	if (!xilinx->mem_res)
		return -EINVAL;

	err = of_property_read_u32(node, "xlnx,num-windows",
				   &xilinx->num_windows);
	if (err < 0)
		xilinx->num_windows = 1;

	err = of_property_read_u32(node, "xlnx,window-size",
				   &xilinx->window_size);
	if (err < 0 || !is_power_of_2(xilinx->window_size))
		return -EINVAL;

	ep->max_regions = xilinx->num_windows;

	ep->ob_region_map = 0;

	/* Only one function supported */
	ep->epc->max_functions = 1;

	return 0;
}

static const struct of_device_id xilinx_pcie_ep_of_match[] = {
	{ .compatible = "xlnx,xilinx-pcie-ep"},
	{},
};

/// @todo dealloc on errors ?
static int xilinx_pcie_ep_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xilinx_pcie_ep *ep;
	struct xilinx_pcie *xilinx;
	struct pci_epc *epc;
	size_t max_regions;
	struct pci_epc_mem_window *windows = NULL;
	int err, i;

	dev_info(dev, "xilinx-pcie-ep: Probe\n");

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	xilinx = &ep->xilinx;
	xilinx->dev = dev;

	epc = devm_pci_epc_create(dev, &xilinx_pcie_epc_ops);
	if (IS_ERR(epc)) {
		dev_err(dev, "failed to create epc device\n");
		return PTR_ERR(epc);
	}

	ep->epc = epc;
	epc_set_drvdata(epc, ep);

	err = xilinx_pcie_parse_ep_dt(xilinx, ep);
	if (err) {
		dev_err(dev, "Could not parse device tree\n");
		return err;
	}

	max_regions = ep->max_regions;
	ep->ob_addr = devm_kcalloc(dev, max_regions, sizeof(*ep->ob_addr),
				   GFP_KERNEL);

	if (!ep->ob_addr)
		return -ENOMEM;

	windows = devm_kcalloc(dev, ep->max_regions,
			       sizeof(struct pci_epc_mem_window), GFP_KERNEL);
	if (!windows)
		return -ENOMEM;

	for (i = 0; i < ep->max_regions; i++) {
		windows[i].phys_base = xilinx->mem_res->start +
				       (xilinx->window_size * i);
		windows[i].size = xilinx->window_size;
		windows[i].page_size = xilinx->window_size;
	}
	err = pci_epc_multi_mem_init(epc, windows, ep->max_regions);
	devm_kfree(dev, windows);

	if (err < 0) {
		dev_err(dev, "failed to initialize the memory space\n");
		return err;
	}

#if 0
	ep->irq_cpu_addr = pci_epc_mem_alloc_addr(epc, &ep->irq_phys_addr,
						  SZ_1M);
	if (!ep->irq_cpu_addr) {
		dev_err(dev, "failed to reserve memory space for MSI\n");
		err = -ENOMEM;
		goto err_epc_mem_exit;
	}

	ep->irq_pci_addr = ROCKCHIP_PCIE_EP_DUMMY_IRQ_ADDR;

	/*
	 * MSI-X is not supported but the controller still advertises the MSI-X
	 * capability by default, which can lead to the Root Complex side
	 * allocating MSI-X vectors which cannot be used. Avoid this by skipping
	 * the MSI-X capability entry in the PCIe capabilities linked-list: get
	 * the next pointer from the MSI-X entry and set that in the MSI
	 * capability entry (which is the previous entry). This way the MSI-X
	 * entry is skipped (left out of the linked-list) and not advertised.
	 */
	cfg_msi = rockchip_pcie_read(rockchip, PCIE_EP_CONFIG_BASE +
				     ROCKCHIP_PCIE_EP_MSI_CTRL_REG);

	cfg_msi &= ~ROCKCHIP_PCIE_EP_MSI_CP1_MASK;

	cfg_msix_cp = rockchip_pcie_read(rockchip, PCIE_EP_CONFIG_BASE +
					 ROCKCHIP_PCIE_EP_MSIX_CAP_REG) &
					 ROCKCHIP_PCIE_EP_MSIX_CAP_CP_MASK;

	cfg_msi |= cfg_msix_cp;

	rockchip_pcie_write(rockchip, cfg_msi,
			    PCIE_EP_CONFIG_BASE + ROCKCHIP_PCIE_EP_MSI_CTRL_REG);

	rockchip_pcie_write(rockchip, PCIE_CLIENT_CONF_ENABLE,
			    PCIE_CLIENT_CONFIG);
#endif

	/// @todo maybe config some MSI/MSI-X things ?

	dev_info(dev, "Probe successful\n");

	return 0;
}

static struct platform_driver xilinx_pcie_ep_driver = {
	.driver = {
		.name = "xilinx-pcie-ep",
		.of_match_table = xilinx_pcie_ep_of_match,
	},
	.probe = xilinx_pcie_ep_probe,
};

builtin_platform_driver(xilinx_pcie_ep_driver);
