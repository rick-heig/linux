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

#define XILINX_PCIE_EP_MSI_X_TABLE_OFFSET	0x8000
#define XILINX_PCIE_EP_MSI_X_PBA_OFFSET		0x8fe0

struct xilinx_pcie {
	void	__iomem		*reg_base;	/* DT ctrl */
	void	__iomem		*hdr_conf;	/* DT hdr-conf */
	void	__iomem		*irq_reg;	/* DT irq-reg */
	void	__iomem		*msix_addr_reg; /* DT */
	void	__iomem		*msix_data_reg; /* DT */
	size_t			msix_table_offset; /* In BAR0 */
	size_t			msix_pba_offset;
	struct	pci_epf_bar	*bar_0;
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
};

static struct pci_epc_features xilinx_pcie_epc_features = {
	.msi_capable = true,
	/* Reserved BARs */
	.msix_capable = true,
	.reserved_bar = GENMASK(5, 3),
	/* Fixed BARs - All are fixed */
	.fixed_bar = GENMASK(5, 0),
	/* Only BAR0 and BAR2 are available, both are 64-bit */
	.bar_fixed_64bit = BIT(2) | BIT(0),
	/* Fixed sizes */ /** @todo get from DT */
	.bar_fixed_size = {
		SZ_128K,
		0,
		SZ_16M,
	},
	//.align = XILINX_PCIE_AT_SIZE_ALIGN, /** @todo get from DT */
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
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);
	struct xilinx_pcie *xilinx = &ep->xilinx;

	/// @todo implement this, get info from device tree, because it is fixed

	if (!epf_bar)
		return -EINVAL;

	switch (epf_bar->barno) {
	case BAR_0:
		/* Get BAR0, which we need for MSI-X */
		xilinx->bar_0 = epf_bar;
		if (!(epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64))
			dev_warn(&epc->dev, "BAR0 is not set to 64-bit !\n");
		/* This is fixed */ /** @todo get from DT */
		epf_bar->phys_addr = 0x470000000ULL;
		break;
	case BAR_2:
		if (!(epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64))
			dev_warn(&epc->dev, "BAR2 is not set to 64-bit !\n");
		/* This is fixed */ /** @todo get from DT */
		epf_bar->phys_addr = 0x480000000ULL;
		break;
	default: return -EINVAL;
	}

	/* This is fixed */
	epf_bar->size = xilinx_pcie_epc_features.bar_fixed_size[epf_bar->barno];

	epf_bar->addr = memremap(epf_bar->phys_addr, epf_bar->size, MEMREMAP_WC);
	if (!epf_bar->addr)
		return -ENOMEM;

	/* This is fixed */
	epf_bar->flags = PCI_BASE_ADDRESS_MEM_TYPE_64 |
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

static int xilinx_pcie_ep_set_msix(struct pci_epc *epc, u8 func_no, u8 vfunc_no,
				   u16 interrupts, enum pci_barno, u32 offset)
{
	/** @note The number of MSI-X IRQs is fixed in the IP ... */
	if (interrupts > 33)
		return -EINVAL;

	return 0;
}

static int xilinx_pcie_ep_get_msix(struct pci_epc *epc, u8 func_no, u8 vfunc_no)
{
	struct xilinx_pcie_ep *ep = epc_get_drvdata(epc);

	if (!ep->xilinx.bar_0 ||
	    !ep->xilinx.msix_addr_reg || !ep->xilinx.msix_data_reg) {
		dev_warn(&epc->dev, "Xilinx get msix return -ENODEV\n");
		return -ENODEV;
	}

	/** @note The number of MSI-X IRQs is fixed in the IP ... */
	return 33; /* We don't know how many the host has allocated ... */
	/* @todo we could check the values in the MSI table actually */
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

/*
 * Read a 32-bits BAR 0 register (equivalent to readl()).
 */
static inline u32 pci_ep_bar_0_reg_read32(struct xilinx_pcie_ep *ep,
					  u32 reg)
{
	volatile __le32 *ctrl_reg = ep->xilinx.bar_0->addr + reg;

	return le32_to_cpu(*ctrl_reg);
}

/*
 * Write a 32-bits BAR 0 register (equivalent to readl()).
 */
static inline void pci_ep_bar_0_reg_write32(struct xilinx_pcie_ep *ep,
					    u32 reg, u32 val)
{
	volatile __le32 *ctrl_reg = ep->xilinx.bar_0->addr + reg;

	*ctrl_reg = cpu_to_le32(val);
}

static int xilinx_pcie_ep_send_msix(struct xilinx_pcie_ep *ep, u8 fn, u16 irq)
{
	u32 addr_hi, addr_lo, data;

	if (fn || !irq || irq > 33)
		return -EINVAL;

	if (!ep->xilinx.bar_0 ||
	    !ep->xilinx.msix_addr_reg || !ep->xilinx.msix_data_reg)
		return -ENODEV;

	/* Get address from MSI-X table */
	addr_lo = pci_ep_bar_0_reg_read32(
		ep, XILINX_PCIE_EP_MSI_X_TABLE_OFFSET +
		    (irq - 1) * PCI_MSIX_ENTRY_SIZE + PCI_MSIX_ENTRY_LOWER_ADDR);
	addr_hi = pci_ep_bar_0_reg_read32(
		ep, XILINX_PCIE_EP_MSI_X_TABLE_OFFSET +
		    (irq - 1) * PCI_MSIX_ENTRY_SIZE + PCI_MSIX_ENTRY_UPPER_ADDR);

	/* Get data from MSI-X table */
	data = pci_ep_bar_0_reg_read32(
		ep, XILINX_PCIE_EP_MSI_X_TABLE_OFFSET +
		    (irq - 1) * PCI_MSIX_ENTRY_SIZE + PCI_MSIX_ENTRY_DATA);

	/* Check that it is not 0 */
	if (!addr_lo && !addr_hi) {
		dev_warn(&ep->epc->dev, "MSI-X address is NULL\n");
		return -EINVAL;
	}

	/* Write address to registers */
	writel(addr_lo, ep->xilinx.msix_addr_reg);
	writel(addr_hi, ep->xilinx.msix_addr_reg + SZ_8);
	/* Yes offset is 8 because Xilinx GPIO IP is used */

	/* Write data to registers */
	writel(data, ep->xilinx.msix_data_reg);

	/* Set bit in PBA array (this should be atomic, but oh well...) */
	data = pci_ep_bar_0_reg_read32(ep, XILINX_PCIE_EP_MSI_X_PBA_OFFSET);
	data |= 1 << (irq - 1);
	pci_ep_bar_0_reg_write32(ep, XILINX_PCIE_EP_MSI_X_PBA_OFFSET, data);

	/* Write bit in register */
	writel(0x10000, ep->xilinx.irq_reg);
	udelay(1);
	writel(0, ep->xilinx.irq_reg);

	/* Check msix sent / fail, we can't do this by polling because these
	 * signals only stay up for a single clock cycle (see PG194) */

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
	case PCI_IRQ_MSI:
		return xilinx_pcie_ep_send_irq(ep, fn, 0);
	case PCI_IRQ_MSIX:
		return xilinx_pcie_ep_send_msix(ep, fn, interrupt_num);
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
	.set_msix	= xilinx_pcie_ep_set_msix,
	.get_msix	= xilinx_pcie_ep_get_msix,
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

	/* Check for MSI-X property */
	xilinx_pcie_epc_features.msix_capable = of_property_read_bool(node,
		"msix-capable");

	/* Get access to the register to set MSI-X addr */
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "msix-addr-reg");
	xilinx->msix_addr_reg = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(xilinx->msix_addr_reg)) {
		dev_warn(dev, "Missing MSI-X address register\n");
		xilinx->msix_addr_reg = NULL;
		xilinx_pcie_epc_features.msix_capable = false;
	}

	/* Get access to the register to set MSI-X addr */
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "msix-data-reg");
	xilinx->msix_data_reg = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(xilinx->msix_data_reg)) {
		dev_warn(dev, "Missing MSI-X data register\n");
		xilinx->msix_data_reg = NULL;
		xilinx_pcie_epc_features.msix_capable = false;
	}

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
