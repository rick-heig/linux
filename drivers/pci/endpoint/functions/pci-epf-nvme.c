// SPDX-License-Identifier: GPL-2.0
/*
 * NVMe function driver for PCI Endpoint Framework
 *
 * Copyright (C) 2019 SiFive
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/nvme.h>
#include <linux/list.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/cdev.h>
#include <linux/uio_driver.h>
#include <net/sock.h>
#include <generated/utsrelease.h>

#include "../../../nvme/host/nvme.h"
#include "../../../nvme/host/fabrics.h"
#include "../../../nvme/target/nvmet.h"

/*
 * Time to (busy) wait in microseconds if the completion queue is full
 */
#define PCI_EPF_NVME_CQ_FULL_DELAY_US 10

/*
 * Size of the FIFOs used in driver, should be bigger than MQES
 */
#define PCI_EPF_NVME_FIFO_SIZE 2048

/*
 * Maximum number of IO queues supported
 */
#define PCI_EPF_NVME_MAX_IO_QUEUES 32
/*
 * Maximum number of queues Admin queue included
 */
#define PCI_EPF_NVME_MAX_QUEUES (PCI_EPF_NVME_MAX_IO_QUEUES + 1)

/*
 *
 */
#define PCI_EPF_NVME_MIN_POLL_DELAY_US 50

#define CMB_PAGE_SHIFT			12
#define CMB_PAGE_SIZE			(1 << CMB_PAGE_SHIFT)
#define CMB_BIR_MASK			0x7
#define CMB_BIR				2
#define CMBMSC_CRE_BIT			(1 << 0)
#define CMBMSC_CMSE_BIT			(1 << 1)
#define CMBMSC_CBA_MASK			GENMASK(63, 12)

/* Network relay related below */
#define TSP_RELAY_BUFFER_SIZE (SZ_4K - sizeof(u32))
#define TSP_RELAY_MAX_RELAYS 4

struct tsp_relay {
	struct socket		*socket;
	struct task_struct	*thread;
	DECLARE_KFIFO_PTR(fifo, typeof(struct pci_epf_nvme_cmd *));
	bool			shutdown;
	struct wait_queue_head	wq;
};

struct pci_epf_nvme;

struct tsp_relay_thread_args {
	struct pci_epf_nvme	*nvme;
	int			relay_desc;
};

struct __attribute__((packed)) addr_info {
	char	node[256];
	char	service[256];
	//s32	flags;
	//s32	family;
	//s32	socktype;
	//s32	protocol
};

/* Local Memory Management */
typedef struct LinkedList_t {
    size_t size;
    void *ptr;
    struct LinkedList_t *prev;
    struct LinkedList_t *next;
    int used;
} LinkedList_t;

typedef struct Context_t {
    void *base_address;
    size_t size;
    size_t used;
    LinkedList_t *head;
} Context_t;

#define CONTEXT(ctx) ((Context_t*)ctx)

int tsp_mm_init_context(void *memory_region, size_t memory_region_size, void **opaque_p) {
	Context_t *ctx;

	if (!opaque_p) {
		return -EINVAL;
	}

	*opaque_p = kmalloc(sizeof(Context_t), GFP_USER);
	if (!*opaque_p) {
		return -ENOMEM;
	}

	ctx = CONTEXT(*opaque_p);
	ctx->base_address = memory_region;
	ctx->size = memory_region_size;
	ctx->used = 0;

	ctx->head = kmalloc(sizeof(LinkedList_t), GFP_USER);
	if (!ctx->head) {
		kfree(*opaque_p);
		*opaque_p = NULL;
		return -ENOMEM;
	}

	ctx->head->size = memory_region_size;
	ctx->head->ptr = memory_region;
	ctx->head->next = NULL;
	ctx->head->prev = NULL;
	ctx->head->used = 0;

	return 0;
}

static void *_reserve(size_t size, LinkedList_t* ll, Context_t* ctx) {
	LinkedList_t *next, *new;
	size_t total_size;

	total_size = ll->size;
	next = ll->next;

	// Update linked list
	ll->size = size;

	// Split linked list if not exact allocation
	if (size != total_size) {
		new = kmalloc(sizeof(LinkedList_t), GFP_USER);
		if (!new)
			return NULL;

		ll->next = new;

		new->prev = ll;
		new->size = total_size - size;
		new->ptr = (void *)((char *)ll->ptr + size);
		new->next = next;
		if (next) {
			next->prev = new;
		}
		new->used = 0;
	}

	// Block is now used (reserved)
	ll->used = 1;
	ctx->used += size;
	return ll->ptr;
}

void *tsp_mm_alloc(size_t size, void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return NULL;
	if (!size)
		return NULL;
	ctx = CONTEXT(opaque);

	ll = ctx->head;
	while(ll) {
		if (!ll->used && (ll->size >= size)) {
			return _reserve(size, ll, ctx);
		} else {
			ll = ll->next;
		}
	}

	return NULL; // Cannot fit
}

static void _try_merge_next(LinkedList_t *ll) {
	LinkedList_t *next;
	next = ll->next;

	if (next && !next->used) {
		// Merge next into current
		ll->size += next->size;
		ll->next = next->next;
		if (next->next) {
			next->next->prev = ll;
		}
		kfree(next); // Block disapears
	}
}

static void _unreserve(LinkedList_t *ll, Context_t *ctx) {
	ll->used = 0;
	ctx->used -= ll->size;

	_try_merge_next(ll);

	// If previous block is unused merge it into current block
	if (ll->prev && !ll->prev->used) {
		_try_merge_next(ll->prev);
		// ll is not necessarily a valid ptr anymore here
	}
}

static void tsp_mm_free(void *ptr, void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return;

	ctx = CONTEXT(opaque);

	ll = ctx->head;

	// Search entry
	while(ll && ll->ptr != ptr) {
		ll = ll->next;
	}

	if (ll && ll->used) {
		_unreserve(ll, ctx);
		// ll is not necessarily a valid ptr anymore here
	} else {
		// Should not happen
		pr_err("TSP Memory Management Pointer %#llx was not allocated... cannot free\n", (phys_addr_t)ptr);
	}
}

static void mm_quit_context(void *opaque) {
	Context_t *ctx;
	LinkedList_t *ll;

	if (!opaque)
		return;

	ctx = CONTEXT(opaque);

	ll = ctx->head;
	while(ll) {
		LinkedList_t *to_be_freed = ll;
		ll = ll->next;
		kfree(to_be_freed);
	}

	kfree(ctx);
}

#ifndef __TSP_CS_H__
#define __TSP_CS_H__

typedef struct __attribute__((packed)) CSEProperties {
	u16 HwVersion;
	u16 SwVersion;
	char UniqueName[32];     // an identifiable string for this CSE
	u16 NumBuiltinFunctions; // number of available preloaded functions
	u32 MaxRequestsPerBatch; // maximum number of requests supported per batch request
	u32 MaxFunctionParametersAllows;    // maximum number of parameters supported
	u32 MaxConcurrentFunctionInstances; // maximum number of function instances supported
} CSEProperties;

typedef struct __attribute__((packed)) CSxProperties {
	u16 HwVersion;         // specifies the hardware version of this CSx
	u16 SwVersion;         // specifies the software version that runs on this CSx
	u16 VendorId;          // specifies the vendor id of this CSx
	u16 DeviceId;          // specifies the device id of this CSx
	char FriendlyName[32];      // an identifiable string for this CSx
	u32 CFMinMB;           // Amount of CFM in megabytes installed in device
	u32 FDMinMB;           // amount of FDM in megabytes installed in device
	struct __attribute__((packed)) {
		u64 FDMIsDeviceManaged : 1;     // FDM allocations managed by device
		u64 FDMIsHostVisible : 1;       // FDM may be mapped to host address space
		u64 BatchRequestsSupported : 1; // CSx supports batch requests in hardware
		u64 StreamsSupported : 1;       // CSx supports streams in hardware
		u64 Reserved : 60;
	} Flags;
	u16 NumCSEs;
	CSEProperties CSE[1];  // see 6.3.4.1.14
} CSxProperties;

typedef struct __attribute__((packed)) CsCapabilities {
	// specifies the fixed functionality device capability
	struct __attribute__((packed)) {
		u64 Compression : 1;
		u64 Decompression : 1;
		u64 Encryption : 1;
		u64 Decryption : 1;
		u64 RAID : 1;
		u64 EC : 1;
		u64 Dedup : 1;
		u64 Hash : 1;
		u64 Checksum : 1;
		u64 RegEx : 1;
		u64 DbFilter : 1;
		u64 ImageEncode : 1;
		u64 VideoEncode : 1;
		u64 CustomType : 48;
	} Functions;
} CsCapabilities;

typedef CsCapabilities CsFunctionBitSelect;

typedef enum {
	CS_AFDM_TYPE = 1,
	CS_32BIT_VALUE_TYPE = 2,
	CS_64BIT_VALUE_TYPE = 3,
	// https://stackoverflow.com/questions/35380279/avoid-name-collisions-with-enum-in-c-c99
	// Added _ to avoid collision with struct CS_STREAM_TYPE below
	CS_STREAM_TYPE_ = 4,
	CS_DESCRIPTOR_TYPE = 5
} CS_COMPUTE_ARG_TYPE;

typedef struct {
	void *MemHandle;  // an opaque memory handle for AFDM
	unsigned long ByteOffset; // denotes the offset with AFDM
} __attribute__((packed)) CsDevAFDM;

typedef void* CS_STREAM_HANDLE;
typedef s32 CS_CSE_HANDLE;
typedef u32 CS_FUNCTION_ID;

typedef struct {
	CS_COMPUTE_ARG_TYPE Type;
	union {
		CsDevAFDM DevMem;  // see 6.3.4.2.1
		u64 Value64;
		u32 Value32;
		CS_STREAM_HANDLE StreamHandle;
	} u;
} __attribute__((packed)) CsComputeArg;

typedef struct {
	CS_CSE_HANDLE CSEHandle;
	CS_FUNCTION_ID FunctionId;
	int NumArgs;               // set to total arguments to CSF
	CsComputeArg Args[1];      // see 6.3.4.2.6
			       // allocate enough space past this for multiple
			       // arguments
} __attribute__((packed)) CsComputeRequest;

typedef enum {
	TSP_CS_IDENTIFY = 0,
	TSP_CS_GET = 8,
	TSP_CS_ALLOCATE = 16,
	TSP_CS_DEALLOCATE = 17,
	TSP_CS_COMPUTE = 32,
} TSP_CDW10;

typedef enum {
	TSP_CS_CSX = 0,
	TSP_CS_PROPS = 8,
	TSP_CS_CAPS = 16,
	TSP_CS_FUN = 32,
	TSP_CS_MEM = 64,
	TSP_CS_COMM = 64,
	TSP_CS_OPEN_RELAY = 128,
	TSP_CS_CLOSE_RELAY = 129,
} TSP_CDW11;

enum {
	TSP_CHECKSUM_FUNCTION_ID = 42,
	TSP_SLEEP_FUNCTION_ID = 100,
};

typedef struct AsyncComputeRequest {
	struct pci_epf_nvme *nvme;
	struct pci_epf_nvme_cmd *epcmd;
	CsComputeRequest *creq;
	size_t creq_size;
} AsyncComputeRequest;

#endif /* __TSP_CS_H__ */

static CSEProperties tsp_cse_props = {
	.HwVersion = 0x1,
	.SwVersion = 0x1,
	.UniqueName = "CSE Number 1",
	.NumBuiltinFunctions = 1,
	.MaxRequestsPerBatch = 1,
	.MaxFunctionParametersAllows = 4,
	.MaxConcurrentFunctionInstances = 1,
};

static CSxProperties tsp_csx_props = {
	.HwVersion = 0x1,
	.SwVersion = 0x1,
	.VendorId = 0x42,
	.DeviceId = 0x1337,
	.FriendlyName = "Friendly CSX",
	.CFMinMB = 0,
	.FDMinMB = 0,
	.Flags.FDMIsDeviceManaged = 1,
	.Flags.FDMIsHostVisible = 0,
	.Flags.BatchRequestsSupported = 0,
	.Flags.StreamsSupported = 0,
	.Flags.Reserved = 0,
	.NumCSEs = 1,
};

static CsCapabilities tsp_cs_caps = {
	.Functions.Compression = 0,
	.Functions.Decompression = 0,
	.Functions.Encryption = 0,
	.Functions.Decryption = 0,
	.Functions.RAID = 0,
	.Functions.EC = 0,
	.Functions.Dedup = 0,
	.Functions.Hash = 0,
	.Functions.Checksum = 1,
	.Functions.RegEx = 0,
	.Functions.DbFilter = 0,
	.Functions.ImageEncode = 0,
	.Functions.VideoEncode = 0,
	.Functions.CustomType = 0,
};

static const char *TSP_CS_ID_STRING = "This device has compute";

/* We could have more chardevs to userspace */
#define MAX_IO_CDEVS 4
#define MAX_ADMIN_CDEVS 4
#define MAX_DEV (MAX_IO_CDEVS + MAX_ADMIN_CDEVS)

static DEFINE_SPINLOCK(touser_wr_lk);
static DEFINE_SPINLOCK(touser_rd_lk);
static DECLARE_KFIFO(touser_fifo, typeof(struct pci_epf_nvme_cmd *),
		     PCI_EPF_NVME_FIFO_SIZE);
DECLARE_WAIT_QUEUE_HEAD(touser_wq);
static DEFINE_SPINLOCK(touser_admin_wr_lk);
static DEFINE_SPINLOCK(touser_admin_rd_lk);
static DECLARE_KFIFO(touser_admin_fifo, typeof(struct pci_epf_nvme_cmd *),
		     PCI_EPF_NVME_FIFO_SIZE);
DECLARE_WAIT_QUEUE_HEAD(touser_admin_wq);


/*
 * Maximum data transfer size: limit to 128 KB to avoid excessive local
 * memory use for buffers.
 */
#define PCI_EPF_NVME_MDTS		(128 * 1024)
static int dev_major = 0;
/* PRP manipulation macros */
#define pci_epf_nvme_prp_addr(ctrl, prp)	((prp) & ~(ctrl)->mps_mask)
#define pci_epf_nvme_prp_ofst(ctrl, prp)	((prp) & (ctrl)->mps_mask)
#define pci_epf_nvme_prp_size(ctrl, prp)	\
	((size_t)((ctrl)->mps - pci_epf_nvme_prp_ofst(ctrl, prp)))

static struct workqueue_struct *epf_nvme_reg_wq;
static struct workqueue_struct *epf_nvme_sq_wq;
static struct kmem_cache *epf_nvme_cmd_cache;
static int num_xfer_threads = 1;
DECLARE_WAIT_QUEUE_HEAD(xfer_wq);
DECLARE_WAIT_QUEUE_HEAD(cq_wq);

static bool do_readwrite = true;
static bool do_transfer = true;
static bool do_backend = true;
static unsigned relaxed_poll_threshold = 100;
static unsigned poll_delay_usecs = PCI_EPF_NVME_MIN_POLL_DELAY_US;
static unsigned poll_relaxed_delay_usecs = PCI_EPF_NVME_MIN_POLL_DELAY_US * 100;

#define PCI_EPF_NVME_MAX_STATS 1000
static unsigned collected_read_stats = 0;
static unsigned collected_write_stats = 0;
/* Allows to filter on buffer size, allows to ignore spurious system reads */
size_t collect_size_filter = 0;

#define MAX_PATH_LEN	8

enum destination {
	pci_epf_nvme_to_complete = 0,
	pci_epf_nvme_to_transfer,
	pci_epf_nvme_to_backend,
	pci_epf_nvme_to_read_relay,
	pci_epf_nvme_to_user,
};

/*
 * Host PCI memory segment for admin and IO commands.
 */
struct pci_epf_nvme_segment {
	phys_addr_t	pci_addr;
	size_t		size;
};

/*
 * Queue definition and mapping for a local PCI controller.
 */
struct pci_epf_nvme_queue {
	int			ref;

	u16			qid;
	u16			cqid;
	u16			size;
	u16			depth;
	u16			flags;
	u16			vector;
	u16			head;
	u16			tail;
	u16			phase;
	u32			db;

	size_t			qes;

	phys_addr_t		pci_addr;
	struct pci_epc_map	map;
	bool			mapped;

	struct nvme_command	*local_queue;
	u16			local_tail;
};

struct pci_epf_nvme;

/*
 * Local PCI controller exposed with an endpoint function.
 */
struct pci_epf_nvme_ctrl {
	/* Backing fabrics host controller */
	struct nvme_ctrl		*ctrl;

	/* Registers of the local PCI controller */
	void				*reg;
	u64				cap;
	u32				vs;
	u32				cc;
	u32				csts;
	u32				aqa;
	u64				asq;
	u64				acq;
	u32				cmbloc;
	u32				cmbsz;

	size_t				adm_sqes;
	size_t				adm_cqes;
	size_t				io_sqes;
	size_t				io_cqes;

	size_t				mps_shift;
	size_t				mps;
	size_t				mps_mask;

	unsigned int			nr_queues;
	struct pci_epf_nvme_queue	*sq;
	struct pci_epf_nvme_queue	*cq;
};

/*
 * Command flags.
 */
#define PCI_EPF_NVME_CMD_ASYNC		(1LU << 0)

/*
 * DMA fields
 */
struct pci_epf_nvme_dma {
	bool				dma_supported;
	bool				dma_private;
	struct dma_chan			*dma_chan_tx;
	struct dma_chan			*dma_chan_rx;
	struct dma_chan			*dma_chan;
	struct device			*dma_dev;
	dma_cookie_t			dma_cookie;
	enum dma_status			dma_status;
	struct completion		dma_complete;
};

/*
 * Transfer thread structure, can have a DMA
 */
struct pci_epf_nvme_xfer_thread {
	struct pci_epf_nvme		*epf_nvme;
	struct task_struct		*thread;
	struct pci_epf_nvme_dma		dma;
	int				tid;
	__le64				*prp_list_buf;
};

struct pci_epf_nvme_timestamps {
	u64				creation;
	u64				xfer_start;
	u64				xfer_prp;
	u64				xfer_end;
	u64				backend_start;
	u64				backend_end;
	u64				completion_start;
	u64				completion;
};

#define STATS_BUFFER_SIZE (sizeof(struct pci_epf_nvme_timestamps) * PCI_EPF_NVME_MAX_STATS)
static struct pci_epf_nvme_timestamps *read_stats = NULL;
static struct pci_epf_nvme_timestamps *write_stats = NULL;

/*
 * Descriptor for commands sent by the host. This is also used internally for
 * fabrics commands to control our fabrics target.
 */
struct pci_epf_nvme_cmd {
	struct pci_epf_nvme		*epf_nvme;
	struct pci_epf_nvme_xfer_thread *xfer_thread;
	unsigned long			flags;

	struct nvme_ns			*ns;

	int				sqid;
	int				cqid;
	unsigned int			status;
	struct nvme_command 		cmd;
	struct nvme_completion		cqe;

	enum dma_data_direction		dir;
	int				next_destination;
	__u8				path[MAX_PATH_LEN];
	size_t				transfer_len;

	/* Compute thread */
	struct task_struct		*thread;
	/* Relay */
	int				relay_desc;

	/* Internal buffer that we will transfer over PCI */
	size_t				buffer_size;
	void				*buffer;

	/*
	 * Host PCI adress segments: if nr_segs is 1, we use only "seg",
	 * otherwise, the segs array is allocated and used to store
	 * multiple segments.
	 */
	unsigned int			nr_segs;
	struct pci_epf_nvme_segment	seg;
	struct pci_epf_nvme_segment	*segs;

	struct pci_epf_nvme_timestamps	ts;
};

struct tsp_file_data {
	struct pci_epf_nvme		*epf_nvme;
	struct pci_epf_nvme_cmd 	*epcmd;
	struct cdev			cdev;
	struct mutex			mutex;
};

/*
 * EPF function private data representing our NVMe subsystem.
 */
struct pci_epf_nvme {
	struct pci_epf			*epf;
	const struct pci_epc_features	*epc_features;

	void				*reg[PCI_STD_NUM_BARS];
	enum pci_barno			reg_bar;
	size_t				msix_table_offset;

	unsigned int			irq_type;
	unsigned int			nr_vectors;

	struct delayed_work		reg_poll;
	struct delayed_work		sq_poll;

	struct pci_epf_nvme_xfer_thread sq_xfer_info;
	struct pci_epf_nvme_xfer_thread *xfer_threads;
	spinlock_t			xfer_fifo_wr_lock;
	spinlock_t			xfer_fifo_rd_lock;
	DECLARE_KFIFO_PTR(xfer_fifo, typeof(struct pci_epf_nvme_cmd *));
	struct task_struct		*cq_thread;
	spinlock_t			completion_fifo_wr_lock;
	DECLARE_KFIFO_PTR(completion_fifo, typeof(struct pci_epf_nvme_cmd *));

	atomic_t			in_flight_commands;
	bool				disabled;

	unsigned int			max_nr_queues;

	struct pci_epf_nvme_ctrl	ctrl;
	struct pci_epf_nvme_queue	*currently_mapped_sq;
	struct pci_epf_nvme_queue	*currently_mapped_cq;

	/* Function configfs attributes */
	struct config_group		group;
	char				*ctrl_opts_buf;
	bool				dma_enable;

	/* Towards Storage Processing (TSP) */
	struct uio_info			uio_info;
	struct tsp_file_data		tsp_user_queue[MAX_IO_CDEVS];
	struct tsp_file_data		tsp_admin_queue[MAX_ADMIN_CDEVS];
	struct class 			*tsp_class;
	bool				user_path_enable;
	void				*tsp_alloc_ctx;
	struct tsp_relay		*tsp_relays[TSP_RELAY_MAX_RELAYS];
};

/*
 * Read a 32-bits BAR register (equivalent to readl()).
 */
static inline u32 pci_epf_nvme_reg_read32(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	return le32_to_cpu(*ctrl_reg);
}

/*
 * Write a 32-bits BAR register (equivalent to readl()).
 */
static inline void pci_epf_nvme_reg_write32(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u32 val)
{
	volatile __le32 *ctrl_reg = ctrl->reg + reg;

	*ctrl_reg = cpu_to_le32(val);
}

/*
 * Read a 64-bits BAR register (equivalent to lo_hi_readq()).
 */
static inline u64 pci_epf_nvme_reg_read64(struct pci_epf_nvme_ctrl *ctrl,
					  u32 reg)
{
	return (u64)pci_epf_nvme_reg_read32(ctrl, reg) |
		((u64)pci_epf_nvme_reg_read32(ctrl, reg + 4) << 32);
}

/*
 * Write a 64-bits BAR register (equivalent to lo_hi_writeq()).
 */
static inline void pci_epf_nvme_reg_write64(struct pci_epf_nvme_ctrl *ctrl,
					    u32 reg, u64 val)
{
	pci_epf_nvme_reg_write32(ctrl, reg, val & 0xFFFFFFFF);
	pci_epf_nvme_reg_write32(ctrl, reg + 4, (val >> 32) & 0xFFFFFFFF);
}

struct pci_epf_nvme_dma_filter {
        struct device *dev;
        u32 dma_mask;
};

static bool pci_epf_nvme_dma_filter(struct dma_chan *chan, void *arg)
{
        struct pci_epf_nvme_dma_filter *filter = arg;
        struct dma_slave_caps caps;

        memset(&caps, 0, sizeof(caps));
        dma_get_slave_caps(chan, &caps);

        return chan->device->dev == filter->dev &&
               (filter->dma_mask & caps.directions);
}

static bool pci_epf_nvme_init_dma(struct pci_epf_nvme *epf_nvme,
				  struct pci_epf_nvme_dma *dma)
{
	struct pci_epf *epf = epf_nvme->epf;
	struct device *dev = &epf->dev;
	struct pci_epf_nvme_dma_filter filter;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int ret;

	dma->dma_dev = epf_nvme->epf->epc->dev.parent;
	init_completion(&dma->dma_complete);

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	filter.dev = dma->dma_dev;
	filter.dma_mask = BIT(DMA_DEV_TO_MEM);
	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan)
		goto generic;
	dma->dma_chan_rx = chan;

	filter.dma_mask = BIT(DMA_MEM_TO_DEV);
	chan = dma_request_channel(mask, pci_epf_nvme_dma_filter, &filter);
	if (!chan)
		goto release_rx;
	dma->dma_chan_tx = chan;

	dev_info(dev, "DMA RX channel %s: maximum segment size %d B\n",
		 dma_chan_name(dma->dma_chan_rx),
		 dma_get_max_seg_size(dma->dma_chan_rx->device->dev));
	dev_info(dev, "DMA TX channel %s: maximum segment size %d B\n",
		 dma_chan_name(dma->dma_chan_tx),
		 dma_get_max_seg_size(dma->dma_chan_tx->device->dev));

	dma->dma_private = true;

	return true;

release_rx:
	dma_release_channel(dma->dma_chan_rx);
	dma->dma_chan_rx = NULL;

generic:
	/* Fallback to a generic memcpy channel if we have one */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(chan)) {
		ret = PTR_ERR(chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get generic DMA channel\n");
		return false;
	}

	dev_info(dev, "Generic DMA channel %s: maximum segment size %d B\n",
		 dma_chan_name(chan),
		 dma_get_max_seg_size(chan->device->dev));

	dma->dma_chan_tx = chan;
	dma->dma_chan_rx = chan;

	return true;
}

static void pci_epf_nvme_clean_dma(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_dma *dma)
{
	if (!dma->dma_supported)
		return;

	dma_release_channel(dma->dma_chan_tx);
	if (dma->dma_chan_rx != dma->dma_chan_tx)
		dma_release_channel(dma->dma_chan_rx);

	dma->dma_chan_tx = NULL;
	dma->dma_chan_rx = NULL;
	dma->dma_chan = NULL;
	dma->dma_supported = false;
}

static void pci_epf_nvme_dma_callback(void *param)
{
        struct pci_epf_nvme_dma *dma = param;
        struct dma_tx_state state;
	enum dma_status status;

        status = dmaengine_tx_status(dma->dma_chan,
				     dma->dma_cookie, &state);
	if (status == DMA_COMPLETE || status == DMA_ERROR) {
		dma->dma_status = status;
		complete(&dma->dma_complete);
	}
}

static int pci_epf_nvme_do_dma(struct pci_epf_nvme *epf_nvme,
			      struct pci_epf_nvme_dma *dma,
			      dma_addr_t dma_dst, dma_addr_t dma_src,
			      size_t len, dma_addr_t dma_remote,
			      enum dma_transfer_direction dir)
{
	struct dma_async_tx_descriptor *tx;
	struct dma_slave_config sconf = {};
	unsigned long time_left;
	struct dma_chan *chan;
	dma_addr_t dma_local;
	int ret;

	if (dir == DMA_DEV_TO_MEM) {
		chan = dma->dma_chan_tx;
		dma_local = dma_dst;
	} else {
		dma_local = dma_src;
		chan = dma->dma_chan_rx;
	}
	if (IS_ERR_OR_NULL(chan)) {
		dev_err(&epf_nvme->epf->dev, "Invalid DMA channel\n");
		return -EINVAL;
	}

	if (dma->dma_private) {
		sconf.direction = dir;
		if (dir == DMA_MEM_TO_DEV)
			sconf.dst_addr = dma_remote;
		else
			sconf.src_addr = dma_remote;

		if (dmaengine_slave_config(chan, &sconf)) {
			dev_err(&epf_nvme->epf->dev,
				"DMA slave config failed\n");
			return -EIO;
		}

		tx = dmaengine_prep_slave_single(chan, dma_local, len, dir,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	} else {
		tx = dmaengine_prep_dma_memcpy(chan, dma_dst, dma_src, len,
					DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	}
	if (!tx) {
		dev_err(&epf_nvme->epf->dev, "Prepare DMA memcpy failed\n");
		return -EIO;
	}

	reinit_completion(&dma->dma_complete);
	dma->dma_chan = chan;
	tx->callback = pci_epf_nvme_dma_callback;
	tx->callback_param = dma;
	dma->dma_cookie = dmaengine_submit(tx);

	ret = dma_submit_error(dma->dma_cookie);
	if (ret) {
		dev_err(&epf_nvme->epf->dev, "DMA tx_submit failed %d\n", ret);
		goto terminate;
	}

	dma_async_issue_pending(chan);

	time_left = wait_for_completion_timeout(&dma->dma_complete,
						HZ * 10);
	if (!time_left) {
		dev_err(&epf_nvme->epf->dev, "DMA transfer timeout\n");
		ret = -ETIMEDOUT;
		goto terminate;
	}

	if (dma->dma_status != DMA_COMPLETE) {
		dev_err(&epf_nvme->epf->dev, "DMA transfer failed\n");
		ret = -EIO;
	}

terminate:
	if (ret)
		dmaengine_terminate_sync(chan);

	return ret;
}

static int pci_epf_nvme_dma_transfer(struct pci_epf_nvme *epf_nvme,
				     struct pci_epf_nvme_dma *dma,
				     struct pci_epc_map *map,
				     void *buf, enum dma_data_direction dir)
{
	phys_addr_t dma_phys_addr;
	int ret;

	dma_phys_addr = dma_map_single(dma->dma_dev, buf, map->size, dir);
	if (dma_mapping_error(dma->dma_dev, dma_phys_addr)) {
		dev_err(&epf_nvme->epf->dev,
			"Failed to map source buffer addr\n");
		return -ENOMEM;
	}

	switch (dir) {
	case DMA_FROM_DEVICE:
		ret = pci_epf_nvme_do_dma(epf_nvme, dma, dma_phys_addr,
					  map->phys_addr, map->size,
					  map->pci_addr, DMA_DEV_TO_MEM);
		break;
	case DMA_TO_DEVICE:
		ret = pci_epf_nvme_do_dma(epf_nvme, dma, map->phys_addr,
					  dma_phys_addr, map->size,
					  map->pci_addr, DMA_MEM_TO_DEV);
		break;
	default:
		ret = -EINVAL;
	}

	dma_unmap_single(dma->dma_dev, dma_phys_addr, map->size, dir);

	return ret;
}

static int pci_epf_nvme_mmio_transfer(struct pci_epf_nvme *nvme,
				      struct pci_epc_map *map,
				      void *buf, enum dma_data_direction dir)
{
	switch (dir) {
	case DMA_FROM_DEVICE:
		memcpy_fromio(buf, map->virt_addr, map->size);
		return 0;
	case DMA_TO_DEVICE:
		memcpy_toio(map->virt_addr, buf, map->size);
		return 0;
	default:
		return -EINVAL;
	}
}

static void pci_epf_nvme_print_cmb_info(struct pci_epf_nvme *epf_nvme)
{
	u64 cmbmsc;
	phys_addr_t cba;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	dev_info(&epf_nvme->epf->dev,
		 "CMB phys addr: %#0llx end: %#0llx CMSE %sabled CRE %sabled\n",
		 cba, cba + CMB_SIZE,
		 ((cmbmsc & CMBMSC_CMSE_BIT) ? "en" : "dis"),
		 ((cmbmsc & CMBMSC_CRE_BIT) ? "en" : "dis"));
}

static int pci_epf_nvme_addr_in_cmb(struct pci_epf_nvme *epf_nvme,
				    phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	/* If Controller Memory Space Enabled and Capabilities Register Enabled */
	if (!!(cmbmsc & CMBMSC_CMSE_BIT) && !!(cmbmsc & CMBMSC_CRE_BIT)) {
		/* If the PCI addr is in the CMB */
		if (addr >= cba && addr < (cba + CMB_SIZE)) {
			return true;
		}
	}
	return false;
}

/* The address must be in the CMB, there is no check here */
static void *pci_epf_nvme_cmb_addr_as_local_addr(struct pci_epf_nvme *epf_nvme,
						 phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba, offset;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;
	offset = addr - cba;

	return (char*)(epf_nvme->reg[CMB_BIR]) + offset;
}

static int pci_epf_nvme_segment_in_cmb(struct pci_epf_nvme *epf_nvme,
				       struct pci_epf_nvme_segment *seg)
{
	return pci_epf_nvme_addr_in_cmb(epf_nvme, seg->pci_addr);
}

static int pci_epf_nvme_map_cmb_segment(struct pci_epf_nvme *epf_nvme,
					struct pci_epf_nvme_segment *seg,
					struct pci_epc_map *map)
{
	u64 cmbmsc;
	phys_addr_t cba, offset;
	const size_t CMB_SIZE = epf_nvme->epf->bar[CMB_BIR].size;

	if (!seg | !map | !seg->size)
		return -EINVAL;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;
	offset = seg->pci_addr - cba;

	if (seg->pci_addr < cba ||
	    seg->pci_addr > (cba + CMB_SIZE)) {
		dev_err(&epf_nvme->epf->dev, "Segment not in CMB");
		return -EINVAL;
	}

	if (seg->pci_addr + seg->size > cba + CMB_SIZE) {
		dev_err(&epf_nvme->epf->dev, "Segment overflows CMB\n");
		return -ENOMEM;
	}

	map->pci_addr = seg->pci_addr;
	map->phys_size = seg->size;
	map->phys_addr = epf_nvme->epf->bar[CMB_BIR].phys_addr + offset;
	map->virt_addr = (char*)(epf_nvme->reg[CMB_BIR]) + offset;

	return 0;
}

static int pci_epf_nvme_transfer(struct pci_epf_nvme_xfer_thread *xfer_thread,
				 struct pci_epf_nvme_segment *seg,
				 enum dma_data_direction dir, void *buf,
				 bool no_dma)
{
	struct pci_epf_nvme *epf_nvme = xfer_thread->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	phys_addr_t addr = seg->pci_addr;
	size_t size = seg->size;
	struct pci_epc_map map;
	ssize_t map_size;
	bool in_cmb = pci_epf_nvme_segment_in_cmb(epf_nvme, seg);
	int ret;

	while (size) {

		/* If we have to read/write our own CMB we do not do it through
		 * PCI so the map is different based on CMB/PCI */
		if (in_cmb) {
			ret = pci_epf_nvme_map_cmb_segment(epf_nvme, seg, &map);
			if (ret)
				return ret;
			map_size = seg->size;
		} else {
			map_size = pci_epf_mem_map(epf, addr, size, &map);
			if (map_size < 0)
				return map_size;
		}

		/* Do not bother with DMA for small transfers */
		if (no_dma || !xfer_thread->dma.dma_supported ||
		    map.size < ctrl->mps)
			ret = pci_epf_nvme_mmio_transfer(epf_nvme, &map,
							 buf, dir);
		else
			ret = pci_epf_nvme_dma_transfer(epf_nvme,
							&xfer_thread->dma,
							&map, buf, dir);

		if (!in_cmb)
			pci_epf_mem_unmap(epf, &map);

		if (ret)
			return ret;

		size -= map_size;
		addr += map_size;
		buf += map_size;
	}

	return 0;
}

static inline struct pci_epf_nvme_cmd *
pci_epf_nvme_alloc_cmd(struct pci_epf_nvme *nvme)
{
	return kmem_cache_alloc(epf_nvme_cmd_cache, GFP_KERNEL);
}

static void pci_epf_nvme_init_cmd(struct pci_epf_nvme *epf_nvme,
				  struct pci_epf_nvme_cmd *epcmd,
				  int sqid, int cqid)
{
	atomic_inc(&epf_nvme->in_flight_commands);
	memset(epcmd, 0, sizeof(*epcmd));
	epcmd->epf_nvme = epf_nvme;
	epcmd->sqid = sqid;
	epcmd->cqid = cqid;
	epcmd->status = NVME_SC_SUCCESS;
	epcmd->ts.creation = ktime_get_ns();
	/* next_destination, path, set to 0 by memset */
	/* compute thread and relay info are set to 0 by memset */
}

static int pci_epf_nvme_alloc_cmd_buffer(struct pci_epf_nvme_cmd *epcmd)
{
	void *buffer;

	buffer = kvmalloc(epcmd->transfer_len, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	epcmd->buffer = buffer;
	epcmd->buffer_size = epcmd->transfer_len;

	return 0;
}

static int pci_epf_nvme_alloc_cmd_segs(struct pci_epf_nvme_cmd *epcmd,
				       int nr_segs)
{
	struct pci_epf_nvme_segment *segs;

	/* Single map case: use the command map structure */
	if (nr_segs == 1) {
		epcmd->segs = &epcmd->seg;
		epcmd->nr_segs = 1;
		return 0;
	}

	/* More than one map needed: allocate an array */
	segs = kcalloc(nr_segs, sizeof(struct pci_epf_nvme_segment), GFP_KERNEL);
	if (!segs)
		return -ENOMEM;

	epcmd->nr_segs = nr_segs;
	epcmd->segs = segs;

	return 0;
}

static void pci_epf_nvme_collect_statistics(struct pci_epf_nvme_cmd *epcmd)
{
	if (!epcmd->sqid) /* Ignore admin commands */
		return;

	if (epcmd->cmd.common.opcode == nvme_cmd_read) {
		if (collected_read_stats >= PCI_EPF_NVME_MAX_STATS)
			return;
		if (!collect_size_filter ||
		    epcmd->buffer_size == collect_size_filter)
			memcpy(&read_stats[collected_read_stats++], &epcmd->ts,
			       sizeof(epcmd->ts));
	}

	if (epcmd->cmd.common.opcode == nvme_cmd_write) {
		if (collected_write_stats >= PCI_EPF_NVME_MAX_STATS)
			return;
		if (!collect_size_filter ||
		    epcmd->buffer_size == collect_size_filter)
			memcpy(&write_stats[collected_write_stats++], &epcmd->ts,
			       sizeof(epcmd->ts));
	}
}

static void pci_epf_nvme_free_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->ts.completion = ktime_get_ns();

	pci_epf_nvme_collect_statistics(epcmd);

	atomic_dec_and_test(&epcmd->epf_nvme->in_flight_commands);

	if (epcmd->ns)
		nvme_put_ns(epcmd->ns);

	if (epcmd->buffer)
		kfree(epcmd->buffer);

	if (epcmd->segs && epcmd->segs != &epcmd->seg)
		kfree(epcmd->segs);

	kmem_cache_free(epf_nvme_cmd_cache, epcmd);
}

static const char *pci_epf_nvme_cmd_name(struct pci_epf_nvme_cmd *epcmd)
{
	u8 opcode = epcmd->cmd.common.opcode;

	if (epcmd->sqid)
		return nvme_get_opcode_str(opcode);
	return nvme_get_admin_opcode_str(opcode);
}

static int pci_epf_nvme_cmd_transfer(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme_xfer_thread *xfer_thread = epcmd->xfer_thread;
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_segment *seg;
	void *buf = epcmd->buffer;
	enum dma_data_direction dir = epcmd->dir;
	size_t size = 0;
	int i, ret;

	/* Do nothing for commands already marked as failed */
	if (epcmd->status != NVME_SC_SUCCESS)
		return -EIO;

	/* If the transfer length or direction is not defined */
	if (!epcmd->transfer_len || dir == DMA_NONE ||
	    dir == DMA_BIDIRECTIONAL) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		return -EIO;
	}

	/* Go through the command segments and transfer each one */
	for (i = 0; i < epcmd->nr_segs; i++) {
		seg = &epcmd->segs[i];

		if (size >= epcmd->buffer_size) {
			dev_err(&epf_nvme->epf->dev, "Invalid transfer size\n");
			goto xfer_err;
		}

		ret = pci_epf_nvme_transfer(xfer_thread, seg, dir, buf, false);
		if (ret)
			goto xfer_err;

		buf += seg->size;
		size += seg->size;
	}

	return 0;

xfer_err:
	epcmd->status = NVME_SC_DATA_XFER_ERROR | NVME_SC_DNR;
	return -EIO;
}

static inline void pci_epf_nvme_set_bc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_backend;
	epcmd->path[1] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_xc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_transfer;
	epcmd->path[1] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_xbc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_transfer;
	epcmd->path[1] = pci_epf_nvme_to_backend;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_bxc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_backend;
	epcmd->path[1] = pci_epf_nvme_to_transfer;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_rxc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_read_relay;
	epcmd->path[1] = pci_epf_nvme_to_transfer;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_buxc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_backend;
	epcmd->path[1] = pci_epf_nvme_to_user;
	epcmd->path[2] = pci_epf_nvme_to_transfer;
	epcmd->path[3] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_xubc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_transfer;
	epcmd->path[1] = pci_epf_nvme_to_user;
	epcmd->path[2] = pci_epf_nvme_to_backend;
	epcmd->path[3] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_xuc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_transfer;
	epcmd->path[1] = pci_epf_nvme_to_user;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_uxc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_user;
	epcmd->path[1] = pci_epf_nvme_to_transfer;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_ubc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_user;
	epcmd->path[1] = pci_epf_nvme_to_backend;
	epcmd->path[2] = pci_epf_nvme_to_complete;
}

static inline void pci_epf_nvme_set_uc_path(struct pci_epf_nvme_cmd *epcmd)
{
	epcmd->path[0] = pci_epf_nvme_to_user;
	epcmd->path[1] = pci_epf_nvme_to_complete;
}

static inline
void pci_epf_nvme_send_cmd_to_completion(struct pci_epf_nvme_cmd *epcmd);
static inline
void pci_epf_nvme_send_cmd_to_backend(struct pci_epf_nvme_cmd *epcmd);
static inline
void pci_epf_nvme_send_cmd_to_xfer(struct pci_epf_nvme_cmd *epcmd);
static inline
void tsp_nvme_cmd_to_read_relay(struct pci_epf_nvme_cmd *epcmd);
static inline
void pci_epf_nvme_send_cmd_to_user(struct pci_epf_nvme_cmd *epcmd);

static inline
void pci_epf_nvme_send_cmd_to_next(struct pci_epf_nvme_cmd *epcmd)
{
	switch (epcmd->path[epcmd->next_destination++]) {
	case pci_epf_nvme_to_complete:
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		break;
	case pci_epf_nvme_to_transfer:
		pci_epf_nvme_send_cmd_to_xfer(epcmd);
		break;
	case pci_epf_nvme_to_backend:
		pci_epf_nvme_send_cmd_to_backend(epcmd);
		break;
	case pci_epf_nvme_to_read_relay:
		tsp_nvme_cmd_to_read_relay(epcmd);
		break;
	case pci_epf_nvme_to_user:
		pci_epf_nvme_send_cmd_to_user(epcmd);
		break;
	default:
		epcmd->status = NVME_SC_INTERNAL;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		break;
	}
}

static int pci_epf_nvme_cmd_parse_dptr(struct pci_epf_nvme_cmd *epcmd);

static int pci_epf_nvme_xfer_thread_fn(void *arg)
{
	int ret;
	struct pci_epf_nvme_xfer_thread *xfer_thread = arg;
	struct pci_epf_nvme *epf_nvme = xfer_thread->epf_nvme;
	struct pci_epf_nvme_cmd *epcmd;

	while(1) {
		spin_lock(&epf_nvme->xfer_fifo_rd_lock);
		while (kfifo_is_empty(&epf_nvme->xfer_fifo) &&
		       !epf_nvme->disabled) {
			spin_unlock(&epf_nvme->xfer_fifo_rd_lock);
			ret = wait_event_interruptible(xfer_wq,
				!kfifo_is_empty(&epf_nvme->xfer_fifo) ||
				epf_nvme->disabled);
			spin_lock(&epf_nvme->xfer_fifo_rd_lock);
		}

		if (epf_nvme->disabled) {
			spin_unlock(&epf_nvme->xfer_fifo_rd_lock);
			return 0;
		}
		if (ret) continue; /* signal */

		ret = kfifo_get(&epf_nvme->xfer_fifo, &epcmd);
		spin_unlock(&epf_nvme->xfer_fifo_rd_lock);

		if (ret != 1) {
			dev_err(&epf_nvme->epf->dev,
			        "Could not get element from transfer FIFO\n");
			continue;
		}

		epcmd->ts.xfer_start = ktime_get_ns();

		if (do_transfer) {
			epcmd->xfer_thread = xfer_thread;

			/* Get the host buffer segments */
			ret = pci_epf_nvme_cmd_parse_dptr(epcmd);
			if (ret) {
				pci_epf_nvme_send_cmd_to_completion(epcmd);
				continue;
			}

			epcmd->ts.xfer_prp = ktime_get_ns();

			ret = pci_epf_nvme_cmd_transfer(epcmd);
			if (ret) {
				pci_epf_nvme_send_cmd_to_completion(epcmd);
				continue;
			}
		}

		epcmd->ts.xfer_end = ktime_get_ns();

		pci_epf_nvme_send_cmd_to_next(epcmd);
	}
}

static void pci_epf_nvme_raise_irq(struct pci_epf_nvme *epf_nvme,
				   struct pci_epf_nvme_queue *cq)
{
	struct pci_epf *epf = epf_nvme->epf;
	int ret;

	if (!(cq->flags & NVME_CQ_IRQ_ENABLED))
		return;

	switch (epf_nvme->irq_type) {
	case PCI_IRQ_MSIX:
	case PCI_IRQ_MSI:
		ret = pci_epf_raise_irq(epf, epf_nvme->irq_type,
					cq->vector + 1);
		if (!ret)
			return;
		/*
		 * If we got an error, it is likely because the host is using
		 * legacy IRQs (e.g. BIOS, grub), so fallthrough.
		 */
		fallthrough;
	case PCI_IRQ_LEGACY:
		ret = pci_epf_raise_irq(epf, PCI_IRQ_LEGACY, 0);
		if (!ret)
			return;
		break;
	default:
		WARN_ON_ONCE(1);
		ret = -EINVAL;
		break;
	}

	if (ret)
		dev_err(&epf->dev, "Raise IRQ failed %d\n", ret);
}

static inline bool pci_epf_nvme_ctrl_ready(struct pci_epf_nvme_ctrl *ctrl)
{
	return (ctrl->cc & NVME_CC_ENABLE) && (ctrl->csts & NVME_CSTS_RDY);
}

static inline void pci_epf_nvme_unmap_q(struct pci_epf_nvme *epf_nvme,
					struct pci_epf_nvme_queue *q)
{
	if (q->mapped) {
		pci_epf_mem_unmap(epf_nvme->epf, &q->map);
		q->mapped = false;
	}
}

static inline int pci_epf_nvme_map_q(struct pci_epf_nvme *epf_nvme,
				     struct pci_epf_nvme_queue *q)
{
	struct pci_epf *epf = epf_nvme->epf;
	size_t qsize = q->qes * q->depth;
	int ret;

	if (q->mapped)
		return 0;

	ret = pci_epf_mem_map(epf, q->pci_addr, qsize, &q->map);
	if (ret != qsize) {
		if (ret > 0) {
			dev_err(&epf->dev, "Partial queue %d mapping\n", q->qid);
			pci_epf_mem_unmap(epf, &q->map);
			ret = -ENOMEM;
		} else {
			dev_err(&epf->dev, "Map queue %d failed\n", q->qid);
		}
		return ret;
	}

	q->mapped = true;

	dev_dbg(&epf->dev,
		"Queue %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		q->qid, q->map.pci_addr, (u64)q->map.virt_addr, qsize);

	return 0;
}

static inline void pci_epf_nvme_unmap_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];
	if (epf_nvme->currently_mapped_sq == sq)
		epf_nvme->currently_mapped_sq = NULL;
	pci_epf_nvme_unmap_q(epf_nvme, sq);
}

static inline int pci_epf_nvme_map_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];
	int ret = 0;
	if (epf_nvme->currently_mapped_sq != sq) {
		if (epf_nvme->currently_mapped_sq)
			pci_epf_nvme_unmap_q(epf_nvme,
				             epf_nvme->currently_mapped_sq);
		ret = pci_epf_nvme_map_q(epf_nvme, sq);
		if (ret)
			epf_nvme->currently_mapped_sq = NULL;
		else
			epf_nvme->currently_mapped_sq = sq;
	}
	return ret;
}

static inline void pci_epf_nvme_unmap_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];
	if (epf_nvme->currently_mapped_cq == cq)
		epf_nvme->currently_mapped_cq = NULL;
	pci_epf_nvme_unmap_q(epf_nvme, cq);
}

static inline int pci_epf_nvme_map_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];
	int ret = 0;
	if (epf_nvme->currently_mapped_cq != cq) {
		if (epf_nvme->currently_mapped_cq)
			pci_epf_nvme_unmap_q(epf_nvme,
				             epf_nvme->currently_mapped_cq);
		ret = pci_epf_nvme_map_q(epf_nvme, cq);
		if (ret)
			epf_nvme->currently_mapped_cq = NULL;
		else
			epf_nvme->currently_mapped_cq = cq;
	}
	return ret;
}

static inline void __pci_epf_nvme_queue_response(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf *epf = epf_nvme->epf;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[epcmd->sqid];
	struct pci_epf_nvme_queue *cq = &ctrl->cq[epcmd->cqid];
	struct nvme_completion *cqe = &epcmd->cqe;

	epcmd->ts.completion_start = ktime_get_ns();

	/*
	 * Do not try to complete commands if the controller is not ready
	 * anymore, e.g. after the host cleared CC.EN.
	 */
	if (!pci_epf_nvme_ctrl_ready(ctrl))
		goto free;

	cq->head = pci_epf_nvme_reg_read32(ctrl, cq->db);

	/* Check completion queue full state */
	while (((cq->head + 1) == cq->tail) ||
		(((cq->head + 1) == cq->depth) && (cq->tail == 0))) {
		udelay(PCI_EPF_NVME_CQ_FULL_DELAY_US);
		cq->head = pci_epf_nvme_reg_read32(ctrl, cq->db);
	}

	/* Setup the completion entry */
	cqe->sq_id = cpu_to_le16(epcmd->sqid);
	cqe->sq_head = cpu_to_le16(sq->head);
	cqe->command_id = epcmd->cmd.common.command_id;
	cqe->status = cpu_to_le16((epcmd->status << 1) | cq->phase);

	/* Post the completion entry */
	dev_dbg(&epf->dev, "cq[%d]: status 0x%x, phase %d, tail %d -> %d/%d\n",
		epcmd->cqid, epcmd->status, cq->phase, cq->tail,
		(int)cq->tail, (int)cq->depth);

	/* This should not happen, the map has been tested during queue
		initialization, so if map fails it means the PCI controller
		either doesn't work anymore, and here we can't do anyhting
		about it, either to many window mappings are in use and this
		would be the result of developper error in this driver */
	WARN_ON_ONCE(pci_epf_nvme_map_cq(epf_nvme, epcmd->cqid));
	memcpy_toio(cq->map.virt_addr + cq->tail * cq->qes, cqe,
		    sizeof(struct nvme_completion));

	/* Advance cq tail */
	cq->tail++;
	if (cq->tail >= cq->depth) {
		cq->tail = 0;
		cq->phase ^= 1;
	}

	//epcmd->ts.irq_start = ktime_get_ns();

	pci_epf_nvme_raise_irq(epf_nvme, cq);

	if (epcmd->status)
		dev_err(&epcmd->epf_nvme->epf->dev,
			"QID %d: command %s (0x%x) failed, status 0x%0x\n",
			epcmd->cqid, pci_epf_nvme_cmd_name(epcmd),
			epcmd->cmd.common.opcode, epcmd->status);

free:
	pci_epf_nvme_free_cmd(epcmd);
}

static int pci_epf_nvme_completion_thread_fn(void *arg)
{
	int ret;
	struct pci_epf_nvme *epf_nvme = (struct pci_epf_nvme *)arg;
	struct pci_epf_nvme_cmd *epcmd;

	while(1) {
		ret = wait_event_interruptible(cq_wq,
			!kfifo_is_empty(&epf_nvme->completion_fifo) ||
			epf_nvme->disabled);
		if (epf_nvme->disabled)
			return 0;

		if (ret) continue; /* signal */

		ret = kfifo_get(&epf_nvme->completion_fifo, &epcmd);
		if (ret != 1) {
			dev_err(&epf_nvme->epf->dev,
			        "Could not get element from completion FIFO\n");
		}

		__pci_epf_nvme_queue_response(epcmd);
	}
}

static inline
void pci_epf_nvme_send_cmd_to_completion(struct pci_epf_nvme_cmd *epcmd)
{
	/* XXX TODO Handle full FIFO (unlikely) XXX */

	/* Lock because this is called from multiple threads */
	kfifo_in_spinlocked(&epcmd->epf_nvme->completion_fifo, &epcmd, 1,
			    &epcmd->epf_nvme->completion_fifo_wr_lock);
	wake_up(&cq_wq);
}

static inline int pci_epf_nvme_fetch_sqes(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	int num_cmds;
	int ret;

	if (!sq->size || !pci_epf_nvme_ctrl_ready(ctrl))
		return 0;

	sq->tail = pci_epf_nvme_reg_read32(ctrl, sq->db);
	if (sq->tail == sq->head) {
		/* Queue empty */
		return 0;
	}

	ret = pci_epf_nvme_map_sq(epf_nvme, qid);
	if (ret) {
		dev_err(&epf_nvme->epf->dev, "Cannot map SQ id: %d\n", qid);
		return 0;
	}

	/* If the queue has wrapped */
	if (sq->tail < sq->head) {
		/* Transfer all SQEs to edge end of the queue without wrap */
		num_cmds = sq->depth - sq->head;
		memcpy_fromio(&sq->local_queue[sq->head],
			      sq->map.virt_addr +
				sq->head * sizeof(struct nvme_command),
			      num_cmds * sizeof(struct nvme_command));
		sq->local_tail = 0;
	} else {
		/* Transfer all SQEs from host queue to local queue */
		num_cmds = sq->tail - sq->head;
		memcpy_fromio(&sq->local_queue[sq->head],
			      sq->map.virt_addr +
				sq->head * sizeof(struct nvme_command),
			      num_cmds * sizeof(struct nvme_command));
		sq->local_tail = sq->tail;
	}

	return num_cmds;
}

/*
 * Transfer a prp list from the host and return the number of prps.
 */
static int pci_epf_nvme_get_prp_list(struct pci_epf_nvme_cmd *epcmd, u64 prp,
				     size_t xfer_len)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_xfer_thread *xfer_thread = epcmd->xfer_thread;
	size_t nr_prps = (xfer_len + ctrl->mps_mask) >> ctrl->mps_shift;
	struct pci_epf_nvme_segment seg;
	int ret;

	/*
	 * Compute the number of PRPs required for the number of bytes to
	 * transfer (xfer_len). If this number overflows the memory page size
	 * with the PRP list pointer specified, only return the space available
	 * in the memory page, the last PRP in there will be a PRP list pointer
	 * to the remaining PRPs.
	 */
	seg.pci_addr = prp;
	seg.size = min(pci_epf_nvme_prp_size(ctrl, prp), nr_prps << 3);
	ret = pci_epf_nvme_transfer(xfer_thread, &seg, DMA_FROM_DEVICE,
				    xfer_thread->prp_list_buf, false /* no_dma */);
	if (ret)
		return ret;

	return seg.size >> 3;
}

static int pci_epf_nvme_cmd_parse_prp_list(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	__le64 *prps = epcmd->xfer_thread->prp_list_buf;
	struct pci_epf_nvme_segment *seg;
	size_t size = 0, ofst, prp_size, xfer_len;
	int nr_segs, nr_prps = 0;
	phys_addr_t pci_addr;
	int i = 0, ret;
	u64 prp;

	/*
	 * Allocate segments for the command: this considers the worst case
	 * scenario where all prps are discontiguous, so get as many segments
	 * as we can have prps. In practice, most of the time, we will have
	 * far less segments than prps.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp1);
	if (!prp)
		goto invalid_field;

	nr_segs = (epcmd->transfer_len + ofst + NVME_CTRL_PAGE_SIZE - 1)
		>> NVME_CTRL_PAGE_SHIFT;

	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret)
		goto internal;

	/* Set the first segment using prp1 */
	seg = &epcmd->segs[0];
	seg->pci_addr = prp;
	seg->size = pci_epf_nvme_prp_size(ctrl, prp);
	ofst = pci_epf_nvme_prp_ofst(ctrl, prp);

	size = seg->size;
	pci_addr = prp + size;
	nr_segs = 1;

	/*
	 * Now build the pci address segments using the prp lists, starting
	 * from prp2.
	 */
	prp = le64_to_cpu(cmd->common.dptr.prp2);
	if (!prp)
		goto invalid_field;

	while (size < epcmd->transfer_len) {
		xfer_len = epcmd->transfer_len - size;

		if (!nr_prps) {
			/* Get the prp list */
			nr_prps = pci_epf_nvme_get_prp_list(epcmd, prp,
							    xfer_len);
			if (nr_prps < 0)
				goto internal;

			i = 0;
			ofst = 0;
		}

		/* Current entry */
		prp = le64_to_cpu(prps[i]);
		if (!prp)
			goto invalid_field;

		/* Did we reach the last prp entry of the list ? */
		if (xfer_len > ctrl->mps && i == nr_prps - 1) {
			/* We need more PRPs: prp is a list pointer */
			nr_prps = 0;
			continue;
		}

		/* Only the first prp is allowed to have an offset */
		if (pci_epf_nvme_prp_ofst(ctrl, prp))
			goto invalid_offset;

		if (prp != pci_addr) {
			/* Discontiguous prp: new segment */
			nr_segs++;
			if (WARN_ON_ONCE(nr_segs > epcmd->nr_segs))
				goto internal;

			seg++;
			seg->pci_addr = prp;
			seg->size = 0;
			pci_addr = prp;
		}

		prp_size = min_t(size_t, ctrl->mps, xfer_len);
		seg->size += prp_size;
		pci_addr += prp_size;
		size += prp_size;

		i++;
	}

	epcmd->nr_segs = nr_segs;
	ret = 0;

	if (size != epcmd->transfer_len) {
		dev_err(&epf_nvme->epf->dev,
			"PRPs transfer length mismatch %zu / %zu\n",
			size, epcmd->transfer_len);
		goto internal;
	}

	return 0;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;
}

static int pci_epf_nvme_cmd_parse_prp_simple(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	int ret, nr_segs = 1;
	u64 prp1, prp2 = 0;
	size_t prp1_size;

	/* prp1 */
	prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	prp1_size = pci_epf_nvme_prp_size(ctrl, prp1);

	/* For commands crossing a page boundary, we should have a valid prp2 */
	if (epcmd->transfer_len > prp1_size) {
		prp2 = le64_to_cpu(cmd->common.dptr.prp2);
		if (!prp2)
			goto invalid_field;
		if (pci_epf_nvme_prp_ofst(ctrl, prp2))
			goto invalid_offset;
		if (prp2 != prp1 + prp1_size)
			nr_segs = 2;
	}

	/* Create segments using the prps */
	ret = pci_epf_nvme_alloc_cmd_segs(epcmd, nr_segs);
	if (ret )
		goto internal;

	epcmd->segs[0].pci_addr = prp1;
	if (nr_segs == 1) {
		epcmd->segs[0].size = epcmd->transfer_len;
	} else {
		epcmd->segs[0].size = prp1_size;
		epcmd->segs[1].pci_addr = prp2;
		epcmd->segs[1].size = epcmd->transfer_len - prp1_size;
	}

	return 0;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

internal:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return ret;
}

static int pci_epf_nvme_cmd_parse_dptr(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct nvme_command *cmd = &epcmd->cmd;
	u64 prp1 = le64_to_cpu(cmd->common.dptr.prp1);
	size_t ofst;
	int ret;

	if (epcmd->transfer_len > PCI_EPF_NVME_MDTS)
		goto invalid_field;

	/* We do not support SGL for now */
	if (cmd->common.flags & NVME_CMD_SGL_ALL)
		goto invalid_field;

	/* Get pci segments for the command using its prps */
	ofst = pci_epf_nvme_prp_ofst(ctrl, prp1);
	if (ofst & 0x3)
		goto invalid_offset;

	if (epcmd->transfer_len + ofst <= NVME_CTRL_PAGE_SIZE * 2)
		ret = pci_epf_nvme_cmd_parse_prp_simple(epcmd);
	else
		ret = pci_epf_nvme_cmd_parse_prp_list(epcmd);
	if (ret)
		return ret;

	return 0;

invalid_field:
	epcmd->status = NVME_SC_INVALID_FIELD | NVME_SC_DNR;
	return -EINVAL;

invalid_offset:
	epcmd->status = NVME_SC_PRP_INVALID_OFFSET | NVME_SC_DNR;
	return -EINVAL;
}

static void pci_epf_nvme_clean_cq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *cq = &epf_nvme->ctrl.cq[qid];

	if (cq->ref < 1)
		return;

	cq->ref--;
	if (cq->ref)
		return;

	if (cq->mapped)
		pci_epf_mem_unmap(epf_nvme->epf, &cq->map);
	if (cq->local_queue) /* Should not be allocated */
		kfree(cq->local_queue);
	memset(cq, 0, sizeof(*cq));
}

static int pci_epf_nvme_init_cq(struct pci_epf_nvme *epf_nvme, int qid,
				int flags, int size, int vector,
				phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *cq = &ctrl->cq[qid];
	struct pci_epf *epf = epf_nvme->epf;
	size_t qsize;
	ssize_t ret;

	/*
	 * Increment the queue ref count: if the queue was already mapped,
	 * we have nothing to do.
	 */
	cq->ref++;
	if (cq->ref > 1)
		return 0;

	/* Setup and map the completion queue */
	cq->qid = qid;
	cq->size = size;
	cq->flags = flags;
	cq->depth = size + 1;
	cq->vector = vector;
	cq->phase = 1;
	cq->db = NVME_REG_DBS + (((qid * 2) + 1) * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, cq->db, 0);

	if (!qid)
		cq->qes = ctrl->adm_cqes;
	else
		cq->qes = ctrl->io_cqes;
	qsize = cq->qes * cq->depth;

	cq->pci_addr = pci_addr;
	cq->mapped = false;

	/* Try to map, to detect unmappable (partially mappable) queues */
	ret = pci_epf_nvme_map_cq(epf_nvme, qid);
	if (ret) {
		memset(cq, 0, sizeof(*cq));
		return ret;
	}
	pci_epf_nvme_unmap_cq(epf_nvme, qid);

	/* Only submission queues use the local queue */
	cq->local_queue = NULL;
	cq->local_tail = 0;

	dev_dbg(&epf->dev,
		"CQ %d: PCI addr 0x%llx, virt addr 0x%llx, size %zu B\n",
		qid, cq->map.pci_addr, (u64)cq->map.virt_addr, qsize);
	dev_dbg(&epf->dev,
		"CQ %d: %d entries of %zu B, vector IRQ %d\n",
		qid, cq->size, cq->qes, (int)cq->vector + 1);

	return 0;
}

static void pci_epf_nvme_clean_sq(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_queue *sq = &epf_nvme->ctrl.sq[qid];

	if (!sq->ref)
		return;

	if (WARN_ON_ONCE(sq->ref != 1))
		return;

	WARN_ON_ONCE(epf_nvme->ctrl.cq[sq->cqid].ref < 1);
	epf_nvme->ctrl.cq[sq->cqid].ref--;

	if (sq->mapped)
		pci_epf_mem_unmap(epf_nvme->epf, &sq->map);
	if (sq->local_queue)
		kfree(sq->local_queue);
	memset(sq, 0, sizeof(*sq));
}

static int pci_epf_nvme_init_sq(struct pci_epf_nvme *epf_nvme, int qid,
			       int cqid, int flags, int size,
			       phys_addr_t pci_addr)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf *epf = epf_nvme->epf;
	ssize_t ret;

	/* Setup and map the submission queue */
	sq->ref = 1;
	sq->qid = qid;
	sq->cqid = cqid;
	sq->size = size;
	sq->flags = flags;
	sq->depth = size + 1;
	sq->db = NVME_REG_DBS + (qid * 2 * sizeof(u32));
	pci_epf_nvme_reg_write32(ctrl, sq->db, 0);

	if (!qid)
		sq->qes = ctrl->adm_sqes;
	else
		sq->qes = ctrl->io_sqes;

	sq->pci_addr = pci_addr;
	sq->mapped = false;
	/* Try to map, to detect unmappable (partially mappable) queues */
	ret = pci_epf_nvme_map_sq(epf_nvme, qid);
	if (ret) {
		memset(sq, 0, sizeof(*sq));
		return ret;
	}
	pci_epf_nvme_unmap_sq(epf_nvme, qid);

	sq->local_queue = kmalloc(sq->depth * sizeof(struct nvme_command),
				  GFP_KERNEL);
	sq->local_tail = 0;

	if (!sq->local_queue) {
		dev_err(&epf->dev, "Couldn't allocate space for local queue\n");
		memset(sq, 0, sizeof(*sq));
		ret = -ENOMEM;
	}

	/* Get a reference on the completion queue */
	epf_nvme->ctrl.cq[cqid].ref++;

	dev_dbg(&epf->dev,
		"SQ %d: %d queue entries of %zu B, CQ %d\n",
		qid, size, sq->qes, cqid);

	return 0;
}

static void pci_epf_nvme_init_ctrl_regs(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	ctrl->reg = epf_nvme->reg[epf_nvme->reg_bar];

	/* Copy the fabrics controller capabilities as a base */
	ctrl->cap = ctrl->ctrl->cap;

	/* Contiguous Queues Required (CQR) */
	ctrl->cap |= 0x1ULL << 16;

	/* Set Doorbell stride to 4B (DSTRB) */
	ctrl->cap &= ~GENMASK(35, 32);

	/* Clear NVM Subsystem Reset Supported (NSSRS) */
	ctrl->cap &= ~(0x1ULL << 36);

	/* Clear Boot Partition Support (BPS) */
	ctrl->cap &= ~(0x1ULL << 45);

	/* Memory Page Size minimum (MPSMIN) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Memory Page Size maximum (MPSMAX) = 4K */
	ctrl->cap |= (NVME_CTRL_PAGE_SHIFT - 12) << NVME_CC_MPS_SHIFT;

	/* Clear Persistent Memory Region Supported (PMRS) */
	ctrl->cap &= ~(0x1ULL << 56);

	/* Clear Controller Memory Buffer Supported (CMBS) */
	//ctrl->cap &= ~(0x1ULL << 57);

	/* Controller Memory Buffer Supported (CMBS) */
	ctrl->cap |= (0x1ULL << 57);
	ctrl->cmbloc = CMB_BIR;
	/* Controller Memory Buffer Size (CMB page 4kB when SZU set to 0) */
	ctrl->cmbsz = (epf_nvme->epf->bar[CMB_BIR].size >> CMB_PAGE_SHIFT) << 12;
	ctrl->cmbsz |= 1 << 4; /* WDS */
	ctrl->cmbsz |= 1 << 3; /* RDS */
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CMBLOC, ctrl->cmbloc);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CMBSZ, ctrl->cmbsz);

	/* NVMe version supported */
	ctrl->vs = ctrl->ctrl->vs;

	/* Controller configuration */
	ctrl->cc = ctrl->ctrl->ctrl_config & (~NVME_CC_ENABLE);

	/* Controller Status (not ready) */
	ctrl->csts = 0;

	pci_epf_nvme_reg_write64(ctrl, NVME_REG_CAP, ctrl->cap);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_VS, ctrl->vs);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);

	return;
}

static void pci_epf_nvme_delete_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;

	if (ctrl->ctrl) {
		nvme_delete_ctrl(ctrl->ctrl);
		ctrl->ctrl = NULL;
	}

	ctrl->nr_queues = 0;
	kfree(ctrl->cq);
	ctrl->cq = NULL;
	kfree(ctrl->sq);
	ctrl->sq = NULL;
}

static int pci_epf_nvme_create_ctrl(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct device *dev = &epf->dev;
	struct nvme_ctrl *nctrl;
	int ret;

	/* We must have nvme fabrics options. */
	if (!epf_nvme->ctrl_opts_buf) {
		dev_err(dev, "No nvme options specified\n");
		return ret;
	}

	memset(ctrl, 0, sizeof(*ctrl));

	nctrl = nvmf_create_ctrl(dev, epf_nvme->ctrl_opts_buf);
	if (IS_ERR(nctrl)) {
		dev_err(dev, "Create nvme controller failed\n");
		return PTR_ERR(nctrl);
	}

	/* Only support IO controllers */
	if (nctrl->cntrltype != NVME_CTRL_IO) {
		dev_err(dev, "Unsupported controller type\n");
		ret = -EINVAL;
		goto out_delete_ctrl;
	}

	dev_info(dev, "NVMe controller created, %u I/O queues\n",
		 nctrl->queue_count - 1);

	/* Allocate queues */
	ctrl->nr_queues = nctrl->queue_count;
	if (ctrl->nr_queues > epf_nvme->max_nr_queues) {
		/* Admin queue takes one slot, therefore -1 */
		dev_err(dev, "Too many IO queues (maximum allowed: %u)\n",
			epf_nvme->max_nr_queues - 1);
		ret = -EINVAL;
		goto out_delete_ctrl;
	}

	ctrl->sq = kcalloc(ctrl->nr_queues, sizeof(struct pci_epf_nvme_queue),
			   GFP_KERNEL);
	if (!ctrl->sq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	ctrl->cq = kcalloc(ctrl->nr_queues, sizeof(struct pci_epf_nvme_queue),
			   GFP_KERNEL);
	if (!ctrl->cq) {
		ret = -ENOMEM;
		goto out_delete_ctrl;
	}

	epf_nvme->ctrl.ctrl = nctrl;

	pci_epf_nvme_init_ctrl_regs(epf_nvme);

	return 0;

out_delete_ctrl:
	pci_epf_nvme_delete_ctrl(epf);

	return ret;
}

static void pci_epf_nvme_alloc_dma(struct pci_epf_nvme_xfer_thread *xfer_thread)
{
	struct pci_epf_nvme *epf_nvme = xfer_thread->epf_nvme;
	struct pci_epf *epf = epf_nvme->epf;

	if (epf_nvme->dma_enable) {
		xfer_thread->dma.dma_supported =
			pci_epf_nvme_init_dma(epf_nvme, &xfer_thread->dma);
		if (xfer_thread->dma.dma_supported) {
			dev_info(&epf->dev,
				 "Transfer thread %d: DMA supported\n",
				 xfer_thread->tid);
		} else {
			dev_info(&epf->dev,
				 "Transfer thread %d: DMA not supported,"
				 "falling back to mmio\n",
				 xfer_thread->tid);
		}
	} else {
		xfer_thread->dma.dma_supported = false;
		dev_info(&epf->dev, "Transfer thread %d: DMA disabled\n",
			 xfer_thread->tid);
	}
}

static inline void
pci_epf_nvme_free_dma(struct pci_epf_nvme_xfer_thread *xfer_thread)
{
	pci_epf_nvme_clean_dma(xfer_thread->epf_nvme, &xfer_thread->dma);
}

static int pci_epf_nvme_alloc_thread(struct pci_epf_nvme *epf_nvme, int tid)
{
	struct pci_epf_nvme_xfer_thread *thread = &epf_nvme->xfer_threads[tid];
	thread->epf_nvme = epf_nvme;
	thread->tid = tid;
	thread->prp_list_buf = kzalloc(NVME_CTRL_PAGE_SIZE, GFP_KERNEL);
	if (!thread->prp_list_buf) {
		return -ENOMEM;
	}

	pci_epf_nvme_alloc_dma(thread);

	thread->thread = kthread_run(pci_epf_nvme_xfer_thread_fn,
				     (void *)thread,
				     "xfer_thread %d", tid);
	if (IS_ERR_OR_NULL(thread->thread)) {
		dev_err(&epf_nvme->epf->dev,
			"Error creating xfer thread %d\n", tid);
		thread->tid = 0;
		pci_epf_nvme_free_dma(thread);
		kfree(thread->prp_list_buf);
		thread->prp_list_buf = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void pci_epf_nvme_free_thread(struct pci_epf_nvme_xfer_thread *thread)
{
	if (thread) {
		kthread_stop(thread->thread);
		thread->thread = NULL;
		pci_epf_nvme_free_dma(thread);
		kfree(thread->prp_list_buf);
		thread->prp_list_buf = NULL;
	}
}

static void pci_epf_nvme_disable_ctrl(struct pci_epf_nvme *epf_nvme,
				      bool shutdown)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int qid, val, temp = 0;
	bool empty = false;

	dev_info(&epf->dev, "%s controller\n",
		 shutdown ? "Shutting down" : "Disabling");

	/* Stop polling the submission queues */
	cancel_delayed_work_sync(&epf_nvme->sq_poll);

	/* XXX We don't have to execute more commands if disabled !
	   don't transfer SQEs from host ! see NVMe specifications XXX */
	//for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
	//	pci_epf_nvme_drain_sq(epf_nvme, qid);

	/* Wait for all in-flight commands to finish */
	val = atomic_read(&epf_nvme->in_flight_commands);
	dev_info(&epf->dev, "There are %d commands in flight\n", val);
	while((val = atomic_read(&epf_nvme->in_flight_commands))) {
		if (++temp == 100) {
			dev_info(&epf->dev,
				 "Waiting for %d commands to be flushed\n", val);
			empty = kfifo_is_empty(&epf_nvme->xfer_fifo);
			dev_info(&epf->dev,
				 "xfer fifo is %s\n",
				 (empty ?
				 "empty" : "not empty"));
			empty = kfifo_is_empty(&epf_nvme->completion_fifo);
			dev_info(&epf->dev,
				 "completion fifo is %s\n",
				 (empty ?
				 "empty" : "not empty"));
			temp = 0;
		}
		msleep(1);
	}

	/* Tell all the work that the controller is disabled*/
	WRITE_ONCE(epf_nvme->disabled, true);
	if (epf_nvme->xfer_threads) {
		wake_up_all(&xfer_wq);
		for (int i = 0; i < num_xfer_threads; ++i) {
			pci_epf_nvme_free_thread(&epf_nvme->xfer_threads[i]);
		}
		kfree(epf_nvme->xfer_threads);
		dev_info(&epf->dev, "Transfer threads joined\n");
	}
	wake_up(&cq_wq);
	if (epf_nvme->cq_thread) {
		kthread_stop(epf_nvme->cq_thread);
		epf_nvme->cq_thread = NULL;
	}
	dev_info(&epf->dev, "All threads joined\n");

	/*
	 * Unmap the submission queues first to release all references
	 * on the completion queues.
	 */
	for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
		pci_epf_nvme_clean_sq(epf_nvme, qid);

	for (qid = ctrl->nr_queues - 1; qid >= 0; qid--)
		pci_epf_nvme_clean_cq(epf_nvme, qid);

	epf_nvme->currently_mapped_sq = NULL;
	epf_nvme->currently_mapped_cq = NULL;

	if (shutdown) {
		pci_epf_nvme_delete_ctrl(epf);
		ctrl->cc &= ~NVME_CC_SHN_NORMAL;
		ctrl->csts |= NVME_CSTS_SHST_CMPLT;
	}

	/* Tell the host we are done */
	ctrl->csts &= ~NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);
	ctrl->cc &= ~NVME_CC_ENABLE;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CC, ctrl->cc);
}

static void pci_epf_nvme_enable_ctrl(struct pci_epf_nvme *epf_nvme)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf *epf = epf_nvme->epf;
	int ret, effective_xfer_threads;

	dev_info(&epf->dev, "Enabling controller\n");

	ctrl->mps_shift = ((ctrl->cc >> NVME_CC_MPS_SHIFT) & 0xf) + 12;
	ctrl->mps = 1UL << ctrl->mps_shift;
	ctrl->mps_mask = ctrl->mps - 1;

	ctrl->adm_sqes = 1UL << NVME_ADM_SQES;
	ctrl->adm_cqes = sizeof(struct nvme_completion);
	ctrl->io_sqes = 1UL << ((ctrl->cc >> NVME_CC_IOSQES_SHIFT) & 0xf);
	ctrl->io_cqes = 1UL << ((ctrl->cc >> NVME_CC_IOCQES_SHIFT) & 0xf);

	if (ctrl->io_sqes < sizeof(struct nvme_command)) {
		dev_err(&epf->dev, "Unsupported IO sqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_command));
		return;
	}

	if (ctrl->io_cqes < sizeof(struct nvme_completion)) {
		dev_err(&epf->dev, "Unsupported IO cqes %zu (need %zu)\n",
			ctrl->io_sqes, sizeof(struct nvme_completion));
		return;
	}

	ctrl->aqa = pci_epf_nvme_reg_read32(ctrl, NVME_REG_AQA);
	ctrl->asq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ASQ);
	ctrl->acq = pci_epf_nvme_reg_read64(ctrl, NVME_REG_ACQ);

	/*
	 * Map the controller admin submission and completion queues. The
	 * target admin queues were already created when the target was
	 * enabled.
	 */
	ret = pci_epf_nvme_init_cq(epf_nvme, 0,
				   NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED,
				   (ctrl->aqa & 0x0fff0000) >> 16, 0,
				   ctrl->acq & GENMASK(63, 12));
	if (ret)
		return;

	ret = pci_epf_nvme_init_sq(epf_nvme, 0, 0, NVME_QUEUE_PHYS_CONTIG,
				   ctrl->aqa & 0x0fff,
				   ctrl->asq & GENMASK(63, 12));
	if (ret) {
		pci_epf_nvme_clean_cq(epf_nvme, 0);
		return;
	}

	epf_nvme->currently_mapped_sq = NULL;
	epf_nvme->currently_mapped_cq = NULL;

	/* Controller is no longer disabled */
	epf_nvme->disabled = false;

	/* Tell the host we are now ready */
	ctrl->csts |= NVME_CSTS_RDY;
	pci_epf_nvme_reg_write32(ctrl, NVME_REG_CSTS, ctrl->csts);

	/* Start polling the submission queues */
	queue_delayed_work(epf_nvme_sq_wq, &epf_nvme->sq_poll,
			   msecs_to_jiffies(1));

	/* Init transfer threads */
	epf_nvme->xfer_threads = kzalloc(num_xfer_threads *
					sizeof(struct pci_epf_nvme_xfer_thread),
					GFP_KERNEL);
	if (IS_ERR_OR_NULL(epf_nvme->xfer_threads)) {
		dev_err(&epf_nvme->epf->dev, "Error creating xfer threads");
		goto disable;
	}

	/* One window is used by SQ poll thread, one by CQ thread, and each
	 * xfer thread requires its own window */
	effective_xfer_threads = min_t(int, num_xfer_threads,
			epf->epc->num_windows - 2);

	if (effective_xfer_threads < 1) {
		dev_err(&epf_nvme->epf->dev, "Cannot init transfer threads\n");
		goto disable;
	} else if (effective_xfer_threads != num_xfer_threads) {
		dev_warn(&epf_nvme->epf->dev,
			 "Can only accomodate %d transfer threads\n",
			 effective_xfer_threads);
		num_xfer_threads = effective_xfer_threads;
	}

	for (int i = 0; i < num_xfer_threads; ++i) {
		ret = pci_epf_nvme_alloc_thread(epf_nvme, i);
		if (ret < 0) {
			dev_err(&epf_nvme->epf->dev,
				"Error creating xfer thread %d\n", i);
			if (i < 1)
				goto disable;
			else /* We have at least one thread, that's enough */
				break;
		}
	}
	/* Init completion queue thread */
	epf_nvme->cq_thread = kthread_run(pci_epf_nvme_completion_thread_fn,
				(void *)epf_nvme,
				"completion_thread");
	if (IS_ERR_OR_NULL(epf_nvme->cq_thread)) {
		dev_err(&epf_nvme->epf->dev, "Error creating completion thread");
		epf_nvme->cq_thread = NULL;
		goto disable;
	}

	return;
disable:
	pci_epf_nvme_disable_ctrl(epf_nvme, false);
}

static enum rq_end_io_ret pci_epf_nvme_backend_cmd_done_cb(struct request *req,
							blk_status_t blk_status)
{
	struct pci_epf_nvme_cmd *epcmd = req->end_io_data;

	epcmd->ts.backend_end = ktime_get_ns();

	epcmd->status = nvme_req(req)->status;
	blk_mq_free_request(req);

	/* If there was a problem with the backend stop */
	if (epcmd->status != NVME_SC_SUCCESS)
		goto complete;

	pci_epf_nvme_send_cmd_to_next(epcmd);
	return RQ_END_IO_NONE;

complete:
	pci_epf_nvme_send_cmd_to_completion(epcmd);
	return RQ_END_IO_NONE;
}

static int pci_epf_nvme_submit_cmd_nowait(struct pci_epf_nvme *epf_nvme,
					  struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	struct request *req;
	void *buffer = epcmd->buffer;
	unsigned bufflen = epcmd->buffer_size;
	int ret;

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	req = blk_mq_alloc_request(q, nvme_req_op(cmd), 0);

	if (IS_ERR(req)) {
		ret = PTR_ERR(req);
		goto err;
	}
	nvme_init_request(req, cmd);

	if (buffer && bufflen) {
		ret = blk_rq_map_kern(q, req, buffer, bufflen, GFP_KERNEL);
		if (ret)
			goto err_free;
	}

	req->end_io = pci_epf_nvme_backend_cmd_done_cb;
	req->end_io_data = epcmd;
	blk_execute_rq_nowait(req, false);

	return 0;

err_free:
	blk_mq_free_request(req);
err:
	epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	return ret;
}

static void pci_epf_nvme_submit_sync_cmd(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct request_queue *q;
	int ret;

	if (epcmd->ns)
		q = epcmd->ns->queue;
	else
		q = epf_nvme->ctrl.ctrl->admin_q;

	ret = nvme_submit_sync_cmd(q, cmd, epcmd->buffer, epcmd->buffer_size);

	if (ret < 0)
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	else if (ret > 0)
		epcmd->status = ret;
}

static inline
void pci_epf_nvme_send_cmd_to_backend(struct pci_epf_nvme_cmd *epcmd)
{
	int ret;

	epcmd->ts.backend_start = ktime_get_ns();

	/* Always send admin commands, and non read/write commands */
	if (do_backend || epcmd->sqid == 0 ||
	    (epcmd->cmd.common.opcode != nvme_cmd_read &&
	     epcmd->cmd.common.opcode != nvme_cmd_write)) {
		ret = pci_epf_nvme_submit_cmd_nowait(epcmd->epf_nvme, epcmd);
		if (ret)
			pci_epf_nvme_send_cmd_to_completion(epcmd);
	} else {
		pci_epf_nvme_send_cmd_to_completion(epcmd);
	}
}

static inline void pci_epf_nvme_send_cmd_to_xfer(
	struct pci_epf_nvme_cmd *epcmd)
{
	/* XXX TODO Handle full FIFO XXX (unlikely) */

	/* Lock because this is called from multiple threads */
	kfifo_in_spinlocked(&epcmd->epf_nvme->xfer_fifo, &epcmd, 1,
			    &epcmd->epf_nvme->xfer_fifo_wr_lock);
	wake_up(&xfer_wq);
}

static void pci_epf_nvme_admin_create_cq(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 cqid, cq_flags, qsize, vector;
	int ret;

	cqid = le16_to_cpu(cmd->create_cq.cqid);
	if (cqid >= epf_nvme->ctrl.nr_queues || epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cq_flags = le16_to_cpu(cmd->create_cq.cq_flags);
	if (!(cq_flags & NVME_QUEUE_PHYS_CONTIG)) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_cq.qsize);
	if (!qsize || qsize > NVME_CAP_MQES(epf_nvme->ctrl.cap)) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create CQ %d, qsize %d > mqes %d: buggy driver?\n",
				 cqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	vector = le16_to_cpu(cmd->create_cq.irq_vector);
	if (vector >= epf_nvme->nr_vectors) {
		epcmd->status = NVME_SC_INVALID_VECTOR | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_init_cq(epf_nvme, cqid, cq_flags, qsize, vector,
				   le64_to_cpu(cmd->create_cq.prp1));
	if (ret) {
		dev_warn(&epf_nvme->epf->dev,
			 "Cannot map CQ\n");
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	}
}

static void pci_epf_nvme_admin_create_sq(struct pci_epf_nvme *epf_nvme,
					 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	int mqes = NVME_CAP_MQES(epf_nvme->ctrl.cap);
	u16 sqid, cqid, sq_flags, qsize;
	int ret;

	sqid = le16_to_cpu(cmd->create_sq.sqid);
	if (sqid > epf_nvme->ctrl.nr_queues || epf_nvme->ctrl.sq[sqid].ref) {
		epcmd->status = NVME_SC_QID_INVALID | NVME_SC_DNR;
		return;
	}

	cqid = le16_to_cpu(cmd->create_sq.cqid);
	if (sqid && !epf_nvme->ctrl.cq[cqid].ref) {
		epcmd->status = NVME_SC_CQ_INVALID | NVME_SC_DNR;
		return;
	}

	sq_flags = le16_to_cpu(cmd->create_sq.sq_flags);
	if (sq_flags != NVME_QUEUE_PHYS_CONTIG) {
		epcmd->status = NVME_SC_INVALID_QUEUE | NVME_SC_DNR;
		return;
	}

	qsize = le16_to_cpu(cmd->create_sq.qsize);
	if (!qsize || qsize > mqes) {
		if (qsize > mqes)
			dev_warn(&epf_nvme->epf->dev,
				 "Create SQ %d, qsize %d > mqes %d: buggy driver?\n",
				 sqid, (int)qsize, mqes);
		epcmd->status = NVME_SC_QUEUE_SIZE | NVME_SC_DNR;
		return;
	}

	ret = pci_epf_nvme_init_sq(epf_nvme, sqid, cqid, sq_flags, qsize,
				   le64_to_cpu(cmd->create_sq.prp1));
	if (ret) {
		dev_warn(&epf_nvme->epf->dev,
			 "Cannot map SQ\n");
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
	}
}

static void pci_epf_nvme_admin_identify_hook(struct pci_epf_nvme *epf_nvme,
					     struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	struct nvme_id_ctrl *id = epcmd->buffer;
	unsigned int page_shift;

	if (cmd->identify.cns != NVME_ID_CNS_CTRL)
		return;

	/* Set device vendor IDs */
	id->vid = cpu_to_le16(epf_nvme->epf->header->vendorid);
	id->ssvid = id->vid;

	/* Set Maximum Data Transfer Size (MDTS) */
	page_shift = NVME_CAP_MPSMIN(epf_nvme->ctrl.ctrl->cap) + 12;
	id->mdts = ilog2(PCI_EPF_NVME_MDTS) - page_shift;

	/* Clear Controller Multi-Path I/O and Namespace Sharing Capabilities */
	id->cmic = 0;

	/* Do not report support for Autonomous Power State Transitions */
	id->apsta = 0;

	/* Indicate no support for SGLs */
	id->sgls = 0;
}

static u16 pci_epf_nvme_h2c(struct pci_epf_nvme *epf_nvme,
			    size_t len,
			    struct pci_epf_nvme_cmd *epcmd)
{
	int ret;

	/* Set length and direction */
	epcmd->transfer_len = len;
	epcmd->dir = DMA_FROM_DEVICE;
	/* Do directly nested inside sq_poll() */
	epcmd->xfer_thread = &epf_nvme->sq_xfer_info;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		return -1;
	}

	ret = pci_epf_nvme_cmd_parse_dptr(epcmd);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	ret = pci_epf_nvme_cmd_transfer(epcmd);
	if (ret < 0)
		return NVME_SC_DATA_XFER_ERROR;

	return NVME_SC_SUCCESS;
}

static u16 tsp_nvme_cs_identify(struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct nvme_command *cmd;
	u8 *buffer;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		return -1;
	}

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;

	switch (cmd->common.cdw11) {
	case TSP_CS_CSX:
		strncpy((char*)buffer, TSP_CS_ID_STRING, epcmd->transfer_len);
		break;

	default:
		memset(buffer, 0, epcmd->transfer_len);
		break;
	}

	pci_epf_nvme_set_xc_path(epcmd);
	pci_epf_nvme_send_cmd_to_next(epcmd);

	return -1;
}

static u16 tsp_nvme_cs_get(struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct nvme_command *cmd;
	u64 fun;
	s32 function_id;
	u8 *buffer;
	CsFunctionBitSelect f;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		return -1;
	}

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;

	switch (cmd->common.cdw11) {
	case TSP_CS_PROPS:
		memcpy(&(tsp_csx_props.CSE), &tsp_cse_props, sizeof(tsp_cse_props));
		memcpy(buffer, &tsp_csx_props, sizeof(tsp_csx_props));
		break;

	case TSP_CS_CAPS:
		memcpy(buffer, &tsp_cs_caps, sizeof(CsCapabilities));
		break;

	case TSP_CS_FUN:
		fun = (((u64)cmd->common.cdw12) | (((u64)cmd->common.cdw13) << 32));
		memcpy(&f, &fun, sizeof(f));
		if (f.Functions.Checksum) {
			function_id = TSP_CHECKSUM_FUNCTION_ID; // Arbitrary FunctionId ... /// @todo change this
		} else {
			// No other functions supported yet
			function_id = -1;
		}
		memcpy(buffer, &function_id, sizeof(function_id));
	break;

	default:
		// Should not happen... return 0's
		memset(buffer, 0, epcmd->transfer_len);
		break;
	}

	pci_epf_nvme_set_xc_path(epcmd);
	pci_epf_nvme_send_cmd_to_next(epcmd);

	return -1;
}

static phys_addr_t tsp_allocate_in_cmb(struct pci_epf_nvme *epf_nvme,
				       s32 bytes_requested)
{
	u64 cmbmsc;
	phys_addr_t cba;
	u8 *ptr, *base_ptr;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	if (cba) {
		if (bytes_requested > 0) {
			if (bytes_requested & (4096-1)) {
				/// @todo change this ? maybe to lba size ?
				return (phys_addr_t)NULL; // Do not allow requests that are not a multiple of 4096 bytes
				// This will make sure we are 4k aligned in the CMB and that transfers don't transfer outside of buffer
			}
			// Allocate inside locally allocated cmb buffer
			ptr = tsp_mm_alloc((size_t)bytes_requested,
					   epf_nvme->tsp_alloc_ctx);
			if (ptr) {
				base_ptr = epf_nvme->reg[CMB_BIR];
				// Conversion from pointer in local buffer to physical address (for host)
				dev_dbg(&epf_nvme->epf->dev, "bytes req: %u base_ptr: %#0llx ptr: %#0llx\n", bytes_requested, (phys_addr_t)base_ptr, (phys_addr_t)ptr);
				return cba + (ptr - base_ptr);
			}
		}
	}
	// All error / failed allocation cases return "NULL"
	return (phys_addr_t)NULL;
}

static void tsp_deallocate_in_cmb(struct pci_epf_nvme *epf_nvme, phys_addr_t addr)
{
	u64 cmbmsc;
	phys_addr_t cba;
	void *ptr;

	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);
	cba = cmbmsc & CMBMSC_CBA_MASK;

	if (cba) {
		if (addr > cba) {
			ptr = ((char*)(epf_nvme->reg[CMB_BIR])) + (addr - cba);
			tsp_mm_free(ptr, epf_nvme->tsp_alloc_ctx);
		}
	}
}

static u16 tsp_nvme_cs_mm(struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct nvme_command *cmd;
	u8 *buffer;
	u64 cmbmsc;
	s32 bytes_requested;
	phys_addr_t phys_addr, to_be_freed;

	/* Get an internal buffer for the command */
	ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
	if (ret) {
		epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		return -1;
	}

	buffer = epcmd->buffer;
	cmd = &epcmd->cmd;
	cmbmsc = pci_epf_nvme_reg_read64(&epf_nvme->ctrl, NVME_REG_CMBMSC);

	switch (cmd->common.cdw11) {
	case TSP_CS_MEM:
		phys_addr = (phys_addr_t)NULL;
		if (!(cmbmsc & CMBMSC_CMSE_BIT) || !(cmbmsc & CMBMSC_CRE_BIT)) {
			// No CMB enabled, cannot manage memory...
			// (return a null addr)
		} else {
			if (cmd->common.cdw10 == TSP_CS_ALLOCATE) {
				/// @note int32_t because CS API specifies "int" ...
				bytes_requested = cmd->common.cdw12;
				phys_addr = tsp_allocate_in_cmb(epf_nvme, bytes_requested);
			} else if (cmd->common.cdw10 == TSP_CS_DEALLOCATE) {
				to_be_freed = cmd->common.cdw12;
				if (sizeof(phys_addr_t) == 8) {
					// 64 bit hwaddr
					to_be_freed |= ((phys_addr_t)cmd->common.cdw13) << 32;
				}
				tsp_deallocate_in_cmb(epf_nvme, to_be_freed);
			}
		}
		memcpy(buffer, &phys_addr, sizeof(phys_addr));
		break;

	default:
		break;
	}

	pci_epf_nvme_send_cmd_to_next(epcmd);

	return -1;
}

static u16 tsp_compute_checksum(struct pci_epf_nvme *nvme,
				phys_addr_t source, u32 len,
				phys_addr_t dest)
{
	void *cmb_source, *cmb_dest;
	u32 *source_as_u32, *dest_as_u32, checksum, i;

	pci_epf_nvme_print_cmb_info(nvme);

	if (pci_epf_nvme_addr_in_cmb(nvme, source) &&
	    pci_epf_nvme_addr_in_cmb(nvme, dest) &&
	    pci_epf_nvme_addr_in_cmb(nvme, source+len-1) &&
	    pci_epf_nvme_addr_in_cmb(nvme, dest+sizeof(checksum)-1)) {
		cmb_source = pci_epf_nvme_cmb_addr_as_local_addr(nvme, source);
		cmb_dest = pci_epf_nvme_cmb_addr_as_local_addr(nvme, dest);
		source_as_u32 = (u32*)cmb_source;
		dest_as_u32 = (u32*)cmb_dest;
		checksum = 0;
		dev_info(&nvme->epf->dev, "Computing checksum :\n");
		for (i = 0; i < len/sizeof(u32); ++i) {
			checksum += source_as_u32[i];
			//dev_info(&nvme->epf->dev, "value : %#08lx\n", source_as_u32[i]);
		}
		dev_info(&nvme->epf->dev, "Result : %#08x\n", checksum);
		*dest_as_u32 = checksum;
	} else {
		dev_err(&nvme->epf->dev, "Data is outside of CMB\n"
		"source: %#0llx, dest: %#0llx, len: %u\n", source, dest, len);
		// Data is outside of CMB ... don't even try
		return NVME_SC_DATA_XFER_ERROR;
	}
	return NVME_SC_SUCCESS;
}

static u16 tsp_do_compute_request(struct pci_epf_nvme *nvme,
				  CsComputeRequest *creq)
{
	phys_addr_t source, dest;
	u32 len;

	switch (creq->FunctionId) {
	case TSP_CHECKSUM_FUNCTION_ID:
		/// @todo check arguments
		source = (phys_addr_t)creq->Args[0].u.DevMem.MemHandle;
		dest = (phys_addr_t)creq->Args[2].u.DevMem.MemHandle;
		len = creq->Args[1].u.Value32;
		return tsp_compute_checksum(nvme, source, len, dest);
	case TSP_SLEEP_FUNCTION_ID:
		len = creq->Args[0].u.Value32;
		/* Sleeping in deferred requests (handled by a thread) makes
		 * sense, sleeping in a synchronous command will block the
		 * entire controller. This sleep is for testing purposes,
		 * to estimate the time to handle compute requests */
		//dev_info(&nvme->epf->dev, "Sleeping for %d msecs\n", len);
		msleep_interruptible(len);
		//dev_info(&nvme->epf->dev, "Done sleeping\n");
		return NVME_SC_SUCCESS;
	default:
		/// @todo change error handling ?
		return NVME_SC_INVALID_OPCODE;
	}
}

/* This is a function to be threaded */
static int tsp_do_async_compute_request_fn(void *opaque)
{
	AsyncComputeRequest *acreq = (AsyncComputeRequest*)opaque;
	struct pci_epf_nvme_cmd *epcmd;

	if (!acreq) {
		pr_warn("Async compute request pointer is NULL\n");
		return -EINVAL;
	}

	if (!acreq->creq) {
		pr_warn("Async compute request compute request is NULL\n");
		return -EINVAL;
	}

	epcmd = acreq->epcmd;

	/* Set the status inside the request */
	epcmd->status = tsp_do_compute_request(acreq->nvme, acreq->creq);
	/* The thread frees the memory when it is no longer needed
	 * (allocated by caller) */
	kfree(acreq->creq);
	kfree(acreq);

	pci_epf_nvme_send_cmd_to_completion(epcmd);

	return 0;
}

static u16 tsp_do_compute_request_sync(struct pci_epf_nvme *nvme,
				       struct pci_epf_nvme_cmd *epcmd)
{
	return tsp_do_compute_request(nvme, (CsComputeRequest *)epcmd->buffer);
}

static u16 tsp_launch_async_compute(struct pci_epf_nvme *nvme,
				    struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	CsComputeRequest *creq;
	AsyncComputeRequest *acreq;

	/* Allocate memory for the thread, because the buffer is local */
	creq = kzalloc(cmd->common.cdw12 /* req_size */, GFP_KERNEL);
	if (creq == NULL) {
		return NVME_SC_INTERNAL;
	}
	/** @note this copy could be omitted since the buffer lives as long as
	 *  the command (so until the completion is sent) */
	memcpy(creq, epcmd->buffer, cmd->common.cdw12 /* req_size */);
	/* Allocate memory to pass to threaded function */
	acreq = kzalloc(sizeof(AsyncComputeRequest), GFP_KERNEL);
	if (acreq == NULL) {
		kfree(creq);
		return NVME_SC_INTERNAL;
	}
	/* Request lives as long as the queue does and the thread lives as
	 * long as the queue lives, this is a shared structure between the
	 * thread and the nvme controller code, the thread only sets the status
	 * and the nvme code periodically checks (polls) the completion to see
	 * if the thread finished, then joins the thread */
	acreq->nvme = nvme; /* Very important ! */
	acreq->epcmd = epcmd;
	// WHY? req->status = NVME_SUCCESS; // Make sure it is not NVME_NO_COMPLETE
	acreq->creq = creq;
	//dev_info(&nvme->epf->dev, "Launching thread\n");
	epcmd->thread = kthread_run(tsp_do_async_compute_request_fn, (void *)acreq, "TSP Async thread");

	if (IS_ERR_OR_NULL(epcmd->thread)) {
		// The thread could not be created
		dev_err(&nvme->epf->dev, "TSP Error creating async compute thread");
		kfree(acreq->creq);
		kfree(acreq);
		return NVME_SC_INTERNAL;
	}

	/* The dynamically allocated memory above is freed by the thread (maybe
	 * not the best idea), the thread could return the acreq pointer and
	 * both creq and acreq could be freed upon join... */

	/* Dirty way to tell the caller not to enqueue the completion */
	return (u16)-1;
}

static u16 tsp_nvme_cs_compute(struct pci_epf_nvme *nvme,
			       struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd = &epcmd->cmd;
	u16 ret;

	ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);
	if (ret != NVME_SC_SUCCESS)
		return ret;

	if (cmd->common.cdw11) {
		/* Synchronous */
		return tsp_do_compute_request_sync(nvme, epcmd);
	} else {
		/* Asynchronous - will return -1 (deferred) or error code */
		return tsp_launch_async_compute(nvme, epcmd);
	}

	return NVME_SC_SUCCESS;
}

u32 tsp_inet_addr(const char *str) {
	int ret;
	u32 addr;
	u8 *addr_arr = (u8 *)&addr;

	ret = sscanf(str, "%hhd.%hhd.%hhd.%hhd",
		     &addr_arr[0], &addr_arr[1], &addr_arr[2], &addr_arr[3]);

	if (ret < 4)
		return -1;

	return addr;
}

static int tsp_relay_thread_fn(void *opaque)
{
	struct tsp_relay_thread_args *rta =
		(struct tsp_relay_thread_args *)opaque;
	struct pci_epf_nvme *nvme = rta->nvme;
	struct pci_epf_nvme_cmd *epcmd;
	struct tsp_relay *relay = nvme->tsp_relays[rta->relay_desc];
	struct kvec vec;
	struct msghdr msg;
	int ret = 0;

	vec.iov_len = TSP_RELAY_BUFFER_SIZE;

	msg.msg_name = 0;
	msg.msg_namelen = 0;
	msg.msg_control = NULL;
	msg.msg_flags = 0;

	dev_dbg(&nvme->epf->dev, "Relay %d : Thread launched\n",
		rta->relay_desc);

	while (1) {
		/* Wait until we have a command to process */
		wait_event_interruptible(relay->wq,
					 !kfifo_is_empty(&relay->fifo) ||
					 relay->shutdown);

		/* Stop the thread */
		if (relay->shutdown) {
			break;
		}

		ret = kfifo_get(&relay->fifo, &epcmd);
		if (ret != 1) {
			dev_err(&nvme->epf->dev,
			        "Could not get element from relay FIFO\n");
		}

		/* The relay buffers have the size before the data */
		vec.iov_base = epcmd->buffer + sizeof(u32);

		/* Fill the cmd buffer */
		dev_dbg(&nvme->epf->dev, "v Relay %d : Blocking read on socket\n",
			rta->relay_desc);
		ret = kernel_recvmsg(relay->socket, &msg, &vec,
				     TSP_RELAY_BUFFER_SIZE,
				     TSP_RELAY_BUFFER_SIZE, msg.msg_flags);
		dev_dbg(&nvme->epf->dev, "^ Relay %d : read %d bytes\n",
			rta->relay_desc, ret);

		if (ret < 0) {
			dev_warn(&nvme->epf->dev, "Relay %d failed to read\n",
				 rta->relay_desc);
			break;
		}

		*((s32*)epcmd->buffer) = ret;

		pci_epf_nvme_send_cmd_to_next(epcmd);
	}

	/* Empty the FIFO if needed note that this should not hold more than
	   one command at a time ... */
	while(!kfifo_is_empty(&relay->fifo)) {
		ret = kfifo_get(&relay->fifo, &epcmd);
		epcmd->status = NVME_SC_DATA_XFER_ERROR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
	}

	kfree(rta);
	return 0;
}

static int tsp_find_unallocated_relay(struct pci_epf_nvme *nvme)
{
	int i;
	for (i = 0; i < TSP_RELAY_MAX_RELAYS; ++i) {
		if (nvme->tsp_relays[i] == NULL) {
			return i;
		}
	}
	return -1;
}

static int tsp_new_relay(struct pci_epf_nvme *nvme,
			 struct addr_info *ai)
{
	struct socket *sock = NULL;
	struct sockaddr_in saddr = {0,};
	struct tsp_relay *relay;
	struct tsp_relay_thread_args *rta;
	u32 addr;
	u16 port;
	int ret, relay_number = tsp_find_unallocated_relay(nvme);

	if (relay_number < 0)
		return relay_number;

	dev_dbg(&nvme->epf->dev, "Relay %d : addr (node) : %s\n",
		relay_number, ai->node);
	dev_dbg(&nvme->epf->dev, "Relay %d : port (service) : %s\n",
		relay_number, ai->service);

	addr = tsp_inet_addr(ai->node);
	if (addr == (u32)-1) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not parse addr (node) : %s\n",
			 relay_number, ai->node);
		return -1;
	}

	ret = kstrtou16(ai->service, 10, &port);
	if (ret < 0) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not parse port (serice) : %s\n",
			 relay_number, ai->node);
		return -1;
	}

	saddr.sin_family = AF_INET;
	saddr.sin_port = htons(port);
	saddr.sin_addr.s_addr = addr;

	ret = sock_create(PF_INET, SOCK_STREAM, IPPROTO_TCP, &sock);
	if (ret < 0) {
		dev_warn(&nvme->epf->dev, "Relay %d : Socket creation failed\n",
			 relay_number);
		return -1;
	}

	ret = sock->ops->connect(sock, (struct sockaddr *)&saddr, sizeof(saddr),
				 O_RDWR);
	if (ret) {
		dev_warn(&nvme->epf->dev,
			 "Relay %d : Could not connect, ret = %d\n",
			 relay_number, ret);
		sock_release(sock);
		return -1;
	}

	relay = kzalloc(sizeof(struct tsp_relay), GFP_KERNEL);
	if (!relay) {
		dev_warn(&nvme->epf->dev, "Could not allocate relay\n");
		sock_release(sock);
		return -1;
	}
	ret = kfifo_alloc(&relay->fifo, PCI_EPF_NVME_FIFO_SIZE,
			  GFP_KERNEL);
	if (ret < 0) {
		dev_err(&nvme->epf->dev, "Could not allocate relay FIFO\n");
		kfree(relay);
		sock_release(sock);
		return ret;
	}

	nvme->tsp_relays[relay_number] = relay;
	dev_dbg(&nvme->epf->dev, "Relay %d : Connected !\n", relay_number);
	relay->socket = sock;
	relay->shutdown = false;
	init_waitqueue_head(&relay->wq);

	/* These are freed by the thread itself */
	rta = kzalloc(sizeof(struct tsp_relay_thread_args), GFP_KERNEL);
	rta->nvme = nvme;
	rta->relay_desc = relay_number;
	relay->thread = kthread_run(tsp_relay_thread_fn, (void *)rta,
				    "TSP relay thread %d", relay_number);

	if (IS_ERR_OR_NULL(relay->thread)) {
		/* The thread could not be created */
		dev_err(&nvme->epf->dev, "TSP Error creating relay thread");
		kfree(rta);
		sock_release(sock);
		return -1;
	}

	return relay_number;
}

static u16 tsp_nvme_address_info_from_req(struct pci_epf_nvme *nvme,
					  struct pci_epf_nvme_cmd *epcmd,
					  struct addr_info *ai) {
	u16 ret;

	ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);

	if (ret != NVME_SC_SUCCESS)
		return ret;

	memcpy(ai, epcmd->buffer, sizeof(*ai));
	return NVME_SC_SUCCESS;
}

static u16 tsp_nvme_cs_open_relay(struct pci_epf_nvme *nvme,
				  struct pci_epf_nvme_cmd *epcmd)
{
	struct addr_info ai;
	int desc;
	u16 ret;

	ret = tsp_nvme_address_info_from_req(nvme, epcmd, &ai);
	if (ret != NVME_SC_SUCCESS)
		return ret;

	desc = tsp_new_relay(nvme, &ai);
	if (desc < 0)
		return NVME_SC_INTERNAL;

	/* buffer is allocated in tsp_nvme_address_info_from_req */
	*(s32*)epcmd->buffer = (s32)desc;

	/* This is a bidirectional command */
	epcmd->dir = DMA_TO_DEVICE;
	pci_epf_nvme_set_xc_path(epcmd);
	pci_epf_nvme_send_cmd_to_next(epcmd);

	return -1;
}

static int tsp_nvme_cs_stop_relay_thread(struct pci_epf_nvme *nvme,
					 struct tsp_relay *relay)
{
	int ret;
	relay->shutdown = true;

	/* The thread could be waiting on a buffer */
	wake_up(&relay->wq);

	/* The thread could be blocked on recv() on socket */
	kernel_sock_shutdown(relay->socket, SHUT_RDWR);
	ret = kthread_stop(relay->thread);
	sock_release(relay->socket);

	return ret;
}

static u16 tsp_nvme_cs_close_relay(struct pci_epf_nvme *nvme,
				   struct pci_epf_nvme_cmd *epcmd)
{
	int ret;
	int desc = epcmd->cmd.common.cdw13;
	struct tsp_relay *relay;

	if (desc < 0 || desc >= TSP_RELAY_MAX_RELAYS) {
		/* Out of range... */
		return NVME_SC_INVALID_FIELD;
	}

	relay = nvme->tsp_relays[desc];

	if (!relay) {
		dev_warn(&nvme->epf->dev, "Relay %d is already closed\n", desc);
	} else {
		ret = tsp_nvme_cs_stop_relay_thread(nvme, relay);

		kfree(relay);
		nvme->tsp_relays[desc] = NULL;
	}

	return NVME_SC_SUCCESS;
}

static inline void tsp_nvme_cmd_to_read_relay(struct pci_epf_nvme_cmd *epcmd)
{
	struct tsp_relay *relay = epcmd->epf_nvme->tsp_relays[epcmd->relay_desc];

	/* XXX TODO Handle full FIFO (unlikely) XXX */
	kfifo_in(&relay->fifo, &epcmd, 1);
	wake_up(&relay->wq);
}

#if 0 /* XXX TODO handle close relay in edge cases */

/* Do not use this function in a thread, call only if filled_buffers is not
 * empty, this is a very specific helper function, don't use */
static u16 tsp_nvme_cs_comm_to_host(struct pci_epf_nvme *nvme,
				    struct pci_epf_nvme_cmd *epcmd)
{
	/// @todo check data_len
	int ret = NVME_SC_SUCCESS;
	int desc = epcmd->cmd.common.cdw13;
	struct tsp_relay *relay = nvme->tsp_relays[desc];

	if (!relay) {
		dev_warn(&nvme->epf->dev, "Relay %d isn't allocated\n", desc);
		epcmd->status = NVME_SC_DATA_XFER_ERROR;
		return NVME_SC_DATA_XFER_ERROR;
	}

	/** @todo it could be better to use the CQE "result" field to pass the
	 * size */
	dev_dbg(&nvme->epf->dev,
		"Relay %d : Transferring 4k buffer with %d bytes to host\n",
		desc, relay->buffer.pack.size);

	ret = pci_epf_nvme_cmd_parse_dptr(nvme, epcmd, SZ_4K);
	if (ret < 0) {
		return epcmd->status;
	}
	/** @todo this copy could be avoided but requires to have the relay
	 * thread fill the data directly into the requests */
	memcpy(epcmd->buffer, &relay->buffer.pack, SZ_4K);
	ret = pci_epf_nvme_cmd_transfer(nvme, epcmd, DMA_TO_DEVICE /* c2h */);
	if (ret < 0) {
		return epcmd->status;
	}

	relay->buffer_full = false;
	wake_up(&relay->wq);

	if (relay->buffer.pack.size == 0) {
		/* Disconnection, close relay (stops thread) */
		dev_dbg(&nvme->epf->dev,
			"Relay %d : socket disconnected, closing relay\n", desc);

		return tsp_nvme_cs_close_relay(nvme, epcmd);
	}

	return epcmd->status;
}

#endif

static u16 tsp_nvme_cs_comm(struct pci_epf_nvme *nvme,
			    struct pci_epf_nvme_cmd *epcmd)
{
	struct msghdr msg;
	struct kvec vec;
	struct socket *sock;
	u32 write_nread = epcmd->cmd.common.cdw11;
	u32 length = epcmd->cmd.common.cdw12;
	u32 desc = epcmd->cmd.common.cdw13;
	u16 ret;

	if (desc >= TSP_RELAY_MAX_RELAYS) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d does not exist\n", desc);
		return NVME_SC_INVALID_FIELD;
	}
	if (!nvme->tsp_relays[desc]) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d is not allocated\n", desc);
		return NVME_SC_INVALID_FIELD;
	}

	sock = nvme->tsp_relays[desc]->socket;

	if (!sock) {
		dev_warn(&nvme->epf->dev,
			 "Relay with descriptor %d has no socket\n", desc);
		return NVME_SC_DATA_XFER_ERROR;
	}

	if (length > SZ_4K) {
		dev_warn(&nvme->epf->dev,
			 "Cannot send more than 4k for the moment\n");
		return NVME_SC_DATA_XFER_ERROR;
	}

	if (write_nread) {
		/* If this is a write get data from host */
		ret = pci_epf_nvme_h2c(nvme, SZ_4K, epcmd);
		if (ret != NVME_SC_SUCCESS) {
			return ret;
		}

		dev_dbg(&nvme->epf->dev,
			"Relay %d : Writing %d bytes to socket\n", desc, length);

		msg.msg_name = 0;
		msg.msg_namelen = 0;
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
		msg.msg_flags = 0;

		vec.iov_len = length;
		vec.iov_base = epcmd->buffer;
		/* And send it into the socket */
		ret = kernel_sendmsg(sock, &msg, &vec, length, length);
		if (ret < 0) {
			dev_warn(&nvme->epf->dev,
				 "Relay %d : Failed to write to socket\n", desc);
			return NVME_SC_INTERNAL;
		} else if (ret < length) {
			dev_warn(&nvme->epf->dev,
				 "Relay %d : Failed to write all data to socket\n",
				 desc);
			return NVME_SC_DATA_XFER_ERROR;
		}
		return NVME_SC_SUCCESS;
	} else {
		/* This is a read */
		dev_dbg(&nvme->epf->dev,
			"Relay %d : A socket read was requested...\n", desc);

		epcmd->transfer_len = SZ_4K;
		epcmd->dir = DMA_TO_DEVICE;
		pci_epf_nvme_set_xc_path(epcmd);
		/* Get an internal buffer for the command */
		ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
		if (ret) {
			epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
			pci_epf_nvme_send_cmd_to_completion(epcmd);
			return -1;
		}

		epcmd->relay_desc = desc;
		pci_epf_nvme_set_rxc_path(epcmd);
		pci_epf_nvme_send_cmd_to_next(epcmd);
		return (u16)-1; /* deferred */
	}

	return NVME_SC_SUCCESS;
}

static u16 pci_epf_nvme_process_custom_admin_cmd(struct pci_epf_nvme *nvme,
						 struct pci_epf_nvme_cmd *epcmd)
{
	struct nvme_command *cmd;
	cmd = &epcmd->cmd;
	dev_dbg(&nvme->epf->dev, "TSP: Opcode: %#x, CDW10: %#x, CDW11: %#x\n",
		cmd->common.opcode, cmd->common.cdw10, cmd->common.cdw11);
	switch (cmd->common.cdw10) {
	case TSP_CS_IDENTIFY:
		epcmd->transfer_len = SZ_4K;
		epcmd->dir = DMA_TO_DEVICE;
		pci_epf_nvme_set_xc_path(epcmd);
		return tsp_nvme_cs_identify(epcmd);
	case TSP_CS_GET:
		epcmd->transfer_len = SZ_4K;
		epcmd->dir = DMA_TO_DEVICE;
		pci_epf_nvme_set_xc_path(epcmd);
		return tsp_nvme_cs_get(epcmd);
	case TSP_CS_ALLOCATE:
		/* fallthrough */
	case TSP_CS_DEALLOCATE:
		epcmd->transfer_len = SZ_4K;
		epcmd->dir = DMA_TO_DEVICE;
		pci_epf_nvme_set_xc_path(epcmd);
		return tsp_nvme_cs_mm(epcmd);
	case TSP_CS_COMPUTE:
		return tsp_nvme_cs_compute(nvme, epcmd);
	case TSP_CS_COMM:
		return tsp_nvme_cs_comm(nvme, epcmd);
	case TSP_CS_OPEN_RELAY:
		return tsp_nvme_cs_open_relay(nvme, epcmd);
	case TSP_CS_CLOSE_RELAY:
		return tsp_nvme_cs_close_relay(nvme, epcmd);
	default:
		break;
	}

	return NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
}

static void pci_epf_nvme_process_admin_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	void (*post_process_hook)(struct pci_epf_nvme *,
				  struct pci_epf_nvme_cmd *) = NULL;
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct nvme_command *cmd = &epcmd->cmd;
	u16 status;
	int ret = 0;

	if (epcmd->cmd.common.opcode >= nvme_admin_vendor_start) {
		if (epcmd->cmd.common.opcode == 0xCC) {
			/* Redirect these commands to user space */
			pci_epf_nvme_set_uc_path(epcmd);
			return;
		} else {
			status = pci_epf_nvme_process_custom_admin_cmd(epf_nvme,
								       epcmd);
			/* The above returns -1 to show that the cmd is deferred
			 */
			if (status == (u16)-1) {
				/* Request is handled by another thread / task */
				return;
			} else if (status < 0) {
				dev_warn(&epf_nvme->epf->dev,
					"Negative status from custom admin"
					" command\n");
			} else {
				/* This might not be needed, it should be set */
				epcmd->status = status;
			}
		}
		goto complete;
	}

	switch (epcmd->cmd.common.opcode) {
	case nvme_admin_identify:
		post_process_hook = pci_epf_nvme_admin_identify_hook;
		epcmd->transfer_len = NVME_IDENTIFY_DATA_SIZE;
		pci_epf_nvme_set_xc_path(epcmd);
		epcmd->dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_get_log_page:
		epcmd->transfer_len = nvme_get_log_page_len(cmd);
		pci_epf_nvme_set_xc_path(epcmd);
		epcmd->dir = DMA_TO_DEVICE;
		break;

	case nvme_admin_async_event:
		/* XXX For now XXX */
		dev_info(&epf_nvme->epf->dev, "nvme_admin_async_event\n");
		/* The controller posts a completion queue entry for this
		 * command when there is an asynchronous event to report to the
		 * host. If Asynchronous Event Request commands are outstanding
		 * when the controller is reset, then each of those commands is
		 * aborted and should not return a CQE */

		/* XXX For the moment just ignore and free the command XXX */
		pci_epf_nvme_free_cmd(epcmd);
		return;

	case nvme_admin_get_features:
		/* XXX TODO XXX */
		/* Fallthrough */
	case nvme_admin_set_features:
		/* Here the result is only for IO queues, and is 0 based,
		 * therefore ctrl number of queues -1 (admin) -1 ( 0 based)
		 * We are allowed to return a number of queues different than
		 * what the host wants to set (smaller or bigger) */
		if ((cmd->features.fid & 0xff) == NVME_FEAT_NUM_QUEUES) {
			epcmd->cqe.result.u32 =
				(epf_nvme->ctrl.nr_queues - 2) << 16 |
				(epf_nvme->ctrl.nr_queues - 2);
		}
	case nvme_admin_abort_cmd:
		break;

	case nvme_admin_create_cq:
		pci_epf_nvme_admin_create_cq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_create_sq:
		pci_epf_nvme_admin_create_sq(epf_nvme, epcmd);
		goto complete;

	case nvme_admin_delete_cq:
	case nvme_admin_delete_sq:
	case nvme_admin_ns_attach:
		goto complete;

	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled admin command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd), cmd->common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (epcmd->transfer_len) {
		/* Get an internal buffer for the command */
		ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
		if (ret) {
			epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
			goto complete;
		}
	}

	/* Note this could also be dispatched */
	pci_epf_nvme_submit_sync_cmd(epf_nvme, epcmd);
	if (epcmd->status != NVME_SC_SUCCESS)
		pci_epf_nvme_send_cmd_to_completion(epcmd);

	/* Command done: post process it and transfer data if needed */
	if (post_process_hook)
		post_process_hook(epf_nvme, epcmd);

	pci_epf_nvme_send_cmd_to_next(epcmd);
	return;

complete:
	pci_epf_nvme_send_cmd_to_completion(epcmd);
}

static inline size_t pci_epf_nvme_rw_data_len(struct pci_epf_nvme_cmd *epcmd)
{
	return ((u32)le16_to_cpu(epcmd->cmd.rw.length) + 1) <<
		epcmd->ns->lba_shift;
}

static inline
void pci_epf_nvme_send_cmd_to_user(struct pci_epf_nvme_cmd *epcmd)
{
	/* XXX TODO Handle full FIFO XXX (unlikely) */

	/* Lock because this is called from multiple threads */
	if (epcmd->sqid == 0) { /* admin */
		kfifo_in_spinlocked(&touser_admin_fifo, &epcmd, 1,
				    &touser_admin_wr_lk);
		wake_up(&touser_admin_wq);
	} else { /* IO */
		kfifo_in_spinlocked(&touser_fifo, &epcmd, 1, &touser_wr_lk);
		wake_up(&touser_wq);
	}
}

void pci_epf_nvme_process_custom_io_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	int ret;

	switch (epcmd->cmd.common.opcode) {
	case nvme_cmd_vendor_start:
		pci_epf_nvme_set_uc_path(epcmd);
		break;
	case nvme_cmd_vendor_start + nvme_cmd_read:
		epcmd->transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dir = DMA_TO_DEVICE;
		/* Read from user space, not the same as backend read through
		 * userspace */
		pci_epf_nvme_set_uxc_path(epcmd);
		break;
	case nvme_cmd_vendor_start + nvme_cmd_write:
		epcmd->transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		epcmd->dir = DMA_FROM_DEVICE;
		pci_epf_nvme_set_xuc_path(epcmd);
		break;
	default:
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		pci_epf_nvme_send_cmd_to_completion(epcmd);
		return;
	}

	if (epcmd->transfer_len) {
		/* Get an internal buffer for the command */
		ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
		if (ret) {
			epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
			pci_epf_nvme_send_cmd_to_completion(epcmd);
		}
	}
	pci_epf_nvme_send_cmd_to_next(epcmd);
}

void pci_epf_nvme_process_io_cmd(struct pci_epf_nvme_cmd *epcmd)
{
	struct pci_epf_nvme *epf_nvme = epcmd->epf_nvme;
	struct nvme_command *cmd = &epcmd->cmd;
	int ret = 0;

	if (!epcmd) /* Should not happen */
		return;

	/* Get the command target namespace */
	epcmd->ns = nvme_find_get_ns(epf_nvme->ctrl.ctrl,
				     le32_to_cpu(cmd->common.nsid));
	if (!epcmd->ns) {
		epcmd->status = NVME_SC_INVALID_NS | NVME_SC_DNR;
		goto complete;
	}

	if (cmd->common.opcode >= nvme_cmd_vendor_start) {
		pci_epf_nvme_process_custom_io_cmd(epcmd);
		return;
	}

	switch (cmd->common.opcode) {
	case nvme_cmd_read:
		if (!do_readwrite) {
			pci_epf_nvme_send_cmd_to_completion(epcmd);
			return;
		}
		epcmd->transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		if (epf_nvme->user_path_enable)
			pci_epf_nvme_set_buxc_path(epcmd);
		else
			pci_epf_nvme_set_bxc_path(epcmd);
		epcmd->dir = DMA_TO_DEVICE;
		break;

	case nvme_cmd_write:
		if (!do_readwrite) {
			pci_epf_nvme_send_cmd_to_completion(epcmd);
			return;
		}
		epcmd->transfer_len = pci_epf_nvme_rw_data_len(epcmd);
		if (epf_nvme->user_path_enable)
			pci_epf_nvme_set_xubc_path(epcmd);
		else
			pci_epf_nvme_set_xbc_path(epcmd);
		epcmd->dir = DMA_FROM_DEVICE;
		break;

	case nvme_cmd_dsm:
		epcmd->transfer_len = (le32_to_cpu(cmd->dsm.nr) + 1) *
			sizeof(struct nvme_dsm_range);
		epcmd->dir = DMA_FROM_DEVICE;
		goto complete;

	case nvme_cmd_flush:
		pci_epf_nvme_set_bc_path(epcmd);
		break;

	case nvme_cmd_write_zeroes:
		if (epf_nvme->user_path_enable) {
			/* Convert to a write command so that userspace can
			 * modiffy the data, e.g., for encryption */
			epcmd->transfer_len = pci_epf_nvme_rw_data_len(epcmd);
			epcmd->cmd.common.opcode = nvme_cmd_write;
			pci_epf_nvme_set_ubc_path(epcmd);
		} else
			pci_epf_nvme_set_bc_path(epcmd);
		break;

	default:
		dev_err(&epf_nvme->epf->dev,
			"Unhandled IO command %s (0x%02x)\n",
			pci_epf_nvme_cmd_name(epcmd),
			cmd->common.opcode);
		epcmd->status = NVME_SC_INVALID_OPCODE | NVME_SC_DNR;
		goto complete;
	}

	if (epcmd->transfer_len) {
		/* Get an internal buffer for the command */
		ret = pci_epf_nvme_alloc_cmd_buffer(epcmd);
		if (ret) {
			epcmd->status = NVME_SC_INTERNAL | NVME_SC_DNR;
			goto complete;
		}
	}

	pci_epf_nvme_send_cmd_to_next(epcmd);
	return;

complete:
	pci_epf_nvme_send_cmd_to_completion(epcmd);
	return;
}

static bool pci_epf_nvme_process_cmds(struct pci_epf_nvme *epf_nvme, int qid)
{
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	struct pci_epf_nvme_queue *sq = &ctrl->sq[qid];
	struct pci_epf_nvme_cmd *epcmd;
	int num_cmds = pci_epf_nvme_fetch_sqes(epf_nvme, qid);
	int in_flight_cmds;

	if (!num_cmds)
		return false; /* Queue is inactive */

	in_flight_cmds = atomic_read(&epf_nvme->in_flight_commands);
	if (num_cmds + in_flight_cmds > PCI_EPF_NVME_FIFO_SIZE) {
		return false; /* Wait for space in controller */
	}

	/* While there are commands in the local queue, proccess them */
	while (sq->head != sq->local_tail) {
		do {
			epcmd = pci_epf_nvme_alloc_cmd(epf_nvme);
			if (!epcmd) { /* No space in cache, retry later */
				dev_warn_once(&epf_nvme->epf->dev,
					      "Alloc wait...\n");
				usleep_range(10, 100);
			}
		} while (!epcmd);
		pci_epf_nvme_init_cmd(epf_nvme, epcmd, sq->qid, sq->cqid);
		memcpy(&epcmd->cmd, &sq->local_queue[sq->head],
		       sizeof(epcmd->cmd));
		sq->head++;
		if (sq->head == sq->depth)
			sq->head = 0;
		if (qid)
			pci_epf_nvme_process_io_cmd(epcmd);
		else
			pci_epf_nvme_process_admin_cmd(epcmd);
	}

	return true; /* Queue is active */
}

static void pci_epf_nvme_sq_poll(struct work_struct *work)
{
	struct pci_epf_nvme *epf_nvme =
		container_of(work, struct pci_epf_nvme, sq_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	bool active_queues = true;
	int qid, in_flight_cmds;

	/* Process pending commands, starting with the IO queues */
	while (active_queues && pci_epf_nvme_ctrl_ready(ctrl)) {
		active_queues = false;
		for (qid = 0; qid < ctrl->nr_queues; qid++) {
			active_queues |= pci_epf_nvme_process_cmds(epf_nvme, qid);
		}
	}

	if (!pci_epf_nvme_ctrl_ready(ctrl))
		return;

	in_flight_cmds = atomic_read(&epf_nvme->in_flight_commands);
	if (in_flight_cmds > relaxed_poll_threshold)
		/* The controller has a lot of work already, wait a bit more */
		usleep_range(PCI_EPF_NVME_MIN_POLL_DELAY_US,
			     poll_relaxed_delay_usecs);
	else {
		/* The controller is not busy, poll often */
		usleep_range(PCI_EPF_NVME_MIN_POLL_DELAY_US, poll_delay_usecs);
	}
	/* We wait above and don't use jiffy parameter below because one jiffy
	   can actually be 10ms if HZ = 100 so sleep few us above and queue
	   directly, 10ms polling is way to slow */
	queue_delayed_work(epf_nvme_sq_wq, &epf_nvme->sq_poll, 0);
}

static void pci_epf_nvme_reg_poll(struct work_struct *work)
{
	struct pci_epf_nvme *epf_nvme =
		container_of(work, struct pci_epf_nvme, reg_poll.work);
	struct pci_epf_nvme_ctrl *ctrl = &epf_nvme->ctrl;
	bool shutdown;
	u32 old_cc;

	/* Check CC.EN to determine what we need to do */
	old_cc = ctrl->cc;
	ctrl->cc = pci_epf_nvme_reg_read32(ctrl, NVME_REG_CC);

	/* If not enabled yet, wait */
	if (!(old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE))
		goto again;

	/* If CC.EN was set by the host, enable the controller */
	if (!(old_cc & NVME_CC_ENABLE) && (ctrl->cc & NVME_CC_ENABLE)) {
		pci_epf_nvme_enable_ctrl(epf_nvme);
		goto again;
	}

	/* If CC.EN was cleared by the host, disable the controller */
	shutdown = ctrl->cc & NVME_CC_SHN_NORMAL;
	if (((old_cc & NVME_CC_ENABLE) && !(ctrl->cc & NVME_CC_ENABLE)) ||
	    shutdown) {
		pci_epf_nvme_disable_ctrl(epf_nvme, shutdown);
		if (shutdown)
			return;
	}

again:
	queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
			   msecs_to_jiffies(5));
}

/* Char device operations */
static int tsp_open(struct inode *inode, struct file *file)
{
	struct tsp_file_data *tfd = container_of(inode->i_cdev,
						 struct tsp_file_data, cdev);
	struct pci_epf_nvme *epf_nvme = tfd->epf_nvme;

	/* Allow only one process to open the file at a time */
	if (!mutex_trylock(&tfd->mutex))
		return -EBUSY;

	tfd->epcmd = NULL;

	file->private_data = tfd;

	/* The private data is used to store the epcmd currently being worked on,
	   this will allow to sync the read/write commands, then later use the
	   offset maybe to handle multiple read async (note that this is very
	   hacky...) */

	return 0;
}

static int tsp_release(struct inode *inode, struct file *file)
{
	struct tsp_file_data *tfd = file->private_data;
	//struct pci_epf_nvme *epf_nvme = tfd->epf_nvme;

	mutex_unlock(&tfd->mutex);

	if (tfd->epcmd) {
		/* If there is an unprocessed command queue it */
		tfd->epcmd->status = NVME_SC_INTERNAL;
		pci_epf_nvme_send_cmd_to_completion(tfd->epcmd);
		tfd->epcmd = NULL;
	}

	return 0;
}

static ssize_t tsp_read(struct file *file, char __user *buf, size_t count,
			loff_t *offset)
{
	int ret;
	struct tsp_file_data *tfd = file->private_data;
	struct pci_epf_nvme *epf_nvme = tfd->epf_nvme;

	if (!tfd) {
		dev_err(&epf_nvme->epf->dev,
			"Missing file data pointer !\n");
		return -EFAULT;
	}

	if (count < sizeof(struct nvme_command))
		return -EINVAL;

	if (tfd->epcmd) {
		dev_warn(&epf_nvme->epf->dev,
			 "Cannot process multiple commands for the moment\n");
		return -EINVAL;
	}

	spin_lock(&touser_rd_lk);
	while (kfifo_is_empty(&touser_fifo)) {
		spin_unlock(&touser_rd_lk);
		if (file->f_flags & O_NONBLOCK) /* Do not block */
			return 0;
		ret = wait_event_interruptible(touser_wq,
					       !kfifo_is_empty(&touser_fifo));
		if (ret < 0) /* Interrupted by signal */
			return 0;

		spin_lock(&touser_rd_lk);
	}

	ret = kfifo_get(&touser_fifo, &tfd->epcmd);
	spin_unlock(&touser_rd_lk);
	if (ret != 1) {
		dev_err(&epf_nvme->epf->dev,
			"Could not get element from touser FIFO\n");
		return -EFAULT;
	}

	if (!tfd->epcmd) {
		dev_err(&epf_nvme->epf->dev,
			"Missing epcmd pointer !\n");
		return -EFAULT;
	}

	if (!buf) {
		dev_err(&epf_nvme->epf->dev,
			"Userspace buffer pointer seems to be NULL\n");
		return -EFAULT;
	}

	if (count < (sizeof(tfd->epcmd->cmd) + sizeof(tfd->epcmd->buffer_size))) {
		dev_err(&epf_nvme->epf->dev,
			"Userspace buffer is too small\n");
		/* Not we could requeue it as well ... */
		goto fail_command;
	}
	ret = copy_to_user(buf, &tfd->epcmd->cmd, sizeof(tfd->epcmd->cmd));
	if (ret) {
		dev_err(&epf_nvme->epf->dev,
			"Incomplete copy to user fail\n");
		goto fail_command;
	}
	count = sizeof(tfd->epcmd->cmd);
	if (tfd->epcmd->buffer && tfd->epcmd->buffer_size) {
		ret = copy_to_user(buf + sizeof(tfd->epcmd->cmd),
				   tfd->epcmd->buffer,
				   tfd->epcmd->buffer_size);
		if (ret) {
			dev_err(&epf_nvme->epf->dev,
				"Incomplete copy to user fail on buffer\n");
			goto fail_command;
		}
	}
	count += tfd->epcmd->buffer_size;

	//dev_info(&epf_nvme->epf->dev, "TSP: Cmd read from userspace\n");
	return count;

fail_command:
	tfd->epcmd->status = NVME_SC_INTERNAL;
	pci_epf_nvme_send_cmd_to_completion(tfd->epcmd);
	tfd->epcmd = NULL;
	return -EFAULT;
}

static ssize_t tsp_admin_read(struct file *file, char __user *buf, size_t count,
			      loff_t *offset)
{
	int ret;
	struct tsp_file_data *tfd = file->private_data;
	struct pci_epf_nvme *epf_nvme = tfd->epf_nvme;

	if (!tfd) {
		dev_err(&epf_nvme->epf->dev,
			"Missing file data pointer !\n");
		return -EFAULT;
	}

	if (count < sizeof(struct nvme_command))
		return -EINVAL;

	if (tfd->epcmd) {
		dev_warn(&epf_nvme->epf->dev,
			 "Cannot process multiple commands for the moment\n");
		return -EINVAL;
	}

	spin_lock(&touser_admin_rd_lk);
	while (kfifo_is_empty(&touser_admin_fifo)) {
		spin_unlock(&touser_admin_rd_lk);
		if (file->f_flags & O_NONBLOCK) /* Do not block */
			return 0;
		ret = wait_event_interruptible(touser_admin_wq,
				!kfifo_is_empty(&touser_admin_fifo));
		if (ret < 0) /* Interrupted by signal */
			return 0;

		spin_lock(&touser_admin_rd_lk);
	}

	ret = kfifo_get(&touser_admin_fifo, &tfd->epcmd);
	spin_unlock(&touser_admin_rd_lk);
	if (ret != 1) {
		dev_err(&epf_nvme->epf->dev,
			"Could not get element from touser admin FIFO\n");
		return -EFAULT;
	}

	if (!tfd->epcmd) {
		dev_err(&epf_nvme->epf->dev,
			"Missing epcmd pointer !\n");
		return -EFAULT;
	}

	if (!buf) {
		dev_err(&epf_nvme->epf->dev,
			"Userspace buffer pointer seems to be NULL\n");
		return -EFAULT;
	}

	if (count < (sizeof(tfd->epcmd->cmd) + sizeof(tfd->epcmd->buffer_size))) {
		dev_err(&epf_nvme->epf->dev,
			"Userspace buffer is too small\n");
		/* Not we could requeue it as well ... */
		goto fail_command;
	}
	ret = copy_to_user(buf, &tfd->epcmd->cmd, sizeof(tfd->epcmd->cmd));
	if (ret) {
		dev_err(&epf_nvme->epf->dev,
			"Incomplete copy to user fail\n");
		goto fail_command;
	}
	count = sizeof(tfd->epcmd->cmd);
	if (tfd->epcmd->buffer && tfd->epcmd->buffer_size) {
		ret = copy_to_user(buf + sizeof(tfd->epcmd->cmd),
				   tfd->epcmd->buffer,
				   tfd->epcmd->buffer_size);
		if (ret) {
			dev_err(&epf_nvme->epf->dev,
				"Incomplete copy to user fail on buffer\n");
			goto fail_command;
		}
	}
	count += tfd->epcmd->buffer_size;

	return count;

fail_command:
	tfd->epcmd->status = NVME_SC_INTERNAL;
	pci_epf_nvme_send_cmd_to_completion(tfd->epcmd);
	tfd->epcmd = NULL;
	return -EFAULT;
}

static ssize_t tsp_write(struct file *file, const char __user *buf, size_t count,
			 loff_t *offset)
{
	struct tsp_file_data *tfd = file->private_data;
	struct pci_epf_nvme *epf_nvme = tfd->epf_nvme;

	if (count < sizeof(struct nvme_completion))
		return -EINVAL;

	if (!tfd->epcmd) {
		dev_warn(&epf_nvme->epf->dev,
			 "No pending command, read by userspace first\n");
		return -EINVAL;
	}

	if (copy_from_user(&tfd->epcmd->cqe, buf, sizeof(tfd->epcmd->cqe))) {
		dev_err(&epf_nvme->epf->dev,
			"Incomplete copy from user fail\n");
		goto fail_command;
	}
	if (count > sizeof(tfd->epcmd->cqe) &&
	    (count - sizeof(tfd->epcmd->cqe)) <= tfd->epcmd->buffer_size &&
	    tfd->epcmd->buffer) {
		if (copy_from_user(tfd->epcmd->buffer,
				   buf + sizeof(tfd->epcmd->cqe),
				   count - sizeof(tfd->epcmd->cqe))) {
			dev_err(&epf_nvme->epf->dev,
				"Incomplete copy from user fail on buffer\n");
			goto fail_command;
		}
	}

	dev_dbg(&epf_nvme->epf->dev, "TSP: Completion write from userspace\n");

	pci_epf_nvme_send_cmd_to_next(tfd->epcmd);
	tfd->epcmd = NULL;
	return count;
fail_command:
	tfd->epcmd->status = NVME_SC_INTERNAL;
	pci_epf_nvme_send_cmd_to_completion(tfd->epcmd);
	tfd->epcmd = NULL;
	return -EFAULT;
}

static const struct file_operations tsp_io_fops = {
	.owner		= THIS_MODULE,
	.open		= tsp_open,
	.release	= tsp_release,
	.read		= tsp_read,
	.write		= tsp_write
};

static const struct file_operations tsp_admin_fops = {
	.owner		= THIS_MODULE,
	.open		= tsp_open,
	.release	= tsp_release,
	.read		= tsp_admin_read,
	.write		= tsp_write
};

/* End of char device operations */

static int pci_epf_nvme_set_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	struct pci_epf_bar *epf_bar;
	int bar, add;
	int ret;

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];

		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation requires a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		if (features->reserved_bar & (1 << bar))
			continue;

		if (features->fixed_bar & (1 << bar))
			continue;

		ret = pci_epf_set_bar(epf, epf_bar);
		if (ret) {
			dev_err(&epf->dev, "Failed to set BAR%d\n", bar);
			pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
					   PRIMARY_INTERFACE);
			if (bar == reg_bar)
				return ret;
		}
	}

	/* Init CMB memory management context */
	tsp_mm_init_context(epf_nvme->reg[CMB_BIR],
			    epf_nvme->epf->bar[CMB_BIR].size,
			    &epf_nvme->tsp_alloc_ctx);

	return 0;
}

static int pci_epf_nvme_alloc_reg_bar(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	enum pci_barno reg_bar = epf_nvme->reg_bar;
	size_t reg_size, reg_bar_size;
	size_t msix_table_size = 0;
	void *reg_base;
	int ret;

	/*
	 * Max number of queues is not limited by MSI(X) interrupt numbers.
	 *
	 * CQs are mapped dynamically, one window is used by CQ thread
	 * SQs are mapped dynamically and one window is used by sq_poll thread
	 * xfer threads use a window each, at least one is xfer thread required.
	 *
	 * The number of queues is not limited by the windows
	 */
	epf_nvme->max_nr_queues = PCI_EPF_NVME_MAX_QUEUES;

	dev_info(&epf->dev, "Maximum number of queues: %u\n",
		 epf_nvme->max_nr_queues);

	/*
	 * Calculate the size of the register bar: NVMe registers first with
	 * enough space for the doorbells, followed by the MSIX table
	 * if supported.
	 */
	reg_size = NVME_REG_DBS + ((epf_nvme->max_nr_queues * 2) * sizeof(u32));
	reg_size = ALIGN(reg_size, 8);

	if (features->msix_capable) {
		size_t pba_size;

		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_nvme->msix_table_offset = reg_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);

		reg_size += msix_table_size + pba_size;
	}

	reg_bar_size = ALIGN(reg_size, 4096);

	if (features->bar_fixed_size[reg_bar]) {
		if (reg_bar_size > features->bar_fixed_size[reg_bar]) {
			dev_err(&epf->dev,
				"Reg BAR %d size %llu B too small, need %zu B\n",
				reg_bar,
				features->bar_fixed_size[reg_bar],
				reg_bar_size);
			return -ENOMEM;
		}
		reg_bar_size = features->bar_fixed_size[reg_bar];
	}

	if (features->fixed_bar & (1 << reg_bar)) {
		ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
					    epf->vfunc_no, reg_bar,
					    &epf->bar[reg_bar]);
		if (ret < 0) {
			dev_err(&epf->dev, "Failed to get fixed bar");
			return ret;
		}
		reg_base = epf->bar[reg_bar].addr;
	} else {
		reg_base = pci_epf_alloc_space(epf, reg_bar_size, reg_bar,
					   PAGE_SIZE, PRIMARY_INTERFACE);
	}

	if (!reg_base) {
		dev_err(&epf->dev, "Allocate register BAR failed\n");
		return -ENOMEM;
	}

	epf_nvme->reg[reg_bar] = reg_base;

	//memset_io(epf_nvme->reg[reg_bar], 0, reg_bar_size);
	/* Try clearing only 4k to see if it also fails */
	memset_io(epf_nvme->reg[reg_bar], 0, min(reg_bar_size, (size_t)SZ_4K));

	dev_dbg(&epf->dev,
		"BAR %d, virt addr 0x%llx, phys addr 0x%llx, %zu B\n",
		reg_bar, (u64)epf_nvme->reg[reg_bar],
		epf->bar[reg_bar].phys_addr, epf->bar[reg_bar].size);

	return 0;
}

static int pci_epf_nvme_configure_bars(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *features = epf_nvme->epc_features;
	struct pci_epf_bar *epf_bar;
	int bar, add, ret;
	size_t bar_size;
	void *base;

	/* The first free BAR will be our register BAR */
	bar = pci_epc_get_first_free_bar(features);
	if (bar < 0) {
		dev_err(&epf->dev, "No free BAR\n");
		return -EINVAL;
	}
	epf_nvme->reg_bar = bar;

	/* Initialize BAR flags */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];
		if (features->bar_fixed_64bit & (1 << bar))
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
	}

	/* Allocate the register BAR */
	ret = pci_epf_nvme_alloc_reg_bar(epf);
	if (ret)
		return ret;

	/* Allocate remaining BARs */
	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		if (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64)
			add = 2;
		else
			add = 1;

		/*
		 * Skip the register BAR (already allocated) and
		 * reserved BARs.
		 */
		if (epf_nvme->reg[bar] || features->reserved_bar & (1 << bar))
			continue;

		if (features->fixed_bar & (1 << bar)) {
			ret = pci_epc_get_fixed_bar(epf->epc, epf->func_no,
						    epf->vfunc_no, bar,
						    epf_bar);
			if (ret < 0)
				base = NULL;
			else
				base = epf->bar[bar].addr;
		} else {
			bar_size = max_t(size_t, features->bar_fixed_size[bar], SZ_4K);
			base = pci_epf_alloc_space(epf, bar_size, bar,
					PAGE_SIZE, PRIMARY_INTERFACE);
		}

		if (!base) {
			dev_err(&epf->dev, "Allocate BAR%d failed\n", bar);
			return -ENOMEM;
		}

		epf_nvme->reg[bar] = base;

		/* Don't clear the other BARs for the moment */
		//memset_io(epf_nvme->reg[bar], 0, bar_size);
	}

	return 0;
}

static int pci_epf_nvme_init_irq(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	/* Enable MSIX if supported, otherwise, use MSI */
	if (epf_nvme->epc_features->msix_capable && epf->msix_interrupts) {
		dev_info(&epf->dev, "MSIX capable, %d vectors\n",
			 epf->msix_interrupts);
		ret = pci_epf_set_msix(epf, epf->msix_interrupts,
				       epf_nvme->reg_bar,
				       epf_nvme->msix_table_offset);
		if (ret) {
			dev_err(&epf->dev, "MSI-X configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msix_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSIX;

		return 0;
	}

	if (epf_nvme->epc_features->msi_capable) {
		dev_info(&epf->dev, "MSI capable, %d vectors\n",
			 epf->msi_interrupts);
		ret = pci_epf_set_msi(epf, epf->msi_interrupts);
		if (ret) {
			dev_err(&epf->dev, "MSI configuration failed\n");
			return ret;
		}

		epf_nvme->nr_vectors = epf->msi_interrupts;
		epf_nvme->irq_type = PCI_IRQ_MSI;

		return 0;
	}

	/* MSI and MSIX are not supported. Use INTX */
	epf_nvme->nr_vectors = 1;
	epf_nvme->irq_type = PCI_IRQ_LEGACY;

	return 0;
}

static int pci_epf_nvme_core_init(struct pci_epf *epf)
{
	int ret;

	if (epf->vfunc_no <= 1) {
		/* Set device ID, class, etc */
		ret = pci_epf_write_header(epf, epf->header);
		if (ret) {
			dev_err(&epf->dev,
				"Write configuration header failed %d\n", ret);
			return ret;
		}
	}

	/* Setup the PCIe BARs and enable interrupts */
	ret = pci_epf_nvme_set_bars(epf);
	if (ret)
		return ret;

	ret = pci_epf_nvme_init_irq(epf);
	if (ret)
		return ret;

	/*
	 * Create the fabrics host controller and initialize the attributes
	 * of the PCI controller for the host to see.
	 */
	ret = pci_epf_nvme_create_ctrl(epf);
	if (ret)
		return ret;

	return 0;
}

static int pci_epf_nvme_link_up(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int ret;

	if (!epf_nvme->ctrl.ctrl) {
		/*
		 * We come here if the host was restarted after
		 * shutting down the controller, e.g. a normal reboot.
		 */
		ret = pci_epf_nvme_create_ctrl(epf);
		if (ret)
			return ret;
	}

	/* Start polling the BAR registers to detect controller enable */
	queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
			   msecs_to_jiffies(1));

	return 0;
}

static const struct pci_epf_event_ops pci_epf_nvme_event_ops = {
	.core_init = pci_epf_nvme_core_init,
	.link_up = pci_epf_nvme_link_up,
};

static int pci_epf_nvme_bind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	int ret;

	if (!epc) {
		dev_err(&epf->dev, "No endpoint controller\n");
		return -EINVAL;
	}

	epc_features = pci_epf_get_features(epf);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}
	epf_nvme->epc_features = epc_features;

	ret = pci_epf_nvme_configure_bars(epf);
	if (ret)
		return ret;

	if (!epc_features->core_init_notifier) {
		ret = pci_epf_nvme_core_init(epf);
		if (ret)
			return ret;
	}

	dev_info(&epf->dev, "DMAs %sabled\n",
		 (epf_nvme->dma_enable ? "en" : "dis"));

	epf_nvme->uio_info.mem[0].name = "CMB";
	epf_nvme->uio_info.mem[0].addr = epf_nvme->epf->bar[CMB_BIR].phys_addr;
	dev_dbg(&epf_nvme->epf->dev, "Mem0 addr: %#016llx\n",
		epf_nvme->uio_info.mem[0].addr);
	epf_nvme->uio_info.mem[0].internal_addr = epf_nvme->epf->bar[CMB_BIR].addr;
	epf_nvme->uio_info.mem[0].size = epf_nvme->epf->bar[CMB_BIR].size;
	dev_dbg(&epf_nvme->epf->dev, "Mem0 size: %llu\n",
		epf_nvme->uio_info.mem[0].size);
	epf_nvme->uio_info.mem[0].memtype = UIO_MEM_PHYS;
	epf_nvme->uio_info.version = "0.0.1";
	epf_nvme->uio_info.name = "TSP_NVMe";

	if (uio_register_device(&epf_nvme->epf->dev, &epf_nvme->uio_info))
		dev_err(&epf_nvme->epf->dev, "Could not register UIO device\n");

	if (!epc_features->linkup_notifier && !epc_features->core_init_notifier)
		queue_delayed_work(epf_nvme_reg_wq, &epf_nvme->reg_poll,
				   msecs_to_jiffies(1));

	return 0;
}

static void pci_epf_nvme_unbind(struct pci_epf *epf)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);
	int bar;

	cancel_delayed_work(&epf_nvme->reg_poll);

	pci_epf_nvme_disable_ctrl(epf_nvme, true);

	for (bar = BAR_0; bar < PCI_STD_NUM_BARS; bar++) {
		if (!epf_nvme->reg[bar])
			continue;
		pci_epf_clear_bar(epf, &epf->bar[bar]);
		pci_epf_free_space(epf, epf_nvme->reg[bar], bar,
				   PRIMARY_INTERFACE);
	}
}

static struct pci_epf_header epf_nvme_pci_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.progif_code	= 0x02, /* NVM Express */
	.baseclass_code = PCI_BASE_CLASS_STORAGE,
	.subclass_code	= 0x08, /* Non-Volatile Memory controller */
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static int pci_epf_nvme_probe(struct pci_epf *epf,
			      const struct pci_epf_device_id *id)
{
	int ret, i = 0;
	struct pci_epf_nvme *epf_nvme;
	dev_t dev;

	epf_nvme = devm_kzalloc(&epf->dev, sizeof(*epf_nvme), GFP_KERNEL);
	if (!epf_nvme)
		return -ENOMEM;

	epf_nvme->epf = epf;

	/* Prepare generic transfer thread info if we want to do transfers
	   from within the sq_poll work function */
	epf_nvme->sq_xfer_info.prp_list_buf = kzalloc(NVME_CTRL_PAGE_SIZE,
						      GFP_KERNEL);
	if (!epf_nvme->sq_xfer_info.prp_list_buf)
		return -ENOMEM; /* XXX memory should be freed XXX */
	epf_nvme->sq_xfer_info.dma.dma_supported = false;
	epf_nvme->sq_xfer_info.tid = -1; /* No thread ID, this is special */
	epf_nvme->sq_xfer_info.epf_nvme = epf_nvme;

	INIT_DELAYED_WORK(&epf_nvme->reg_poll, pci_epf_nvme_reg_poll);
	INIT_DELAYED_WORK(&epf_nvme->sq_poll, pci_epf_nvme_sq_poll);
	dev_info(&epf_nvme->epf->dev, "Version with threads and bulk SQEs\n");

	/* Set default attribute values */
	epf_nvme->dma_enable = true;
	epf_nvme->user_path_enable = false;

	epf->event_ops = &pci_epf_nvme_event_ops;
	epf->header = &epf_nvme_pci_header;
	epf_set_drvdata(epf, epf_nvme);

	spin_lock_init(&epf_nvme->xfer_fifo_wr_lock);
	spin_lock_init(&epf_nvme->xfer_fifo_rd_lock);
	ret = kfifo_alloc(&epf_nvme->xfer_fifo, PCI_EPF_NVME_FIFO_SIZE,
			  GFP_KERNEL);
	if (ret)
		return ret; /* XXX memory should be freed XXX */

	spin_lock_init(&epf_nvme->completion_fifo_wr_lock);
	ret = kfifo_alloc(&epf_nvme->completion_fifo, PCI_EPF_NVME_FIFO_SIZE,
		GFP_KERNEL);
	if (ret)
		return ret; /* XXX memory should be freed XXX */

	read_stats = devm_kzalloc(&epf->dev, STATS_BUFFER_SIZE, GFP_KERNEL);
	write_stats = devm_kzalloc(&epf->dev, STATS_BUFFER_SIZE, GFP_KERNEL);
	dev_info(&epf_nvme->epf->dev, "Size of stats buffers: %zu bytes\n",
		 STATS_BUFFER_SIZE);

	if (!read_stats || !write_stats)
		return -ENOMEM;

	INIT_KFIFO(touser_fifo);
	INIT_KFIFO(touser_admin_fifo);

	/* allocate chardev region and assign Major number */
	ret = alloc_chrdev_region(&dev, 0, MAX_DEV, "tspchardev");
	if (ret) {
		dev_err(&epf->dev, "Could not alloc chrdev region\n");
		return ret;
	}

	dev_major = MAJOR(dev);

	/* create sysfs class */
	epf_nvme->tsp_class = class_create("tspchardev");
	if (IS_ERR_OR_NULL(epf_nvme->tsp_class)) {
		dev_err(&epf->dev, "Could not create class\n");
		return PTR_ERR(epf_nvme->tsp_class);
	}

	/* add device to the system where "i" is the Minor number of the new
	 * device */
	for (i = 0; i < MAX_IO_CDEVS; ++i) {
		/* init new device */
		cdev_init(&epf_nvme->tsp_user_queue[i].cdev, &tsp_io_fops);
		epf_nvme->tsp_user_queue[i].cdev.owner = THIS_MODULE;

		ret = cdev_add(&epf_nvme->tsp_user_queue[i].cdev,
			       MKDEV(dev_major, i), 1);
		if (ret < 0) {
			dev_err(&epf->dev, "Could not add character device\n");
			return ret;
		}

		/* create device node /dev/mychardev-x where "x" is "i", equal
		 * to the Minor number */
		device_create(epf_nvme->tsp_class, NULL, MKDEV(dev_major, i),
			      NULL, "tsp-io-%d", i);

		epf_nvme->tsp_user_queue[i].epf_nvme = epf_nvme;
		mutex_init(&epf_nvme->tsp_user_queue[i].mutex);
	}

	for (i = 0; i < MAX_ADMIN_CDEVS; ++i) {
		/* init new device */
		cdev_init(&epf_nvme->tsp_admin_queue[i].cdev, &tsp_admin_fops);
		epf_nvme->tsp_admin_queue[i].cdev.owner = THIS_MODULE;

		ret = cdev_add(&epf_nvme->tsp_admin_queue[i].cdev,
			       MKDEV(dev_major, i + MAX_IO_CDEVS), 1);
		if (ret < 0) {
			dev_err(&epf->dev, "Could not add character device\n");
			return ret;
		}

		/* create device node /dev/mychardev-x where "x" is "i", equal
		 * to the Minor number */
		device_create(epf_nvme->tsp_class, NULL,
			      MKDEV(dev_major, i + MAX_IO_CDEVS),
			      NULL, "tsp-admin-%d", i);

		epf_nvme->tsp_admin_queue[i].epf_nvme = epf_nvme;
		mutex_init(&epf_nvme->tsp_admin_queue[i].mutex);
	}

	return 0;
}

#define to_epf_nvme(epf_group)						\
	container_of((epf_group), struct pci_epf_nvme, group)

static ssize_t pci_epf_nvme_ctrl_opts_show(struct config_item *item,
					   char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	if (!epf_nvme->ctrl_opts_buf)
		return 0;

	return sysfs_emit(page, "%s\n", epf_nvme->ctrl_opts_buf);
}

#define PCI_EPF_NVME_OPT_HIDDEN_NS	"hidden_ns"

static ssize_t pci_epf_nvme_ctrl_opts_store(struct config_item *item,
					    const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	size_t opt_buf_size;

	/* Do not allow setting options when the function is already started */
	if (epf_nvme->ctrl.ctrl)
		return -EBUSY;

	if (!len)
		return -EINVAL;

	if (epf_nvme->ctrl_opts_buf)
		kfree(epf_nvme->ctrl_opts_buf);

	/*
	 * Make sure we have enough room to add the hidden_ns option
	 * if it is missing.
	 */
	opt_buf_size = len + strlen(PCI_EPF_NVME_OPT_HIDDEN_NS) + 2;
	epf_nvme->ctrl_opts_buf = kzalloc(opt_buf_size, GFP_KERNEL);
	if (!epf_nvme->ctrl_opts_buf)
		return -ENOMEM;

	strcpy(epf_nvme->ctrl_opts_buf, page);
	if (!strnstr(page, PCI_EPF_NVME_OPT_HIDDEN_NS, len))
		strncat(epf_nvme->ctrl_opts_buf,
			"," PCI_EPF_NVME_OPT_HIDDEN_NS, opt_buf_size);

	dev_dbg(&epf_nvme->epf->dev,
		"NVMe fabrics controller options: %s\n",
		epf_nvme->ctrl_opts_buf);

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, ctrl_opts);

static ssize_t pci_epf_nvme_dma_enable_show(struct config_item *item,
					    char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%d\n", epf_nvme->dma_enable);
}

static ssize_t pci_epf_nvme_dma_enable_store(struct config_item *item,
					     const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	int ret;

	ret = kstrtobool(page, &epf_nvme->dma_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, dma_enable);

static ssize_t pci_epf_nvme_user_path_enable_show(struct config_item *item,
						  char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%d\n", epf_nvme->user_path_enable);
}

static ssize_t pci_epf_nvme_user_path_enable_store(struct config_item *item,
						   const char *page, size_t len)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);
	int ret;

	ret = kstrtobool(page, &epf_nvme->user_path_enable);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, user_path_enable);

static ssize_t pci_epf_nvme_num_xfer_threads_show(struct config_item *item,
						  char *page)
{
	return sysfs_emit(page, "%d\n", num_xfer_threads);
}

static ssize_t pci_epf_nvme_num_xfer_threads_store(struct config_item *item,
						   const char *page, size_t len)
{
	int ret;

	ret = kstrtoint(page, 10, &num_xfer_threads);
	if (ret)
		return ret;

	if (num_xfer_threads < 1) {
		num_xfer_threads = 1;
		return -EINVAL;
	}

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, num_xfer_threads);

static ssize_t pci_epf_nvme_xfer_fifo_show(struct config_item *item,
					   char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%u\n", kfifo_len(&epf_nvme->xfer_fifo));
}
CONFIGFS_ATTR_RO(pci_epf_nvme_, xfer_fifo);

static ssize_t pci_epf_nvme_completion_fifo_show(struct config_item *item,
						 char *page)
{
	struct config_group *group = to_config_group(item);
	struct pci_epf_nvme *epf_nvme = to_epf_nvme(group);

	return sysfs_emit(page, "%u\n", kfifo_len(&epf_nvme->completion_fifo));
}
CONFIGFS_ATTR_RO(pci_epf_nvme_, completion_fifo);

static ssize_t pci_epf_nvme_do_readwrite_show(struct config_item *item,
					      char *page)
{
	return sysfs_emit(page, "%d\n", do_readwrite);
}

static ssize_t pci_epf_nvme_do_readwrite_store(struct config_item *item,
					       const char *page, size_t len)
{
	int ret;
	ret = kstrtobool(page, &do_readwrite);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, do_readwrite);

static ssize_t pci_epf_nvme_do_transfer_show(struct config_item *item,
					     char *page)
{
	return sysfs_emit(page, "%d\n", do_transfer);
}

static ssize_t pci_epf_nvme_do_transfer_store(struct config_item *item,
					      const char *page, size_t len)
{
	int ret;
	ret = kstrtobool(page, &do_transfer);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, do_transfer);

static ssize_t pci_epf_nvme_do_backend_show(struct config_item *item,
					    char *page)
{
	return sysfs_emit(page, "%d\n", do_backend);
}

static ssize_t pci_epf_nvme_do_backend_store(struct config_item *item,
					     const char *page, size_t len)
{
	int ret;
	ret = kstrtobool(page, &do_backend);
	if (ret)
		return ret;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, do_backend);

static ssize_t pci_epf_nvme_relaxed_poll_threshold_show(struct config_item *item,
							char *page)
{
	return sysfs_emit(page, "%u\n", relaxed_poll_threshold);
}

static ssize_t pci_epf_nvme_relaxed_poll_threshold_store(struct config_item *item,
						   const char *page, size_t len)
{
	int ret;

	ret = kstrtouint(page, 10, &relaxed_poll_threshold);
	if (ret)
		return ret;

	if (relaxed_poll_threshold > PCI_EPF_NVME_FIFO_SIZE) {
		relaxed_poll_threshold = PCI_EPF_NVME_FIFO_SIZE;
		return -EINVAL;
	}

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, relaxed_poll_threshold);

static ssize_t pci_epf_nvme_poll_delay_usecs_show(struct config_item *item,
						  char *page)
{
	return sysfs_emit(page, "%u\n", poll_delay_usecs);
}

static ssize_t pci_epf_nvme_poll_delay_usecs_store(struct config_item *item,
						   const char *page, size_t len)
{
	int ret;

	ret = kstrtouint(page, 10, &poll_delay_usecs);
	if (ret)
		return ret;

	if (poll_delay_usecs > 100000) {
		poll_delay_usecs = 100000;
		return -EINVAL;
	}
	if (poll_delay_usecs < PCI_EPF_NVME_MIN_POLL_DELAY_US) {
		poll_delay_usecs = PCI_EPF_NVME_MIN_POLL_DELAY_US;
		return -EINVAL;
	}

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, poll_delay_usecs);

static ssize_t pci_epf_nvme_poll_relaxed_delay_usecs_show(struct config_item *item,
						  char *page)
{
	return sysfs_emit(page, "%u\n", poll_relaxed_delay_usecs);
}

static ssize_t pci_epf_nvme_poll_relaxed_delay_usecs_store(struct config_item *item,
						   const char *page, size_t len)
{
	int ret;

	ret = kstrtouint(page, 10, &poll_relaxed_delay_usecs);
	if (ret)
		return ret;

	if (poll_relaxed_delay_usecs > 1000000) {
		poll_relaxed_delay_usecs = 1000000;
		return -EINVAL;
	}
	if (poll_relaxed_delay_usecs < PCI_EPF_NVME_MIN_POLL_DELAY_US) {
		poll_delay_usecs = PCI_EPF_NVME_MIN_POLL_DELAY_US;
		return -EINVAL;
	}

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, poll_relaxed_delay_usecs);

static ssize_t pci_epf_nvme_statistics_show(struct config_item *item,
					    char *page)
{
	return sysfs_emit(page, "read stats : %u\n"
				"write stats : %u\n",
			  collected_read_stats, collected_write_stats);
}

static ssize_t pci_epf_nvme_statistics_store(struct config_item *item,
					     const char *page, size_t len)
{
	collected_read_stats = 0;
	collected_write_stats = 0;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, statistics);

static ssize_t pci_epf_nvme_collect_size_filter_show(struct config_item *item,
						     char *page)
{
	return sysfs_emit(page, "%zu\n", collect_size_filter);
}

static ssize_t pci_epf_nvme_collect_size_filter_store(struct config_item *item,
						const char *page, size_t len)
{
	int ret;
	unsigned size;

	ret = kstrtouint(page, 10, &size);
	if (ret)
		return ret;

	collect_size_filter = size;

	return len;
}

CONFIGFS_ATTR(pci_epf_nvme_, collect_size_filter);

ssize_t pci_epf_nvme_statistics_buffer_size_show(struct config_item *item,
						 char* page)
{
	return sysfs_emit(page, "%zu\n", STATS_BUFFER_SIZE);
}

CONFIGFS_ATTR_RO(pci_epf_nvme_, statistics_buffer_size);

ssize_t pci_epf_nvme_rd_statistics_read(struct config_item *item,
					void *buffer, size_t size)
{
	printk("Read rd statistics with %s buffer and size %zu\n",
	       (buffer ? "" : " no "), size);

	if (buffer)
		memcpy(buffer, read_stats, STATS_BUFFER_SIZE);
	return STATS_BUFFER_SIZE;
}

CONFIGFS_BIN_ATTR_RO(pci_epf_nvme_, rd_statistics, NULL, STATS_BUFFER_SIZE);

ssize_t pci_epf_nvme_wr_statistics_read(struct config_item *item,
					void *buffer, size_t size)
{
	printk("Read wr statistics with %s buffer and size %zu\n",
	       (buffer ? "" : " no "), size);

	if (buffer)
		memcpy(buffer, write_stats, STATS_BUFFER_SIZE);
	return STATS_BUFFER_SIZE;
}

CONFIGFS_BIN_ATTR_RO(pci_epf_nvme_, wr_statistics, NULL, STATS_BUFFER_SIZE);

static struct configfs_attribute *pci_epf_nvme_attrs[] = {
	&pci_epf_nvme_attr_ctrl_opts,
	&pci_epf_nvme_attr_dma_enable,
	&pci_epf_nvme_attr_user_path_enable,
	&pci_epf_nvme_attr_num_xfer_threads,
	&pci_epf_nvme_attr_xfer_fifo,
	&pci_epf_nvme_attr_completion_fifo,
	&pci_epf_nvme_attr_do_readwrite,
	&pci_epf_nvme_attr_do_transfer,
	&pci_epf_nvme_attr_do_backend,
	&pci_epf_nvme_attr_relaxed_poll_threshold,
	&pci_epf_nvme_attr_poll_delay_usecs,
	&pci_epf_nvme_attr_poll_relaxed_delay_usecs,
	&pci_epf_nvme_attr_statistics,
	&pci_epf_nvme_attr_collect_size_filter,
	&pci_epf_nvme_attr_statistics_buffer_size,
	NULL,
};

static struct configfs_bin_attribute *pci_epf_nvme_bin_attrs[] = {
	&pci_epf_nvme_attr_rd_statistics,
	&pci_epf_nvme_attr_wr_statistics,
	NULL,
};

static const struct config_item_type pci_epf_nvme_group_type = {
	.ct_attrs	= pci_epf_nvme_attrs,
	.ct_bin_attrs	= pci_epf_nvme_bin_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *pci_epf_nvme_add_cfs(struct pci_epf *epf,
						 struct config_group *group)
{
	struct pci_epf_nvme *epf_nvme = epf_get_drvdata(epf);

	/* Add the NVMe target attributes */
	config_group_init_type_name(&epf_nvme->group, "nvme",
				    &pci_epf_nvme_group_type);

	return &epf_nvme->group;
}

static const struct pci_epf_device_id pci_epf_nvme_ids[] = {
	{ .name = "pci_epf_nvme" },
	{},
};

static struct pci_epf_ops pci_epf_nvme_ops = {
	.bind	= pci_epf_nvme_bind,
	.unbind	= pci_epf_nvme_unbind,
	.add_cfs = pci_epf_nvme_add_cfs,
};

/* XXX TODO Remove function is missing XXX */
static struct pci_epf_driver epf_nvme_driver = {
	.driver.name	= "pci_epf_nvme",
	.probe		= pci_epf_nvme_probe,
	.id_table	= pci_epf_nvme_ids,
	.ops		= &pci_epf_nvme_ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_nvme_init(void)
{
	int ret;

	epf_nvme_reg_wq = alloc_workqueue("epf_nvme_reg",
					  WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_reg_wq)
		return -ENOMEM;

	epf_nvme_sq_wq = alloc_workqueue("epf_nvme_sq",
					 WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!epf_nvme_sq_wq)
		goto out_reg_wq;

	epf_nvme_cmd_cache = kmem_cache_create("epf_nvme_cmd",
					sizeof(struct pci_epf_nvme_cmd),
					0, SLAB_HWCACHE_ALIGN, NULL);
	if (!epf_nvme_cmd_cache)
		goto out_sq_wq;

	ret = pci_epf_register_driver(&epf_nvme_driver);
	if (ret)
		goto out_cache;

	pr_info("Registered driver\n");

	return 0;

out_cache:
	kmem_cache_destroy(epf_nvme_cmd_cache);
out_sq_wq:
	destroy_workqueue(epf_nvme_sq_wq);
out_reg_wq:
	destroy_workqueue(epf_nvme_reg_wq);

	pr_info("Register driver failed\n");

	return ret;
}
module_init(pci_epf_nvme_init);

static void __exit pci_epf_nvme_exit(void)
{
	if (epf_nvme_reg_wq)
		destroy_workqueue(epf_nvme_reg_wq);
	if (epf_nvme_sq_wq)
		destroy_workqueue(epf_nvme_sq_wq);

	pci_epf_unregister_driver(&epf_nvme_driver);

	kmem_cache_destroy(epf_nvme_cmd_cache);

#if 0 /* XXX Should be moved to missing remove function */
	device_destroy(tsp_class, MKDEV(dev_major, i));

	class_unregister(tsp_class);
	class_destroy(tsp_class);

	unregister_chrdev_region(MKDEV(dev_major, 0), MINORMASK);
#endif
	pr_info("Unregistered driver\n");
}
module_exit(pci_epf_nvme_exit);

MODULE_DESCRIPTION("PCI endpoint NVMe function driver");
MODULE_AUTHOR("Damien Le Moal <dlemoal@kernel.org>");
MODULE_AUTHOR("Rick Wertenbroek <rick.wertenbroek@gmail.com");
MODULE_IMPORT_NS(NVME_TARGET_PASSTHRU);
MODULE_IMPORT_NS(NVME_FABRICS);
MODULE_LICENSE("GPL v2");
