#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/sched.h>
#include <linux/spi/spi.h>
#include <linux/errno.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/compat.h>
#include <linux/spi/spidev.h>

#include <asm/uaccess.h>

#include <mach/hardware.h>
#include <mach/wmt_gpio.h>
#include <mach/wmt-spi.h>



#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>

#include <mach/hardware.h>
#include <mach/wmt_gpio.h>

//#define  DEBUG  1   /* debug open */
#include <linux/platform_device.h>



//config the spi1 as slave for test

#define GPIO_SPI1_SS0   BIT7
#define GPIO_SPI1_MOSI  BIT3
#define GPIO_SPI1_MISO  BIT2
#define GPIO_SPI1_CLK   BIT6

#define GPIO_SPI1_SS1 BIT7
#define GPIO_SPI1_SS2 BIT0
#define GPIO_SPI1_SS3 BIT1

#define SPIDEV_MAJOR			153	/* assigned */
#define N_SPI_MINORS			32	/* ... up to 256 */

static DECLARE_BITMAP(minors, N_SPI_MINORS);


/* Bit masks for spi_device.mode management.  Note that incorrect
 * settings for some settings can cause *lots* of trouble for other
 * devices on a shared bus:
 *
 *  - CS_HIGH ... this device will be active when it shouldn't be
 *  - 3WIRE ... when active, it won't behave as it should
 *  - NO_CS ... there will be no explicit message boundaries; this
 *	is completely incompatible with the shared bus model
 *  - READY ... transfers may proceed when they shouldn't.
 *
 * REVISIT should changing those flags be privileged?
 */
#define SPI_MODE_MASK		(SPI_CPHA | SPI_CPOL | SPI_CS_HIGH \
				| SPI_LSB_FIRST | SPI_3WIRE | SPI_LOOP \
				| SPI_NO_CS | SPI_READY)


struct wmtspidev_data {
    dev_t			devt;
	spinlock_t		spi_lock;
	struct spi_device	*spi;
	struct list_head	device_entry;

	/* buffer is NULL unless this device is open (users > 0) */
	struct mutex		buf_lock;
	unsigned		users;

	struct wmt_spi_dma *wmtspidev_dma_info;
	u8 *buffer;
	u8 *devmem;
	u8  memindex;
	
	void __iomem *regs_base;

	
};

static u64 wmt_spidev_dma_mask = 0xFFFFFFFFUL;

static struct wmt_spi_hw wmt_spidev_info = {
	/* spi on wmt can support dma */
	.dma_support       = SPI_DMA_ENABLE,
	/* can support 4 slaves when wmt spi as master */
	.num_chipselect    = MAX_SPI_SLAVE,
	/* wmt spi support 16bits_per_word? i'm not sure */
	.bits_per_word_en  = BITS8_PER_WORD_EN,
	/* wmt spi can support multi-master also, but it seems we do not need it */
	.port_mode         = PORT_MODE_PTP,
	/* ssn driven low when enable */
	.ssn_ctrl          = SSN_CTRL_HARDWARE,
	/* actual 36bytes, but we use 32bytes */
	.fifo_size         = SPI_FIFO_SIZE,
	/* 4Kbytes, same as the DMA */
	.max_transfer_length = SPI_MAX_TRANSFER_LENGTH,
	/* it's really needed? i'm not sure   */
	.min_freq_hz       = SPI_MIN_FREQ_HZ,
	/* max freq 100Mhz */
	.max_freq_hz       = SPI_MAX_FREQ_HZ,
};


/*
 * spi_set_reg32 - write a u32 value to spi register
 * @spi: spi controller's driver data
 * @reg_offset: register's offset address
 * @val: value register will be set
 */
static inline void 
spi_set_reg32(struct wmtspidev_data *spi, u32 reg_offset, u32 val)
{
	iowrite32(val, spi->regs_base + reg_offset);  
}

/*
 * spi_get_reg32 - read a u32 value from spi register
 * @spi: spi controller's driver data
 * @reg_offset: register's offset address
 */
static inline unsigned int 
spi_get_reg32(struct wmtspidev_data *spi, int reg_offset)
{
	return ioread32(spi->regs_base + reg_offset);
}

/*
 * spi_setbit: write bit1 to related register's bit
 * @spi: spi controller's driver data
 * @offset: register's offset address
 * @mask: bit setting mask
 */
static void
spi_setbit(struct wmtspidev_data *spi, u32 reg_offset, u32 mask)
{
	u32 tmp;
	tmp  = spi_get_reg32(spi, reg_offset);
	tmp |= mask;
	spi_set_reg32(spi, reg_offset, tmp);
}

/*
 * spi_clrbit: write bit0 to related register's bit
 * @spi: spi controller's driver data
 * @offset: register's offset address
 * @mask: bit setting mask
 */
static void
spi_clrbit(struct wmtspidev_data *spi, u32 reg_offset, u32 mask)
{
	u32 tmp;
	tmp  = spi_get_reg32(spi, reg_offset);
	tmp &= ~mask;
	spi_set_reg32(spi, reg_offset, tmp);
}

/*
 * spi_write_fifo: write a u8 value to spi tx fifo
 * @spi: spi controller's driver data
 * @fifo_reg: spi tx fifo register offset
 * @val: value writen to spi tx fifo
 */
static inline void 
spi_write_fifo(struct wmtspidev_data *spi, u32 fifo_reg, const u8 val)
{
	iowrite8(val, spi->regs_base + fifo_reg);
}

/*
 * spi_read_fifo: read a u8 value from spi rx fifo
 * @spi: spi controller's driver data
 * @fifo_reg: spi rx fifo register offset
 */
static inline u8 
spi_read_fifo(struct wmtspidev_data *spi, u32 fifo_reg)
{
	return ioread8(spi->regs_base + fifo_reg);
}


static void
wmtdev_spi_fifo_tx(struct wmtspidev_data *spi, const u8 *tx_buf, int len)
{
	int i;
	/* load data to tx fifo */
	if (tx_buf) {
		for (i = 0; i < len; i++)
			spi_write_fifo(spi, SPI_TXFIFO, tx_buf[i]);
	} /* load idle data to tx fifo */else {
		for (i = 0; i < len; i++)
			spi_write_fifo(spi, SPI_TXFIFO, 0x00);
	}
}

static inline void wmt_spi1_gpio_set(void)
{
    GPIO_CTRL_GP18_UART_BYTE_VAL &= ~(GPIO_SPI1_CLK |
                                      GPIO_SPI1_SS0 |
			                          GPIO_SPI1_MISO |
			                          GPIO_SPI1_MOSI);
    PULL_EN_GP18_UART_BYTE_VAL |= (GPIO_SPI1_CLK |
                                   GPIO_SPI1_SS0 |
			                       GPIO_SPI1_MISO |
			                       GPIO_SPI1_MOSI);

	PULL_CTRL_GP18_UART_BYTE_VAL |= (GPIO_SPI1_SS0 |
			                         GPIO_SPI1_MISO |
			                         GPIO_SPI1_MOSI);			                         
    PULL_CTRL_GP18_UART_BYTE_VAL &= ~GPIO_SPI1_CLK;


    GPIO_CTRL_GP0_BYTE_VAL &= ~(GPIO_SPI1_SS1);
    PULL_EN_GP0_BYTE_VAL |= GPIO_SPI1_SS1;
    PULL_CTRL_GP0_BYTE_VAL |= GPIO_SPI1_SS1;
    

    GPIO_CTRL_GP1_BYTE_VAL &=  ~(GPIO_SPI1_SS2 | GPIO_SPI1_SS3);
	PULL_EN_GP1_BYTE_VAL |= (GPIO_SPI1_SS2 | GPIO_SPI1_SS3);
	PULL_CTRL_GP1_BYTE_VAL |= (GPIO_SPI1_SS2 | GPIO_SPI1_SS3);
    
    PIN_SHARING_SEL_4BYTE_VAL |= BIT10;		

    auto_pll_divisor(DEV_SPI1, CLK_ENABLE, 0, 0);
	auto_pll_divisor(DEV_SPI1, SET_DIV, 1, 100000);
}




/*
 * spi_is_busy: check spi controller(master) is busy or not 
 * @spi: spi controller's driver data
 */
static int spi_is_busy(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	while (timeout--) {
		if ((spi_get_reg32(spi, SPI_SR) & SPI_SR_BUSY_MASK) == 0)
			return 0;
	}
	return -EAGAIN;  
}

/*
 * spi_enable: enable spi module
 * @spi: spi controller's driver data
 */
static void spi_enable(struct wmtspidev_data *spi)
{
	spi_setbit(spi, SPI_CR, SPI_CR_ME_MASK);
}

/*
 * spi_disable: disable spi module
 * @spi: spi controller's driver data
 */
static void spi_disable(struct wmtspidev_data *spidev)
{
	if (spi_is_busy(spidev)) {
		dev_dbg(&spidev->spi->dev, "Disable spi controller failed\n");
	}
	spi_clrbit(spidev, SPI_CR, SPI_CR_ME_MASK);
}

/*
 * spi_tx_is_finish: polling if data in tx fifo has been sent out 
 * @spi: spi controller's driver data
 */
static int spi_tx_is_finish(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	while (timeout--) {
		if (spi_get_reg32(spi, SPI_SR) | SPI_SR_TFEI_MASK)
			return 0;
	}
	return -EAGAIN;
}


/*
 * spi_rx_is_finish: polling if data in rx fifo has been read out 
 * @spi: spi controller's driver data
 */
static int spi_rx_is_finish(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	while (timeout--) {
		if ((spi_get_reg32(spi, SPI_SR) | SPI_SR_RFEI_MASK))
			return 0;
	}
	return -EAGAIN;
}

static int spi_rx_is_full(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	while (timeout--) {
		if ((spi_get_reg32(spi, SPI_SR) | SPI_SR_RFFI_MASK))
			return 0;
	}
	return -EAGAIN;
}

static int spi_get_rx_cnt(struct wmtspidev_data *spi)
{

    unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	while (timeout--);
	return ((spi_get_reg32(spi, SPI_SR) & 0xFFFF0000) >> 24);
}

/*
 * spi_set_bit_order: set spi redeive/transmit significant bit order
 * @spi: spi controller's driver data
 * @mode: spi device working mode
 */
static void spi_set_bit_order(struct wmtspidev_data *spi, u8 mode)
{
	if (mode & SPI_LSB_FIRST)
		spi_setbit(spi, SPI_DFCR, SPI_DFCR_RSBO_MASK | SPI_DFCR_TSBO_MASK);
}

/*
 * spi_as_master: spi master/slave select
 * @spi: spi controller's driver data
 * @is_master: if spi configured as master, is_master = 1, else is_master = 0
 */
static void spi_as_slave(struct wmtspidev_data *spi)
{
	spi_setbit(spi, SPI_CR, SPI_CR_MSMS_MASK);
}

/*
 * spi_reset_tx_fifo: reset spi transmit fifo
 * @spi: spi controller's driver data
 */
static int spi_reset_tx_fifo(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	spi_setbit(spi, SPI_CR, SPI_CR_TFR_MASK);
	while (timeout--) {
		if ((spi_get_reg32(spi, SPI_CR) & SPI_CR_TFR_MASK) == 0)
        		return 0;
	}
	return -1;
}

/*
 * spi_reset_rx_fifo: reset spi receive fifo
 * @spi: spi controller's driver data
 */
static int spi_reset_rx_fifo(struct wmtspidev_data *spi)
{
	unsigned int timeout = POLLING_SPI_REG_TIMEOUT;
	spi_setbit(spi, SPI_CR, SPI_CR_RFR_MASK);
	while (timeout--) {
		if ((spi_get_reg32(spi, SPI_CR) & SPI_CR_RFR_MASK) == 0)
        		return 0;
	}
	return -EAGAIN;
}

/*
 * spi_reset_fifo: reset both spi transmit fifo and receive fifo
 * @spi: spi controller's driver data
 */
static int spi_reset_fifo(struct wmtspidev_data *spi)
{
	if (spi_reset_tx_fifo(spi))
		return -EAGAIN;
	if (spi_reset_rx_fifo(spi))
		return -EAGAIN;
	return 0;
}

/*
 * spi_reset: reset spi status register and reset spi tx and rx fifo
 * @spi: spi controller's driver data
 */
static int spi_reset(struct wmtspidev_data *spi)
{
	//spi_set_reg32(spi, SPI_SR, ~0UL);
	/* clear spi status register                */
	spi_as_slave(spi);
	spi_set_reg32(spi, SPI_SR, 0x7F10);
	spi_set_reg32(spi, SPI_DFCR, BIT4|BIT5|BIT2);
	/* spi cre register */
	spi_set_reg32(spi, SPI_CRE, 0x20);
	return spi_reset_fifo(spi);
}

/*
 * spi_set_clock_mode: set spi clock polarity and phase (spi clock mode)
 * @spi: spi controller's driver data
 * @clk_mode: spi clock mode
 */
static void spi_set_clock_mode(struct wmtspidev_data *spi, u8 clk_mode)
{
	spi_clrbit(spi, SPI_CR, SPI_CR_CPHS_MASK);
	if (clk_mode > SPI_CLK_MODE3)
		goto err;
	spi_setbit(spi, SPI_CR, clk_mode << SPI_CR_CPHS_SHIFT);
	return ;
err:
	spi_setbit(spi, SPI_CR, SPI_CLK_MODE3 << SPI_CR_CPHS_SHIFT);
	dev_err(&spi->spi->dev, "clock mode err, set clock mode 3 as default\n");
	return ;
}


static inline void wmt_spi1_setup(struct wmtspidev_data *spi)
{
    u8 clockmode;
    struct wmt_spi_slave *slave_info;

    clockmode = (spi->spi->mode & (BIT0 | BIT1));
	slave_info = spi->spi->controller_data;
	
    /* clear spi control register               */
	spi_set_reg32(spi, SPI_CR, 0x400000UL);
	/* clear spi status register                */
	spi_set_reg32(spi, SPI_SR, 0x7F10);
	spi_set_reg32(spi, SPI_DFCR, BIT4|BIT5|BIT2);
	/* spi cre register */
	//spi_set_reg32(spi, SPI_CRE, 0x20);

	/* reset tx and rx fifo                     */
	spi_reset_fifo(spi);

    //spi_set_clock_mode(spi, clockmode);

	
	spi_as_slave(spi);	
    //if (SSN_CTRL_PROGRAM == slave_info->ssn_ctrl) {
	//	if ((clockmode & SPI_CR_CPHS_MASK) == 0) {
	//		dev_warn(&spi->spi->dev, "SSN_ctrl conflict with clock mode\n");
			/* do not abort now, the conflict is not a serious problem, 
			   driver can handle this well, so we work on */
	//	}
	//	spi_setbit(spi, SPI_DFCR, SPI_DFCR_SC_MASK);
	//}	
}


static int wmt_spidev_dma_init(struct wmtspidev_data *spi)
{
	int ret  = 0;
	unsigned int dma_size;
	struct wmt_spi_slave *slave_info;
    
	slave_info = spi->spi->controller_data;

	if (!slave_info->dma_en)
		goto out;

	spi->wmtspidev_dma_info = kmalloc(sizeof(struct wmt_spi_dma), GFP_KERNEL);
	if (!spi->wmtspidev_dma_info) {
		ret = -ENOMEM;
		dev_err(&spi->spi->dev, "SPI allocating dma info memory failed\n");
		goto out;
	}
	dma_size = slave_info->max_transfer_length;
	/* dma read config */
	spi->wmtspidev_dma_info->rx_ch = ~0UL;
	spi->wmtspidev_dma_info->rx_config.ChunkSize     = SPI_DMA_CHUNK_SIZE;
	spi->wmtspidev_dma_info->rx_config.DefaultCCR    = SPI_RX_DMA_CFG;
	spi->wmtspidev_dma_info->rx_config.DeviceReqType = SPI1_DMA_RX_REQ;
	spi->wmtspidev_dma_info->rx_config.MIF1addr      = 0xD8250000 + SPI_RXFIFO;
	//dev_info(&spi->spi->dev, "MIF1 addr %x", spi->wmtspidev_dma_info->rx_config.MIF1addr);
	spi->wmtspidev_dma_info->io_raddr = dma_alloc_coherent(&spi->spi->dev, 
					                    dma_size,
					                    &spi->wmtspidev_dma_info->phys_raddr, 
					                    GFP_KERNEL | GFP_DMA);
	if (!spi->wmtspidev_dma_info->io_raddr) {
		ret = -ENOMEM;
		dev_err(&spi->spi->dev, "SPI allocate rdma failed\n");
		goto out;
	}
	memset(spi->wmtspidev_dma_info->io_raddr, 0x00, dma_size);
	spi->wmtspidev_dma_info->rx_config.MIF0addr = (ulong)spi->wmtspidev_dma_info->io_raddr;
	init_waitqueue_head(&spi->wmtspidev_dma_info->rx_event);
	spi->wmtspidev_dma_info->rx_ack = 0;
	/* dma write config */
	spi->wmtspidev_dma_info->tx_ch = ~0UL;
	spi->wmtspidev_dma_info->tx_config.ChunkSize     = SPI_DMA_CHUNK_SIZE;
	spi->wmtspidev_dma_info->tx_config.DefaultCCR    = SPI_TX_DMA_CFG;
	spi->wmtspidev_dma_info->tx_config.DeviceReqType = SPI1_DMA_TX_REQ;
	spi->wmtspidev_dma_info->tx_config.MIF1addr      = 0xD8250000 + SPI_TXFIFO;
	spi->wmtspidev_dma_info->io_waddr = dma_alloc_coherent(&spi->spi->dev, 
					dma_size + 7,
					&spi->wmtspidev_dma_info->phys_waddr,
					GFP_KERNEL | GFP_DMA);
	if (!spi->wmtspidev_dma_info->io_waddr) {
		ret = -ENOMEM;
		dev_err(&spi->spi->dev, "SPI allocate wdma failed\n");
		goto free_spi_rx_dma;
	}
	memset(spi->wmtspidev_dma_info->io_waddr, 0x00, dma_size + 7);
	spi->wmtspidev_dma_info->tx_config.MIF0addr = (ulong)spi->wmtspidev_dma_info->io_waddr;
	init_waitqueue_head(&spi->wmtspidev_dma_info->tx_event);
	spi->wmtspidev_dma_info->tx_ack = 0;
	dev_info(&spi->spi->dev, "<<< wmt spidev dma init finish");
	return 0;

free_spi_rx_dma:
	dma_free_coherent(&spi->spi->dev,dma_size,spi->wmtspidev_dma_info->io_raddr,
			   spi->wmtspidev_dma_info->phys_raddr);
out:
    
    return ret;
}

static void wmt_spidev_dma_release(struct wmtspidev_data *spi)
{
    struct wmt_spi_slave *slave_info;
	slave_info = spi->spi->controller_data;

	if (!slave_info->dma_en)
		goto out;

	dma_free_coherent(&spi->spi->dev, 
		    slave_info->max_transfer_length, 
		    spi->wmtspidev_dma_info->io_raddr,
		    spi->wmtspidev_dma_info->phys_raddr);
	dma_free_coherent(&spi->spi->dev,
		    slave_info->max_transfer_length + 7, 
		    spi->wmtspidev_dma_info->io_waddr,
		    spi->wmtspidev_dma_info->phys_waddr);
out:
	return ;
}

/*
 * wmt_spi_cs_active: enable chip select signal. SSN is driven 
 *    low to active if spi controller use SSN_CTRL_PROGRAM mode
 * @spi: spi controller's driver data
 */
static inline void wmt_spidev_cs_active(struct wmtspidev_data *spi)
{
	struct wmt_spi_slave *slave_info;
	slave_info = spi->spi->controller_data;

	/* enable SSN */
	if (slave_info->ssn_ctrl == SSN_CTRL_PROGRAM)
		spi_clrbit(spi, SPI_DFCR, SPI_DFCR_DSV_MASK | SPI_DFCR_DSE_MASK);
}

/*
 * wmt_spi_cs_inactive: disable chip select signal. A SSN is driven
 *   high to inactive if spi controller use SSN_CTRL_PROGRAM mode
 * @spi: spi controller's driver data
 */
static inline void wmt_spidev_cs_inactive(struct wmtspidev_data *spi)
{
    struct wmt_spi_slave *slave_info;
	slave_info = spi->spi->controller_data;

	if (slave_info->ssn_ctrl == SSN_CTRL_PROGRAM)
		spi_setbit(spi, SPI_DFCR, SPI_DFCR_DSV_MASK | SPI_DFCR_DSE_MASK);
}


/*
 * spi_dsr_w: spi dma transmit callback function
 * @arg: point to wmt_spi
 **/
static void spidev_dsr_w(void *arg)
{
	struct wmtspidev_data *spi = (struct wmtspidev_data *)arg;
	struct wmt_spi_dma *spi_dma = spi->wmtspidev_dma_info;
	spi_dma->tx_ack = 1;
	wake_up_interruptible(&spi_dma->tx_event);
}

/*
 * spi_dsr_r: spi dma receive callback function
 * @arg: point to wmt_spi
 **/
static void spidev_dsr_r(void *arg)
{
	struct wmtspidev_data *spi = (struct wmtspidev_data *)arg;
	struct wmt_spi_dma *spi_dma = spi->wmtspidev_dma_info;
	spi_dma->rx_ack = 1;
	wake_up_interruptible(&spi_dma->rx_event);
}


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);

static unsigned bufsiz = 4096;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");

/*-------------------------------------------------------------------------*/

/*
 * We can't use the standard synchronous wrappers for file I/O; we
 * need to protect against async removal of the underlying spi_device.
 */
static void spidev_complete(void *arg)
{
	complete(arg);
}

static ssize_t
spidev_sync(struct wmtspidev_data *spidev, struct spi_message *message)
{
	DECLARE_COMPLETION_ONSTACK(done);
	int status;

	message->complete = spidev_complete;
	message->context = &done;

	spin_lock_irq(&spidev->spi_lock);
	if (spidev->spi == NULL)
		status = -ESHUTDOWN;
	else
		status = spi_async(spidev->spi, message);
	spin_unlock_irq(&spidev->spi_lock);

	if (status == 0) {
		wait_for_completion(&done);
		status = message->status;
		if (status == 0)
			status = message->actual_length;
	}
	return status;
}

static inline ssize_t
spidev_sync_write(struct wmtspidev_data *spidev, size_t len)
{
    

	size_t transfering;
	size_t transfered = 0;
	u8 * buf;
	buf = spidev->devmem + spidev->memindex;
	
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer + spidev->memindex,
			.len		= len,
		};
	struct spi_message	m;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	//wmt_spi1_setup(spidev);
	spi_enable(spidev);
	
	transfering = spidev_sync(spidev, &m);
	
    if (transfering) {

		
        if(transfering != spi_get_rx_cnt(spidev)){
            //dev_info(&spidev->spi->dev, "err transfering %x", spi_get_rx_cnt(spidev));
            //dev_info(&spidev->spi->dev, "write reg 0 %x", spi_get_reg32(spidev, 0));
		    //dev_info(&spidev->spi->dev, "write reg 4 %x", spi_get_reg32(spidev, 4));
		    //dev_info(&spidev->spi->dev, "write reg 8 %x", spi_get_reg32(spidev, 8));
            goto writedone;
        }
	    for (; transfering > 0; transfering--)
		{
		    *buf = spi_read_fifo(spidev, SPI_RXFIFO);
			dev_info(&spidev->spi->dev, "rx %x", *buf);
    		buf++;
    		transfered++;
    		spidev->memindex ++;
		}
	} 
writedone:    	
    spi_disable(spidev);
	spi_reset_fifo(spidev);
	spi_set_reg32(spidev, SPI_SR, 0x7F10);
	return transfered;

}

static inline ssize_t
spidev_sync_dma_write(struct wmtspidev_data *spidev, size_t len)
{
    struct wmt_spi_slave *slave_info;
	slave_info = spidev->spi->controller_data;

	struct wmt_spi_dma *spi_dma = spidev->wmtspidev_dma_info;
	wait_queue_head_t *event = &spi_dma->rx_event;
	volatile int *ack = &spi_dma->rx_ack;

	u32 ctrl;
	u32 dfcr;
	size_t transfering = 0;
	int i;

    //dev_info(&spidev->spi->dev, ">>> wmt spidev dma write");
    
	struct spi_transfer	t = {
			.tx_buf		= spidev->buffer + spidev->memindex,
			.len		= len,
		};
	struct spi_message	m;

    u8 * buf;
	buf = spidev->devmem + spidev->memindex;

	spi_message_init(&m);
	//m.is_dma_mapped = 1;
	spi_message_add_tail(&t, &m);

    /* spi dma transfer need cs inactive first*/
	//wmt_spidev_cs_inactive(spidev);
    spi_reset(spidev);
	ctrl = spi_get_reg32(spidev, SPI_CR);
	ctrl |= SPI_CR_DRC_MASK | SPI_CR_RFTS_MASK;
	spi_set_reg32(spidev, SPI_CR, ctrl);  
	dfcr = spi_get_reg32(spidev, SPI_DFCR) | BIT28;
	spi_set_reg32(spidev, SPI_DFCR, dfcr);
	/* reset spi fifo */
	//spi_reset(spidev);

    //spi_enable(spidev);

	//dev_info(&spidev->spi->dev, ">>> wmt spidev request dma ");
	if (wmt_request_dma(&spi_dma->rx_ch, "wmt_spidev_rx",
				  spi_dma->rx_config.DeviceReqType,
				  spidev_dsr_r, spidev)) {
			dev_err(&spidev->spi->dev, "SPI request RX DMA failed\n");
			goto writedone;
		}
	wmt_setup_dma(spi_dma->rx_ch, spi_dma->rx_config);
	wmt_start_dma(spi_dma->rx_ch, spi_dma->phys_raddr, 0x00, len);
	event = &spi_dma->rx_event;
	ack = &spi_dma->rx_ack;
    //dev_info(&spidev->spi->dev, "MIF1 addr %x", spidev->wmtspidev_dma_info->rx_config.MIF1addr);
    
    spi_enable(spidev);    
    /* enable spi and active chipselect signal */
	//msleep(2);
    //spi_enable(spidev);
	/* waitting for transmit finish */
	msleep(2);
	//wmt_spidev_cs_active(spidev);
	
    transfering = spidev_sync(spidev, &m);
	if(transfering)
    {
        memcpy(buf, spi_dma->io_raddr, transfering);
        //for(i = 0; i < transfering; i++)
        //    dev_info(&spidev->spi->dev, "transfered %x, rx %x", transfering, *(buf+i));
		wmt_free_dma(spi_dma->rx_ch);
		memset(spi_dma->io_raddr, 0x00, len);
	}
	
writedone:    
    //dev_info(&spidev->spi->dev, ">>> wmt spidev dma write OK");
	spi_disable(spidev);	
	spi_dma->rx_ack = 0;
	spi_reset_fifo(spidev);
	spi_set_reg32(spidev, SPI_SR, 0x7F10);

	return transfering;
}

static inline ssize_t
spidev_sync_read(struct wmtspidev_data *spidev, size_t len)
{

	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer + spidev->memindex,
			.len		= len,
		};
	struct spi_message	m;

	
	size_t transfering = len;
	size_t transfered = 0;
	u8 * buf;
	buf = spidev->devmem + spidev->memindex;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
    
    
	for (; transfering > 0; transfering--)
	{
	  
		spi_write_fifo(spidev, SPI_TXFIFO, *buf);
		//dev_info(&spidev->spi->dev, "tx %x", *buf);
	    buf++;
		spidev->memindex ++;
	}

	spi_enable(spidev);

	transfering = spidev_sync(spidev, &m);
	

	if (spi_tx_is_finish(spidev))
	{
    	//dev_info(&spidev->spi->dev, "error, transferd %d", transfered);
    	spi_disable(spidev);
        spi_reset_fifo(spidev);
		goto readdone;
	}
	//dev_info(&spidev->spi->dev, "read reg 0 %x", spi_get_reg32(spidev, 0));
    //dev_info(&spidev->spi->dev, "read reg 4 %x", spi_get_reg32(spidev, 4));
	//dev_info(&spidev->spi->dev, "read reg 8 %x", spi_get_reg32(spidev, 8));
	transfered = len;		

readdone:    
	spi_disable(spidev);
	
	spi_reset_fifo(spidev);
	spi_set_reg32(spidev, SPI_SR, 0x7F10);

	return transfered;
}

static inline ssize_t
spidev_sync_dma_read(struct wmtspidev_data *spidev, size_t len)
{
    struct wmt_spi_slave *slave_info;
	slave_info = spidev->spi->controller_data;

	struct wmt_spi_dma *spi_dma = spidev->wmtspidev_dma_info;
	wait_queue_head_t *event = &spi_dma->tx_event;
	volatile int *ack = &spi_dma->tx_ack;
	u32 ctrl;
	u32 dfcr;
	size_t i;

	struct spi_transfer	t = {
			.rx_buf		= spidev->buffer + spidev->memindex,
			.len		= len,
		};
	struct spi_message	m;

	size_t transfering = len;
	u8 * buf;
	buf = spidev->devmem + spidev->memindex;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
    /* spi dma transfer need cs inactive first*/
	//wmt_spidev_cs_inactive(spidev);

	spi_reset(spidev);
	ctrl = spi_get_reg32(spidev, SPI_CR);
	ctrl |= SPI_CR_DRC_MASK | SPI_CR_TFTS_MASK;
	spi_set_reg32(spidev, SPI_CR, ctrl);  
	dfcr = spi_get_reg32(spidev, SPI_DFCR) | BIT28;
	spi_set_reg32(spidev, SPI_DFCR, dfcr);

	
	/* tx dma buffer prepare */
	memcpy(spi_dma->io_waddr, buf, len);
	/* tx dma request */
	if (wmt_request_dma(&spi_dma->tx_ch, "wmt_spidev_tx", 
	                    spi_dma->tx_config.DeviceReqType, 
			            spidev_dsr_w, spidev)) {
		dev_err(&spidev->spi->dev, "SPI request TX DMA failed\n");
		goto readdone;                        
	}
    
    /* transmit dma setup and start */
	wmt_setup_dma(spi_dma->tx_ch, spi_dma->tx_config);
	wmt_start_dma(spi_dma->tx_ch, spi_dma->phys_waddr, 0x00, len + 7);
	/* enable spi and active chipselect signal */
	msleep(2);
	spi_enable(spidev);
	/* waitting for transmit finish */
	msleep(2);
	//wmt_spidev_cs_active(spidev);
	/* waitting transfer finish */
    transfering = spidev_sync(spidev, &m);
	if (!wait_event_interruptible_timeout(*event, *ack, 100)) {
		dev_err(&spidev->spi->dev, "SPIDEV DMA transfer failed\n");
		goto readdone;
	};
	
    wmt_free_dma(spi_dma->tx_ch);
	memset(spi_dma->io_waddr, 0x00, len + 7);  
	
readdone:    
	spi_disable(spidev);
	spi_dma->tx_ack = 0;
	spi_reset_fifo(spidev);
	spi_set_reg32(spidev, SPI_SR, 0x7F10);

	return transfering;
}

/*-------------------------------------------------------------------------*/

/* Read-only message with current device setup */
static ssize_t
spidev_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
	struct wmtspidev_data	*spidev;
	ssize_t			status = 0;
	int i;
    struct wmt_spi_slave *slave_info;

	
	
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;
    spidev->memindex += *f_pos;
    

	slave_info = spidev->spi->controller_data;


	mutex_lock(&spidev->buf_lock);
	if(slave_info->dma_en)
	{
	    status = spidev_sync_dma_read(spidev, count);
	}
	else
	{
	    while(count >= SPI_FIFO_SIZE)
	    {
	        status += spidev_sync_read(spidev, SPI_FIFO_SIZE);
	        count -= SPI_FIFO_SIZE;
	        //dev_info(&spidev->spi->dev, "status= :%d", status);
	    }
	    if(count)
	    {
	        status += spidev_sync_read(spidev, count);
	    }
	    spidev->memindex = 0;
	}
	//dev_info(&spidev->spi->dev, "status= :%d", status);
	
    for(i = 0; i < status; i++)
    {
	    dev_info(&spidev->spi->dev, "buff :%x", *(spidev->buffer + i));
    }	

    if (status > 0) {
		unsigned long	missing;
        missing = copy_to_user(buf, spidev->buffer, status);		
	    if (missing == status)
		   	status = -EFAULT;
    	else
	    	status = status - missing;	   
        for(i = 0; i < missing; i++)
        {
		    dev_info(&spidev->spi->dev, "buff :%x", *(buf + i));
        }	    	
	}
	
	mutex_unlock(&spidev->buf_lock);

	return status;
}

/* Write-only message with current device setup */
static ssize_t
spidev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	struct wmtspidev_data	*spidev;
	ssize_t			status = 0;
	unsigned long		missing;
	struct wmt_spi_slave *slave_info;

    
	/* chipselect only toggles at start or end of operation */
	if (count > bufsiz)
		return -EMSGSIZE;

	spidev = filp->private_data;
	slave_info = spidev->spi->controller_data;
	//dev_info(&spidev->spi->dev, ">>> wmt spidev write");

	
	mutex_lock(&spidev->buf_lock);
	
	missing = copy_from_user(spidev->buffer, buf, count);
	spidev->memindex += *f_pos;
	
	if (missing == 0) {
	    if(slave_info->dma_en)
	    {
	        //dev_info(&spidev->spi->dev, ">>> wmt spidev support dma");
	        status = spidev_sync_dma_write(spidev, count);
	    }
	    else
	    {
            while(count >= SPI_FIFO_SIZE)
            {
		        status += spidev_sync_write(spidev, SPI_FIFO_SIZE);
		        count -= SPI_FIFO_SIZE;
		    }
		    if(count)
		    {
		        status += spidev_sync_write(spidev, count);
		    }
		    spidev->memindex = 0;
		}
	} 
	else
		status = -EFAULT;
	
	mutex_unlock(&spidev->buf_lock);

	return status;
}

static int spidev_message(struct wmtspidev_data *spidev,
		struct spi_ioc_transfer *u_xfers, unsigned n_xfers)
{
	struct spi_message	msg;
	struct spi_transfer	*k_xfers;
	struct spi_transfer	*k_tmp;
	struct spi_ioc_transfer *u_tmp;
	unsigned		n, total;
	u8			*buf;
	int			status = -EFAULT;


	spi_message_init(&msg);
	k_xfers = kcalloc(n_xfers, sizeof(*k_tmp), GFP_KERNEL);
	if (k_xfers == NULL)
		return -ENOMEM;

	/* Construct spi_message, copying any tx data to bounce buffer.
	 * We walk the array of user-provided transfers, using each one
	 * to initialize a kernel version of the same transfer.
	 */
	buf = spidev->buffer;
	total = 0;
	for (n = n_xfers, k_tmp = k_xfers, u_tmp = u_xfers;
			n;
			n--, k_tmp++, u_tmp++) {
		k_tmp->len = u_tmp->len;

		total += k_tmp->len;
		if (total > bufsiz) {
			status = -EMSGSIZE;
			goto done;
		}

		if (u_tmp->rx_buf) {
			k_tmp->rx_buf = buf;
			if (!access_ok(VERIFY_WRITE, (u8 __user *)
						(uintptr_t) u_tmp->rx_buf,
						u_tmp->len))
				goto done;
		}
		if (u_tmp->tx_buf) {
			k_tmp->tx_buf = buf;
			if (copy_from_user(buf, (const u8 __user *)
						(uintptr_t) u_tmp->tx_buf,
					u_tmp->len))
				goto done;
			
		}
		buf += k_tmp->len;

		k_tmp->cs_change = !!u_tmp->cs_change;
		k_tmp->bits_per_word = u_tmp->bits_per_word;
		k_tmp->delay_usecs = u_tmp->delay_usecs;
		k_tmp->speed_hz = u_tmp->speed_hz;
#ifdef VERBOSE
		dev_dbg(&spidev->spi->dev,
			"  xfer len %zd %s%s%s%dbits %u usec %uHz\n",
			u_tmp->len,
			u_tmp->rx_buf ? "rx " : "",
			u_tmp->tx_buf ? "tx " : "",
			u_tmp->cs_change ? "cs " : "",
			u			u_tmp->bits_per_word ? : spidev->spi->bits_per_word,
			u_tmp->delay_usecs,
			u_tmp->speed_hz ? : spidev->spi->max_speed_hz);
#endif
		spi_message_add_tail(k_tmp, &msg);
	}

	status = spidev_sync(spidev, &msg);
	if (status < 0)
		goto done;

	/* copy any rx data out of bounce buffer */
	
	
	buf = spidev->buffer;
	for (n = n_xfers, u_tmp = u_xfers; n; n--, u_tmp++) {
		if (u_tmp->rx_buf) {
			if (__copy_to_user((u8 __user *)
					(uintptr_t) u_tmp->rx_buf, buf,
					u_tmp->len)) {
				status = -EFAULT;
				goto done;
			}
			
		}
		buf += u_tmp->len;
	}
	status = total;

done:
	kfree(k_xfers);
	return status;
}

static long
spidev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int			err = 0;
	int			retval = 0;
	struct wmtspidev_data	*spidev;
	struct spi_device	*spi;
	u32			tmp;
	unsigned		n_ioc;
	struct spi_ioc_transfer	*ioc;

	/* Check type and command number */
	if (_IOC_TYPE(cmd) != SPI_IOC_MAGIC)
		return -ENOTTY;

	/* Check access direction once here; don't repeat below.
	 * IOC_DIR is from the user perspective, while access_ok is
	 * from the kernel perspective; so they look reversed.
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	/* guard against device removal before, or while,
	 * we issue this ioctl.
	 */
	spidev = filp->private_data;
	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);

	if (spi == NULL)
		return -ESHUTDOWN;

	/* use the buffer lock here for triple duty:
	 *  - prevent I/O (from us) so calling spi_setup() is safe;
	 *  - prevent concurrent SPI_IOC_WR_* from morphing
	 *    data fields while SPI_IOC_RD_* reads them;
	 *  - SPI_IOC_MESSAGE needs the buffer locked "normally".
	 */
	mutex_lock(&spidev->buf_lock);

	switch (cmd) {
	/* read requests */
	case SPI_IOC_RD_MODE:
		retval = __put_user(spi->mode & SPI_MODE_MASK,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_LSB_FIRST:
		retval = __put_user((spi->mode & SPI_LSB_FIRST) ?  1 : 0,
					(__u8 __user *)arg);
		break;
	case SPI_IOC_RD_BITS_PER_WORD:
		retval = __put_user(spi->bits_per_word, (__u8 __user *)arg);
		break;
	case SPI_IOC_RD_MAX_SPEED_HZ:
		retval = __put_user(spi->max_speed_hz, (__u32 __user *)arg);
		break;

	/* write requests */
	case SPI_IOC_WR_MODE:
		retval = __get_user(tmp, (u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp & ~SPI_MODE_MASK) {
				retval = -EINVAL;
				break;
			}

			tmp |= spi->mode & ~SPI_MODE_MASK;
			spi->mode = (u8)tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "spi mode %02x\n", tmp);
		}
		break;
	case SPI_IOC_WR_LSB_FIRST:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->mode;

			if (tmp)
				spi->mode |= SPI_LSB_FIRST;
			else
				spi->mode &= ~SPI_LSB_FIRST;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->mode = save;
			else
				dev_dbg(&spi->dev, "%csb first\n",
						tmp ? 'l' : 'm');
		}
		break;
	case SPI_IOC_WR_BITS_PER_WORD:
		retval = __get_user(tmp, (__u8 __user *)arg);
		if (retval == 0) {
			u8	save = spi->bits_per_word;

			spi->bits_per_word = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->bits_per_word = save;
			else
				dev_dbg(&spi->dev, "%d bits per word\n", tmp);
		}
		break;
	case SPI_IOC_WR_MAX_SPEED_HZ:
		retval = __get_user(tmp, (__u32 __user *)arg);
		if (retval == 0) {
			u32	save = spi->max_speed_hz;

			spi->max_speed_hz = tmp;
			retval = spi_setup(spi);
			if (retval < 0)
				spi->max_speed_hz = save;
			else
				dev_dbg(&spi->dev, "%d Hz (max)\n", tmp);
		}
		break;

	default:
		/* segmented and/or full-duplex I/O request */
		if (_IOC_NR(cmd) != _IOC_NR(SPI_IOC_MESSAGE(0))
				|| _IOC_DIR(cmd) != _IOC_WRITE) {
			retval = -ENOTTY;
			break;
		}

		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			break;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		if (n_ioc == 0)
			break;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			break;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			break;
		}

		/* translate to spi_message, execute */
		retval = spidev_message(spidev, ioc, n_ioc);
		kfree(ioc);
		break;
	}

	mutex_unlock(&spidev->buf_lock);
	spi_dev_put(spi);
	return retval;
}

#ifdef CONFIG_COMPAT
static long
spidev_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return wmtspidev_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#else
#define spidev_compat_ioctl NULL
#endif /* CONFIG_COMPAT */

static int spidev_open(struct inode *inode, struct file *filp)
{
	struct wmtspidev_data	*spidev;
	int			status = -ENXIO;

	mutex_lock(&device_list_lock);

	list_for_each_entry(spidev, &device_list, device_entry) {
		if (spidev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}
	if (status == 0) {
		if (!spidev->buffer) {
			spidev->buffer = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		
        if (!spidev->devmem) {
			spidev->devmem = kmalloc(bufsiz, GFP_KERNEL);
			if (!spidev->devmem) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			}
		}
		
		if (status == 0) {
			spidev->users++;
			filp->private_data = spidev;
			nonseekable_open(inode, filp);
		}
	} 
	else
		pr_debug("spidev: nothing for minor %d\n", iminor(inode));
    spi_enable(spidev);
	mutex_unlock(&device_list_lock);
	return status;
}

static int spidev_release(struct inode *inode, struct file *filp)
{
	struct wmtspidev_data	*spidev;
	int			status = 0;

	mutex_lock(&device_list_lock);
	spidev = filp->private_data;
	filp->private_data = NULL;
    
	/* last close? */
	spidev->users--;
	if (!spidev->users) {
		int		dofree;
		
        iounmap(spidev->regs_base);
        
		kfree(spidev->buffer);
		spidev->buffer = NULL;

		kfree(spidev->devmem);
		spidev->devmem = NULL;

		/* ... after we unbound from the underlying device? */
		spin_lock_irq(&spidev->spi_lock);
		dofree = (spidev->spi == NULL);
		spin_unlock_irq(&spidev->spi_lock);

		if (dofree)
			kfree(spidev);
	}
	
		
	mutex_unlock(&device_list_lock);

	return status;
}
static const struct file_operations wmtspidev_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	 * gets more complete API coverage.  It'll simplify things
	 * too, except for the locking.
	 */
	.write =	spidev_write,
	.read =		spidev_read,
	.unlocked_ioctl = spidev_ioctl,
	.compat_ioctl = spidev_compat_ioctl,
	.open =		spidev_open,
	.release =	spidev_release,
	.llseek =	no_llseek,
};
/*-------------------------------------------------------------------------*/

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */

static struct class *wmtspidev_class;

/*-------------------------------------------------------------------------*/

static int __devinit spidev_probe(struct spi_device *spi)
{
	struct wmtspidev_data	*spidev;
	int			status;
	unsigned long		minor;
    struct wmt_spi_slave *slave_info;

	
    dev_info(&spi->dev, "wmtspidev init");

	/* Allocate driver data */
	spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);
	if (!spidev)
		return -ENOMEM;

	/* Initialize the driver data */
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);

	INIT_LIST_HEAD(&spidev->device_entry);

	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */
	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;

		spidev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(wmtspidev_class, &spi->dev, spidev->devt,
				    spidev, "spidev%d.%d",
				    spi->master->bus_num, spi->chip_select);

		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}

    spidev->regs_base = (void __iomem *)SPI1_BASE_ADDR;
	if (!spidev->regs_base) {
	    dev_dbg(&spi->dev, "SPI ioremap failed!\n");
		status = -ENXIO;
	}
	
	
    

	if (status == 0) {
    	spi->dev.coherent_dma_mask = ~0;
		spi->dev.dma_mask = &wmt_spidev_dma_mask;
		spi->dev.platform_data = &wmt_spidev_info;
        slave_info = spidev->spi->controller_data;
        if (slave_info->dma_en) {
		    if (wmt_spidev_dma_init(spidev)) {
			    status = -ENXIO;
			    slave_info->dma_en = 0;
			    dev_err(&spi->dev, "SPI dma init failed\n");
			    goto release_ioremap;
		    }          
	    }		
	    set_bit(minor, minors);
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	if (status == 0)
		spi_set_drvdata(spi, spidev);
	else
	{
		kfree(spidev);
		goto release_dma;
	}
    wmt_spi1_gpio_set();
    //dev_info(&spi->dev, "%x %x %x", spi_get_reg32(spidev, 0), spi_get_reg32(spidev, 4), spi_get_reg32(spidev, 8));
    wmt_spi1_setup(spidev);
    spi_enable(spidev);
    //dev_info(&spi->dev, "%x %x %x", spi_get_reg32(spidev, 0), spi_get_reg32(spidev, 4), spi_get_reg32(spidev, 8));
    return status;
    
release_dma:
	wmt_spidev_dma_release(spidev);
release_ioremap:
	iounmap(spidev->regs_base);
	return status;
}

static int __devexit spidev_remove(struct spi_device *spi)
{
	struct wmtspidev_data	*spidev = spi_get_drvdata(spi);

    
	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	
	spidev->spi = NULL;
	spi_set_drvdata(spi, NULL);
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	list_del(&spidev->device_entry);
	device_destroy(wmtspidev_class, spidev->devt);
	clear_bit(MINOR(spidev->devt), minors);
	if (spidev->users == 0)
		kfree(spidev);
    /* release requested resource         */
		
	mutex_unlock(&device_list_lock);

	return 0;
}


/*-------------------------------------------------------------------------*/

static struct spi_driver wmtspidev_spi_driver = {
	.driver = {
		.name		= "spidev",
		.owner		= THIS_MODULE,
	},
	.probe		= spidev_probe,
	.remove		= __devexit_p(spidev_remove),
};


/*-------------------------------------------------------------------------*/

static int __init spidev_init(void)
{
	int status;

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, "spi", &wmtspidev_fops);
	if (status < 0)
		return status;

	wmtspidev_class = class_create(THIS_MODULE, "spidev");
	if (IS_ERR(wmtspidev_class)) {
		unregister_chrdev(SPIDEV_MAJOR, wmtspidev_spi_driver.driver.name);
		return PTR_ERR(wmtspidev_class);
	}

	status = spi_register_driver(&wmtspidev_spi_driver);
	if (status < 0) {
		class_destroy(wmtspidev_class);
		unregister_chrdev(SPIDEV_MAJOR, wmtspidev_spi_driver.driver.name);
	}

    
	
	return status;
}

module_init(spidev_init);

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&wmtspidev_spi_driver);
	class_destroy(wmtspidev_class);
	unregister_chrdev(SPIDEV_MAJOR, wmtspidev_spi_driver.driver.name);
}
module_exit(spidev_exit);

MODULE_AUTHOR("Sand Fan, <sandfan@viatech.com.cn>");
MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("spi:spidev");

