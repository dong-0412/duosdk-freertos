#ifndef __HAL_DW_SPI_H__
#define __HAL_DW_SPI_H__

#include <stdint.h>
#include <stdbool.h>

enum irqreturn {
	IRQ_NONE		= (0 << 0),
	IRQ_HANDLED		= (1 << 0),
	IRQ_WAKE_THREAD		= (1 << 1),
    IRQ_ERROR       = (1 << 2),
};
typedef enum irqreturn irqreturn_t;
#define BITS_PER_BYTE 8
#define SPI_FREQUENCY 187500000
#define SPI_IRQ_MSAK 0x3e

#define SPI_TXFTLR 0xf

#define SPI_CTRL0_DATA_FREAM_SHIFT 0
#define SPI_CTRL0_FREAM_FORMAT_SHIFT 4
#define SPI_CTRL0_CPHA_SHIFT 6
#define SPI_CTRL0_CPOL_SHIFT 7
#define SPI_CTRL0_TRANS_MODE 8
#define SPI_CTRL0_LOOP_SHIFT 11
#define SPI_CTRL0_CTRL_FREAM_SHIFT 12

#define	SPI_TMOD_TR			0x0		/* xmit & recv */
#define SPI_TMOD_TO			0x1		/* xmit only */
#define SPI_TMOD_RO			0x2		/* recv only */

/* Register offsets */
#define DW_SPI_CTRLR0			0x00
#define DW_SPI_CTRLR1			0x04
#define DW_SPI_SSIENR			0x08
#define DW_SPI_MWCR			0x0c
#define DW_SPI_SER			0x10
#define DW_SPI_BAUDR			0x14
#define DW_SPI_TXFTLR			0x18
#define DW_SPI_RXFTLR			0x1c
#define DW_SPI_TXFLR			0x20
#define DW_SPI_RXFLR			0x24
#define DW_SPI_SR			0x28
#define DW_SPI_IMR			0x2c
#define DW_SPI_ISR			0x30
#define DW_SPI_RISR			0x34
#define DW_SPI_TXOICR			0x38
#define DW_SPI_RXOICR			0x3c
#define DW_SPI_RXUICR			0x40
#define DW_SPI_MSTICR			0x44
#define DW_SPI_ICR			0x48
#define DW_SPI_DMACR			0x4c
#define DW_SPI_DMATDLR			0x50
#define DW_SPI_DMARDLR			0x54
#define DW_SPI_IDR			0x58
#define DW_SPI_VERSION			0x5c
#define DW_SPI_DR			0x60
#define DW_SPI_RX_SAMPLE_DLY		0xf0
#define DW_SPI_CS_OVERRIDE		0xf4

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI			(1 << 0)
#define SPI_INT_TXOI			(1 << 1)
#define SPI_INT_RXUI			(1 << 2)
#define SPI_INT_RXOI			(1 << 3)
#define SPI_INT_RXFI			(1 << 4)
#define SPI_INT_MSTI			(1 << 5)

/* Bit fields in SR, 7 bits */
#define SR_MASK				0x7f		/* cover 7 bits */
#define SR_BUSY				(1 << 0)
#define SR_TF_NOT_FULL			(1 << 1)
#define SR_TF_EMPT			(1 << 2)
#define SR_RF_NOT_EMPT			(1 << 3)
#define SR_RF_FULL			(1 << 4)
#define SR_TX_ERR			(1 << 5)
#define SR_DCOL				(1 << 6)

typedef struct {
    uint8_t *txData;  // 发送数据
    uint32_t len;
    uint8_t *rxData;  // 接收数据
    uint16_t bits_per_word;
    uint32_t speed_hz;
} SPIMessage_t;

typedef struct {
    uint8_t tmode;  // 传输模式
    uint8_t dfs;    // 数据帧大小
    uint32_t freq;  // 通信频率
} DW_SPICfg_t;


struct spi_regs {
    uint32_t spi_ctrl0;         // 0x00
    uint32_t spi_ctrl1;         // 0x04
    uint32_t spi_ssienr;        // 0x08
    uint32_t spi_mwcr;          // 0x0c
    uint32_t spi_ser;           // 0x10
    uint32_t spi_baudr;         // 0x14
    uint32_t spi_txftlr;        // 0x18
    uint32_t spi_rxftlr;        // 0x1c
    uint32_t spi_txflr;         // 0x20
    uint32_t spi_rxflr;         // 0x24
    uint32_t spi_sr;            // 0x28
    uint32_t spi_imr;           // 0x2c
    uint32_t spi_isr;           // 0x30
    uint32_t spi_risr;          // 0x34
    uint32_t spi_txoicr;        // 0x38
    uint32_t spi_rxoicr;        // 0x3c
    uint32_t spi_rxuicr;        // 0x40
    uint32_t spi_msticr;        // 0x44
    uint32_t spi_icr;           // 0x48
    uint32_t spi_dmacr;         // 0x4c
    uint32_t spi_dmatdlr;       // 0x50
    uint32_t spi_dmardlr;       // 0x54
    uint32_t spi_idr;           // 0x58
    uint32_t spi_version;       // 0x5c
    uint32_t spi_dr;            // 0x60
    uint32_t spi_rx_sample_dly; // 0xf0
    uint32_t spi_cs_override;   // 0xf4
};

typedef struct {
    uint8_t spi_id;
    struct spi_regs *regs;
    uint32_t fifo_len;
    uint8_t n_bytes;
    uint8_t *tx;
    uint32_t tx_len;
    uint8_t *rx;
    uint32_t rx_len;
    uint32_t dfs;
    uint32_t speed_hz;
    SPIMessage_t *cur_msg;
    SemaphoreHandle_t spiMutex;
    TaskHandle_t taskHandle;
} DW_SPIControl_t;


static void dw_reader(DW_SPIControl_t *dws) {
    while (dws->rx_len > 0 && (mmio_read_32((uintptr_t)&dws->regs->spi_sr) & SR_RF_NOT_EMPT)) {
        *dws->rx++ = mmio_read_32((uintptr_t)&dws->regs->spi_dr);
        dws->rx_len--;
    }
}

static void dw_writer(DW_SPIControl_t *dws) {
    while (dws->tx_len > 0 && (mmio_read_32((uintptr_t)&dws->regs->spi_sr) & SR_TF_NOT_FULL)) {
        mmio_write_32((uintptr_t)&dws->regs->spi_dr, *dws->tx++);
        dws->tx_len--;
    }
}

// 实现写寄存器函数
static inline void dw_writel(DW_SPIControl_t *dws, uint32_t reg_offset, uint32_t value) {
    // 将值写入到指定的寄存器偏移量
    mmio_write_32((uintptr_t)((char*)dws->regs + reg_offset), value);
}

// 实现读寄存器函数
static inline uint32_t dw_readl(DW_SPIControl_t *dws, uint32_t reg_offset) {
    // 从指定的寄存器偏移量读取32位值
    return mmio_read_32((uintptr_t)((char*)dws->regs + reg_offset));
}

static inline void spi_enable(struct spi_regs *reg, uint32_t enable)
{
    if (enable)
        enable = 0x1;
    else
        enable = 0x0;

    mmio_write_32((uintptr_t)&reg->spi_ssienr, enable);
}

static inline void spi_mask_intr(struct spi_regs *reg, uint32_t mask)
{
	uint32_t new_mask;

	new_mask = mmio_read_32((uintptr_t)&reg->spi_imr) & ~mask;
	mmio_write_32((uintptr_t)&reg->spi_imr, new_mask);
}


void hal_spi_init(uint8_t spi_id);
void spixfer(DW_SPIControl_t *spi_dev, SPIMessage_t *message);

#endif