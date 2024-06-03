#include "hal_dw_spi.h"
#include "top_reg.h"
#include "intr_conf.h"

static DW_SPIControl_t dw_spi[4];
static struct spi_regs *get_spi_base(uint8_t spi_id)
{
    struct spi_regs *spi_base = NULL;

    switch (spi_id) {
    case SPI0:
        spi_base = (struct spi_regs *)SPI0_BASE;
        break;
    case SPI1:
        spi_base = (struct spi_regs *)SPI1_BASE;
        break;
    case SPI2:
        spi_base = (struct spi_regs *)SPI2_BASE;
        break;
    case SPI3:
        spi_base = (struct spi_regs *)SPI3_BASE;
        break;
    }
    return spi_base;
}

static uint32_t get_spi_intr(uint8_t spi_id)
{
    uint32_t spi_intr = 0;

    switch (spi_id) {
    case SPI0:
        spi_intr = SPI_0_SSI_INTR;
        break;
    case SPI1:
        spi_intr = SPI_1_SSI_INTR;
        break;
    }

    return spi_intr;
}

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static void spi_irq_setup(DW_SPIControl_t *dws) {
    uint32_t txftlr = dws->fifo_len / 2;  // 假设触发发送中断的阈值为FIFO长度的一半
    uint32_t rxftlr = dws->fifo_len / 2;  // 假设触发接收中断的阈值为FIFO长度的一半

    dw_writel(dws, DW_SPI_TXFTLR, txftlr);
    dw_writel(dws, DW_SPI_RXFTLR, rxftlr);

    uint32_t imr = SPI_INT_TXEI | SPI_INT_TXOI | SPI_INT_RXUI | SPI_INT_RXOI |
		SPI_INT_RXFI; 
    uint32_t new_mask = dw_readl(dws, DW_SPI_IMR) | imr;
	dw_writel(dws, DW_SPI_IMR, new_mask);
}

// 初始化SPI硬件
static void spi_hw_init(DW_SPIControl_t *dws, DW_SPICfg_t *cfg) {
    if (dws == NULL || cfg == NULL) {
        return;  // 参数检查
    }

    // 确保SPI设备在配置前是禁用的
    spi_enable(dws, 0);

    // 设置传输模式、数据帧大小和通信频率
    uint32_t ctrl0 = 0;
    ctrl0 |= cfg->tmode >> SPI_CTRL0_CPHA_SHIFT;                 // 设置传输模式
    ctrl0 |= (cfg->dfs - 1) >> SPI_CTRL0_DATA_FREAM_SHIFT;       // 设置数据帧大小，寄存器需要的是长度减1
    dw_writel(dws, DW_SPI_CTRLR0, ctrl0);

    // 设置波特率
    uint32_t baudr = SPI_FREQUENCY / cfg->freq;  // 计算波特率分频值
    if (baudr % 2 != 0)
        baudr++;

    if (baudr < 4 || baudr > 65534)
        baudr = 4;
    dw_writel(dws, DW_SPI_BAUDR, baudr);

    // 启用SPI设备
    spi_enable(dws, 1);
}

// 中断处理函数
static irqreturn_t dw_spi_irq(int irq, void *dev) {
    DW_SPIControl_t *dws = (DW_SPIControl_t *)dev;
    uint16_t irq_status = dw_readl(dws, DW_SPI_ISR) & (SPI_INT_RXFI | SPI_INT_TXEI);

    if (irq_status == 0) return IRQ_NONE;

    if (irq_status & SPI_INT_RXFI) {
        // 从接收 FIFO 读取数据到接收缓冲区
        dw_reader(dws);
    }
    if (irq_status & SPI_INT_TXEI) {
        // 如果还有数据要发送，继续写入发送 FIFO
        dw_writer(dws);
    }

    // 如果所有数据传输完成，禁用中断并通知任务
    if (dws->tx_len == 0 && dws->rx_len == 0) {
        spi_mask_intr(dws->regs, 0xff);  // 禁用所有中断
        xTaskNotifyGiveFromISR(dws->taskHandle, NULL);  // 通知等待的任务
    }

    return IRQ_HANDLED;
}

static int dw_spi_transfer_one(DW_SPIControl_t *dws, SPIMessage_t *msg) {

    dws->tx = msg->txData;
    dws->tx_len = msg->len;
    dws->rx = msg->rxData;
    dws->rx_len = msg->len;

    DW_SPICfg_t cfg = {
        .tmode = SPI_TMOD_TR,
        .dfs = msg->bits_per_word,
        .freq = msg->speed_hz,
    };
    spi_hw_init(dws, &cfg);

    spi_enable(dws, 0);  // 禁用设备以设置中断
    spi_irq_setup(dws);  // 配置中断
    spi_enable(dws, 1);  // 使能设备

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // 等待 ISR 通知传输完成

    return 1;  // 返回传输成功
}

int hw_spi_send(DW_SPIControl_t *spi) {
    uint32_t txflr = mmio_read_32((uintptr_t)&spi->regs->spi_txflr);
    uint32_t max = spi->fifo_len - txflr;
    uint16_t value;

    while (max && (spi->tx < spi->tx + spi->tx_len)) {
        if (spi->cur_msg->bits_per_word == 8)
            value = *(uint8_t *)(spi->tx);
        else
            value = *(uint16_t *)(spi->tx);

        mmio_write_32((uintptr_t)&spi->regs->spi_dr, value);
        spi->tx += spi->cur_msg->bits_per_word >> 3;
        max--;
    }

    return 0;
}

int hw_spi_recv(DW_SPIControl_t *spi) {
    uint32_t rxflr = mmio_read_32((uintptr_t)&spi->regs->spi_rxflr);
    uint32_t tem;

    while (rxflr) {
        tem = mmio_read_32((uintptr_t)&spi->regs->spi_dr);

        if (spi->rx < spi->rx + spi->rx_len) {
            if (spi->cur_msg->bits_per_word == 8)
                *(uint8_t *)(spi->rx) = tem;
            else
                *(uint16_t *)(spi->rx) = tem;
        } else {
            return 0;
        }

        rxflr--;
        spi->rx += spi->cur_msg->bits_per_word >> 3;
    }

    return (int)rxflr;
}


// 用户接口函数，发起一次SPI数据传输
static bool dw_spi_transfer(DW_SPIControl_t *dws, SPIMessage_t *msg)
{
    if (xSemaphoreTake(dws->spiMutex, portMAX_DELAY) == pdTRUE) {
        dws->cur_msg = msg;  // 设置当前传输的消息
        int ret = dw_spi_transfer_one(dws, msg);

        if (ret != 1) {
            // 无法开始传输，可能需要处理错误
        }

        // 完成传输后释放互斥锁
        xSemaphoreGive(dws->spiMutex);
        return (ret == 0);
    } else {
        // 无法获取互斥锁，返回传输失败
        return false;
    }
}

static int dw_spi_transfer_poll(DW_SPIControl_t *spi_dev, SPIMessage_t *message) {
    spi_dev->cur_msg = message;
    spi_dev->tx = message->txData;
    spi_dev->rx = message->rxData;
    spi_dev->tx_len = message->len;
    spi_dev->rx_len = message->len;

    if (message->txData) {
        while (spi_dev->tx < spi_dev->tx + spi_dev->tx_len) {
            hw_spi_send(spi_dev);
            // 等待发送完成
            while (mmio_read_32((uintptr_t)&spi_dev->regs->spi_txflr));
        }
    }

    if (message->rxData) {
        while (spi_dev->rx < spi_dev->rx + spi_dev->rx_len) {
            hw_spi_recv(spi_dev);
        }
    }

    return message->len;
}

void spixfer(DW_SPIControl_t *spi_dev, SPIMessage_t *message){
    if(spi_dev->spi_id < 2){
        dw_spi_transfer(spi_dev, message);
    }
    else{
        dw_spi_transfer_poll(spi_dev, message);
    }
}


// SPI控制器整体初始化
void hal_spi_init(uint8_t spi_id)
{
    // 使用现有函数初始化 SPI 硬件和中断处理
    DW_SPIControl_t *dws = &dw_spi[spi_id];
    dws->regs = get_spi_base(spi_id);
    if (dws->regs == NULL) {
        return;
    }

    // 初始化 SPIControl_t 结构体的其他部分
    dws->spi_id = spi_id;
    dws->fifo_len = SPI_TXFTLR; // 假设 FIFO 长度为 32
    dws->n_bytes = 0;   // 初始化已处理的字节数
    dws->tx = NULL;     // 初始化发送缓冲区指针
    dws->tx_len = 0;    // 初始化待发送数据长度
    dws->rx = NULL;     // 初始化接收缓冲区指针
    dws->rx_len = 0;    // 初始化待接收数据长度
    dws->dfs = 8;       // 每帧数据位数设置为 8 位
    dws->speed_hz = 1000000; // SPI 通信频率设置为 1 MHz

    // 定义用于 SPI 硬件初始化的配置
    DW_SPICfg_t cfg = {
        .tmode = SPI_TMOD_TR,      // 设置为传输和接收模式
        .dfs = dws->dfs,           // 设置数据帧大小
        .freq = dws->speed_hz      // 设置通信频率
    };

    // 创建并初始化互斥锁
    dws->spiMutex = xSemaphoreCreateMutex();
    if (dws->spiMutex == NULL) {
        // 处理错误：互斥锁创建失败
        return;
    }

    // 初始化任务句柄（在实际启动传输前由任务自身设置）
    dws->taskHandle = NULL;

    // 初始化硬件
    spi_hw_init(dws, &cfg);

    if(spi_id < 2){
        // 设置中断
        spi_enable(dws, 0);  // 禁用设备
        spi_irq_setup(dws);  // 配置中断
        spi_enable(dws, 1);  // 使能设备

        // 注册 SPI 中断处理函数
        request_irq(get_spi_intr(spi_id), dw_spi_irq, 0, "SPI_INTR int", dws);
    }
    else{
        spi_enable(dws, 0);  // 禁用设备
        dw_writel(dws, DW_SPI_IMR, SPI_IRQ_MSAK);
        spi_enable(dws, 1);  // 使能设备
    }
    // 可选，启用 SPI 硬件（如果 spi_hw_init 中尚未启用）
    spi_enable(dws->regs, 1);
}
