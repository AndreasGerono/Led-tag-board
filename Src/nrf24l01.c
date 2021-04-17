#include "nrf24l01.h"
#include "gpio.h"

#define SPI_DELAY 255
#define RF_CHANNEL 0x11

static SPI_HandleTypeDef *spi;

inline static void ce_set(void)
{
    NRF_CE_GPIO_Port->BSRR = NRF_CE_Pin;
}

inline static void ce_reset(void)
{
    NRF_CE_GPIO_Port->BRR = NRF_CE_Pin;
}

inline static void ss_reset(void)
{
    SPI2_SS_GPIO_Port->BRR = SPI2_SS_Pin;
}

inline static void ss_set(void)
{
    SPI2_SS_GPIO_Port->BSRR = SPI2_SS_Pin;
}


uint8_t nrf_send_command(uint8_t command)
{
    uint8_t status = 0;
    HAL_SPI_TransmitReceive(spi, &command, &status, 1, SPI_DELAY);
    return status;
}

uint8_t nrf_status_get()
{
    uint8_t status;
    uint8_t command = NOP;
    ss_reset();
    HAL_SPI_TransmitReceive(spi, &command, &status, 1, SPI_DELAY);
    ss_set();
    return status;
}

void nrf_status_set(uint8_t mask)
{
    ss_reset();
    uint8_t status = nrf_send_command(W_REGISTER | STATUS);
    status |= mask;
    HAL_SPI_Transmit(spi, &status, 1, SPI_DELAY);
    ss_set();
}

void nrf_status_reset(uint8_t mask)
{
    ss_reset();
    uint8_t status = nrf_send_command(W_REGISTER | STATUS);
    status &= ~mask;
    HAL_SPI_Transmit(spi, &status, 1, SPI_DELAY);
    ss_set();
}

uint8_t nrf_register_read(uint8_t addr)
{
    uint8_t data;
    ss_reset();
    nrf_send_command(R_REGISTER | addr);
    HAL_SPI_Receive(spi, &data, 1, SPI_DELAY);
    ss_set();
    return data;
}

void nrf_register_write(uint8_t addr, uint8_t data)
{
    ss_reset();
    nrf_send_command(W_REGISTER | addr);
    HAL_SPI_Transmit(spi, &data, 1, SPI_DELAY);
    ss_set();
}

void nrf_register_set(uint8_t addr, uint8_t mask)
{
    uint8_t data;
    data = nrf_register_read(addr);
    data |= mask;
    nrf_register_write(addr, data);
}

void nrf_register_reset(uint8_t addr, uint8_t mask)
{
    uint8_t data;
    data = nrf_register_read(addr);
    data &= ~mask;
    nrf_register_write(addr, data);
}

void nrf_standby()
{
    ce_reset();
    HAL_Delay(1);
}

void nrf_power_up()
{
    nrf_register_set(CONFIG, PWR_UP);
    HAL_Delay(2);                        /* Start up 1.5ms */
}

void nrf_power_down()
{
    nrf_register_reset(CONFIG, PWR_UP);
}


void nrf_set_addr(uint8_t pipe_addr, uint8_t addr_len, uint8_t *addr)
{
    ss_reset();
    nrf_send_command(W_REGISTER | pipe_addr);
    HAL_SPI_Transmit(spi, addr, addr_len, SPI_DELAY);
    ss_set();
}

void nrf_set_ard(uint8_t ard)
{
    ard = (ard << 4);
    uint8_t retr_setup = nrf_register_read(SETUP_RETR);
    retr_setup &= ~ARD;
    retr_setup |= ard & ~ARD;
    nrf_register_write(SETUP_RETR, retr_setup);
}

void nrf_set_arc(uint8_t arc)
{
    uint8_t retr_setup = nrf_register_read(SETUP_RETR);
    retr_setup &= ~ARC;
    retr_setup |= arc & ~ARC;
    nrf_register_write(SETUP_RETR, retr_setup);
}

void nrf_set_tx_power(uint8_t power)
{
    power = (power << 1);
    uint8_t rf_setup = nrf_register_read(RF_SETUP);
    rf_setup &= ~RF_PWR;
    rf_setup |= power & ~RF_PWR;
    nrf_register_write(RF_SETUP, rf_setup);
}

void nrf_set_data_rate(uint8_t data_rate)
{
    data_rate = (data_rate << 3);
    uint8_t rf_setup = nrf_register_read(RF_SETUP);
    rf_setup &= ~(RF_DR_HIGH | RF_DR_LOW);
    rf_setup |= data_rate & (RF_DR_HIGH | RF_DR_LOW);
    nrf_register_write(RF_SETUP, rf_setup);
}

/*
 * NRF will go to tx only if fifo not empty else to standby II
 */

void nrf_tx_mode()
{
    nrf_standby();
    nrf_status_set(TX_DS | RX_DR);
    nrf_register_reset(CONFIG, PRIM_RX);
    ce_set();
}

void nrf_rx_mode()
{
    nrf_standby();
    nrf_status_set(TX_DS | RX_DR);
    ss_reset();
    nrf_send_command(FLUSH_RX);
    ss_set();
    nrf_register_set(CONFIG, PRIM_RX);
    ce_set();
}

void nrf_write_payload(uint8_t *data, uint16_t len)
{
    ss_reset();
    nrf_send_command(W_TX_PAYLOAD);
    HAL_SPI_Transmit(spi, data, len, SPI_DELAY);
    ss_set();
}

void nrf_read_payload(uint8_t *data, uint16_t len)
{
    ss_reset();
    nrf_send_command(R_RX_PAYLOAD);
    HAL_SPI_Receive(spi, data, len, SPI_DELAY);
    ss_set();
}

_Bool nrf_payload_transmitted()
{
    uint8_t status = nrf_status_get();
    if (status & TX_DS) {
        nrf_status_set(TX_DS);
        return 1;
    }
    return 0;
}

_Bool nrf_tx_fifo_empty()
{
    uint8_t fifo_status = nrf_register_read(FIFO_STATUS);
    return fifo_status & FIFO_TX_EMPTY;
}

/*
 * Can set interrupts gpio for receiving
 */

_Bool nrf_payload_received()
{
    uint8_t status = nrf_status_get();
    if (status & RX_DR) {
        nrf_status_set(RX_DR);
        return 1;
    }
    return 0;
}

_Bool nrf_rx_fifo_empty()
{
    uint8_t fifo_status = nrf_register_read(FIFO_STATUS);
    return fifo_status & FIFO_RX_EMPTY;
}

void nrf_init_multicast(SPI_HandleTypeDef *hspi)
{
    spi = hspi;
    ss_set();
    uint8_t multicast_addr[] = {100, 100, 100};

    HAL_Delay(100);                                 /* Power on reset 100ms  */
    HAL_SPI_Init(spi);
    
    nrf_register_write(SETUP_RETR, 0);              /* Turn off retransmission */
    nrf_register_write(EN_AA, 0);                   /* Turn off auto ack for all data pipes */
    nrf_register_write(SETUP_AW, AW_LEN3);          /* Set addr width */
    nrf_register_write(RF_CH, RF_CHANNEL);
    nrf_register_write(RX_PW_P0, 1);                /* Set payload pipe0 as 1 byte */
    nrf_register_write(EN_RXADDR, ERX_P0);			/* Enable data pipe 0 */

    nrf_register_reset(CONFIG, EN_CRC);             /* CRC off for */
    nrf_set_data_rate(DR_250KBS);

    nrf_set_addr(RX_ADDR_P0, 3, multicast_addr);    /* set rx addr */
    nrf_set_addr(TX_ADDR, 3, multicast_addr);       /* set tx addr */
}
