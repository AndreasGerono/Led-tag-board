#include <stdint.h>
#include "spi.h"
#include "nrf24l01_defs.h"

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#define SPI_DELAY 255

uint8_t nrf_send_command(uint8_t command);
uint8_t nrf_status_get();
void nrf_status_set(uint8_t mask);
void nrf_status_reset(uint8_t mask);
uint8_t nrf_register_read(uint8_t addr);
void nrf_register_write(uint8_t addr, uint8_t data);
void nrf_register_set(uint8_t addr, uint8_t mask);
void nrf_register_reset(uint8_t addr, uint8_t mask);
void nrf_standby();
void nrf_power_up();
void nrf_power_down();
void nrf_set_addr(uint8_t pipe_addr, uint8_t addr_len, uint8_t *addr);
void nrf_set_tx_power(uint8_t power);
void nrf_tx_mode();
void nrf_rx_mode();
void nrf_write_payload(uint8_t *data, uint16_t len);
void nrf_read_payload(uint8_t *data, uint16_t len);
_Bool nrf_payload_received();
_Bool nrf_rx_fifo_empty();
_Bool nrf_payload_transmitted();
_Bool nrf_tx_fifo_empty();
void nrf_init_multicast(SPI_HandleTypeDef *hspi);

#endif
