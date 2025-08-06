/**
 * @file tp_spi.c
 *
 */

/*********************
 *      INCLUDES
 *********************/
#include "tp_spi.h"
#include "touch_driver.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include <string.h>

#include "../lvgl_helpers.h"
#include "../lvgl_spi_conf.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/


/**********************
 *  STATIC PROTOTYPES
 **********************/
static spi_device_handle_t spi;

/**********************
 *  STATIC VARIABLES
 **********************/

/**********************
 *      MACROS
 **********************/

/**********************
 *   GLOBAL FUNCTIONS
 **********************/
void tp_spi_add_device_config(spi_host_device_t host, spi_device_interface_config_t *devcfg)
{
	esp_err_t ret=spi_bus_add_device(host, devcfg, &spi);
	assert(ret==ESP_OK);
}

void tp_spi_add_device(spi_host_device_t host)
{
	spi_device_interface_config_t devcfg={
                .clock_speed_hz = SPI_TOUCH_CLOCK_SPEED_HZ,
                .mode = SPI_TOUCH_SPI_MODE,
		.spics_io_num=TP_SPI_CS,               //CS pin
		.queue_size=1,
		.pre_cb=NULL,
		.post_cb=NULL,
		.command_bits = 8,
		.address_bits = 0,
		.dummy_bits = 0,
		.flags = SPI_DEVICE_NO_DUMMY,
	};
	
	//Attach the Touch controller to the SPI bus
	tp_spi_add_device_config(host, &devcfg);
}

void tp_spi_xchg(uint8_t* data_send, uint8_t* data_recv, uint8_t byte_count)
{
	spi_transaction_t t = {
		.length = byte_count * 8, // SPI transaction length is in bits
		.tx_buffer = data_send,
		.rx_buffer = data_recv};
	
	esp_err_t ret = spi_device_transmit(spi, &t);
	assert(ret == ESP_OK);
}

void tp_spi_write_reg(uint8_t* data, uint8_t byte_count)
{
	spi_transaction_t t = {
	    .length = byte_count * 8,
	    .tx_buffer = data,
	    .flags = 0
	};
	
	esp_err_t ret = spi_device_transmit(spi, &t);
	assert(ret == ESP_OK);
}

void tp_spi_read_reg(uint8_t reg, uint8_t* data, uint8_t byte_count)
{
	uint8_t tx_data[byte_count + 1];
	uint8_t rx_data[byte_count + 1];
	
	tx_data[0] = reg;
	for(int i = 1; i <= byte_count; i++) {
		tx_data[i] = 0; // Send dummy bytes
	}
	
	spi_transaction_t t = {	
	    .length = (byte_count + 1) * 8,
	    .tx_buffer = tx_data,
	    .rx_buffer = rx_data,
	    .flags = 0
	};
	
	esp_err_t ret = spi_device_transmit(spi, &t);
	assert(ret == ESP_OK);
	
	// Copy received data (skip the first byte which is the command echo)
	for(int i = 0; i < byte_count; i++) {
		data[i] = rx_data[i + 1];
	}
}

/**********************
 *   STATIC FUNCTIONS
 **********************/
