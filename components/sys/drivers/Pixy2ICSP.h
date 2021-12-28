//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//
// ICSP, SPI w/o SS

#ifndef _PIXY2SPI_SS_H
#define _PIXY2SPI_SS_H

#include "sdkconfig.h"
//#include "TPixy2.h"
//#include <drivers/gpio.h>

 #include "esp_log.h"

#define CONFIG_SPIBUS_LOG_READWRITES 1
#define CONFIG_SPIBUS_LOG_RW_LEVEL_VERBOSE 1
#define TAG "spirw"
#if defined   CONFIG_SPIBUS_LOG_RW_LEVEL_INFO
#define SPIBUS_LOG_RW(format, ... ) ESP_LOGI(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_SPIBUS_LOG_RW_LEVEL_DEBUG
#define SPIBUS_LOG_RW(format, ... ) ESP_LOGD(TAG, format, ##__VA_ARGS__)
#elif defined CONFIG_SPIBUS_LOG_RW_LEVEL_VERBOSE
#define SPIBUS_LOG_RW(format, ... ) ESP_LOGV(TAG, format, ##__VA_ARGS__)
#endif
#define SPIBUS_LOGE(format, ... )   ESP_LOGE(TAG, format, ##__VA_ARGS__)


#include "driver/spi_common.h"
#include "driver/spi_master.h"
//#include <drivers/spi.h>


#define PIXY_SPI_CLOCKRATE       CONFIG_PIXY2_SPI_CLOCKRATE

#define MISO_PIN 19
#define MOSI_PIN (23)
#define SCLK_PIN  18
//#define PIN_NUM_CS   22


#define SPI_HOST VSPI_HOST
//#define SPI_HOST HSPI_HOST
#define SPI_MODE 3

#define SPIBUS_READ     (0x80)  /*!< addr | SPIBUS_READ  */
#define SPIBUS_WRITE    (0x7F)  /*!< addr & SPIBUS_WRITE */

typedef struct {
	int spi_device; // SPI device
	uint8_t *buff; // Data buffer
	uint32_t len;   // Data buffer length
} spi_userdata;


esp_err_t SPI_begin(int mosi_io_num, int miso_io_num, int sclk_io_num, int max_transfer_sz) {
    spi_bus_config_t config;
    memset(&config, 0, sizeof(config));

    config.mosi_io_num = mosi_io_num;
    config.miso_io_num = miso_io_num;
    config.sclk_io_num = sclk_io_num;
    config.quadwp_io_num = -1;  // -1 not used
    config.quadhd_io_num = -1;  // -1 not used
    config.max_transfer_sz = max_transfer_sz;
    return spi_bus_initialize(SPI_HOST, &config, 0);  // 0 DMA not used
}
esp_err_t SPI_addDevice(uint8_t mode, uint32_t clock_speed_hz, int cs_io_num, spi_device_handle_t *handle) {
    spi_device_interface_config_t dev_config;
    memset(&dev_config, 0, sizeof(dev_config));
    dev_config.command_bits = 0;
    dev_config.address_bits = 8;
    dev_config.dummy_bits = 0;
    dev_config.mode = mode;
    dev_config.duty_cycle_pos = 128;  // default 128 = 50%/50% duty
    dev_config.cs_ena_pretrans = 0;  // 0 not used
    dev_config.cs_ena_posttrans = 0;  // 0 not used
    dev_config.clock_speed_hz = clock_speed_hz;
    dev_config.spics_io_num = cs_io_num;
    dev_config.flags = 0;  // 0 not used
    dev_config.queue_size = 1;
    dev_config.pre_cb = NULL;
    dev_config.post_cb = NULL;
    return spi_bus_add_device(SPI_HOST, &dev_config, handle);
}

esp_err_t SPI_writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data) {
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr & SPIBUS_WRITE;
    transaction.length = length * 8;
    transaction.rxlength = 0;
    transaction.user = NULL;
    transaction.tx_buffer = data;
    transaction.rx_buffer = NULL;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    #if defined CONFIG_SPIBUS_LOG_READWRITES
        if (!err) {
            char str[length*5+1];
            for(size_t i = 0; i < length; i++)
                sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
            SPIBUS_LOG_RW("[%s, handle:0x%X] Write %d bytes to__ register 0x%X, data: %s", (SPI_HOST == 1 ? "HSPI" : "VSPI"), (uint32_t)handle, length, regAddr, str);
        }
    #endif
    return err;
}

esp_err_t SPI_readBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, uint8_t *data) {
    if(length == 0) return ESP_ERR_INVALID_SIZE;
    spi_transaction_t transaction;
    memset(&transaction, 0, sizeof(transaction));
    transaction.flags = 0;
    transaction.cmd = 0;
    transaction.addr = regAddr | SPIBUS_READ;
    transaction.length = length * 8;
    transaction.rxlength = length * 8;
    transaction.user = NULL;
    transaction.tx_buffer = NULL;
    transaction.rx_buffer = data;
    esp_err_t err = spi_device_transmit(handle, &transaction);
    #if defined CONFIG_SPIBUS_LOG_READWRITES
        if (!err) {
            char str[length*5+1];
            for(size_t i = 0; i < length; i++)
            sprintf(str+i*5, "0x%s%X ", (data[i] < 0x10 ? "0" : ""), data[i]);
            SPIBUS_LOG_RW("[%s, handle:0x%X] Read_ %d bytes from register 0x%X, data: %s", (SPI_HOST == 1 ? "HSPI" : "VSPI"), (uint32_t)handle, length, regAddr, str);
        }
    #endif
    return err;
}


class Pixy2Link  {
public:
	int8_t open(uint32_t arg) {


		/*
		driver_error_t *error;

		int id = SPI_ID;
		int is_master = 1;
		int cs = -1; //use default
		uint32_t clock = PIXY_SPI_CLOCKRATE;
		//int data_bits = 8;
		int spi_mode = 3;
		int flags = SPI_FLAG_WRITE | SPI_FLAG_READ;
		error = spi_setup(id, is_master, cs, spi_mode, clock, flags,
				&spi_device);
		if (error==NULL)
			return 0;
		else {
			printf("ICSP SPI init error: %s\n", error->msg);
			return -1;
		}*/

	    //SPI_t &mySPI = vspi;  // vspi and hspi are the default objects
	    //spi_device_handle_t device;
	    ESP_ERROR_CHECK( SPI_begin(MOSI_PIN, MISO_PIN, SCLK_PIN, 0)); //SPI_MAX_DMA_LEN));
	    ESP_ERROR_CHECK( SPI_addDevice(SPI_MODE, PIXY_SPI_CLOCKRATE, -1, &spi_device));

	    return 0;
	}

	void close() {
		spi_bus_free(SPI_HOST);
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL) {
		esp_err_t error;

		ESP_ERROR_CHECK( SPI_readBytes(spi_device, 0, len, buf) );
		/*if ((error = SPI_readBytes(spi_device, 0, len, buf))) {
			return 0;
		}*/

		if (cs) {
			*cs = 0;
			uint8_t i;
			for (i = 0; i < len; i++) {
				*cs += buf[i];
			}
		}

		return len;
	}

	int16_t send(uint8_t *buf, uint8_t len) {
		//    esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);
		//esp_err_t writeBytes(spi_device_handle_t handle, uint8_t regAddr, size_t length, const uint8_t *data);

		esp_err_t error;

		ESP_ERROR_CHECK( SPI_writeBytes(spi_device, 0, len, buf) );
		/*if ((error = SPI_writeBytes(spi_device, 0, len, buf))) {
			return 0;
		}*/

	    return len;
	}

	void setArg(uint16_t arg) {
	}

private:
	spi_device_handle_t  spi_device;
};

#endif
