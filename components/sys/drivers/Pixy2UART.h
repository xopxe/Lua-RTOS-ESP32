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
// Arduino SPI with slave select link class

#ifndef _PIXY2UART_H
#define _PIXY2UART_H

//#include "TPixy2.h"
//#include <drivers/gpio.h>
//#include <drivers/spi.h>
//#include <drivers/cpu.h>

//#include <drivers/uart.h> //whitecat
#include <driver/uart.h> //esp-idf
#include "sdkconfig.h"

//#define PIXY_SPI_CLOCKRATE       CONFIG_PIXY2_SPI_CLOCKRATE
//#define PIXY_UART (CPU_UART1)
#define PIXY_UART_NUM (UART_NUM_2)
#define BUF_SIZE (1024)


typedef struct {
	int spi_device; // SPI device
	uint8_t *buff; // Data buffer
	uint32_t len;   // Data buffer length
} spi_userdata;

class Pixy2Link  {
public:
	int8_t open(uint32_t arg) {
		/*driver_error_t *error;
		int flags = UART_FLAG_WRITE | UART_FLAG_READ;

		int id = PIXY_UART;
	    int bauds = 19200;
	    int databits = 8;
	    int parity = 0;
	    int stop_bits = 1;
	    int buffer = 1024;


	    error = uart_init(id, bauds, databits, parity, stop_bits, flags, buffer);
		if (error!=NULL) return -1;

	    error = uart_setup_interrupts(id);
		if (error!=NULL) return -1;*/
/*
	    uart_config_t uart_config = {
	        .baud_rate = 19200,
	        .data_bits = UART_DATA_8_BITS,
	        .parity    = UART_PARITY_DISABLE,
	        .stop_bits = UART_STOP_BITS_1,
	        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	        //.source_clk = UART_SCLK_APB,
	    };
	    */


	    switch (PIXY_UART_NUM) {
	    	case 0: periph_module_enable(PERIPH_UART0_MODULE); break;
	    	case 1: periph_module_enable(PERIPH_UART1_MODULE); break;
	    	case 2: periph_module_enable(PERIPH_UART2_MODULE); break;
	    }

	    int intr_alloc_flags = 0;

	#if CONFIG_UART_ISR_IN_IRAM
	    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
	#endif

	    ESP_ERROR_CHECK(uart_driver_install(PIXY_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));

	    //ESP_ERROR_CHECK(uart_param_config(PIXY_UART_NUM, &uart_config));
	    ESP_ERROR_CHECK(uart_set_baudrate(PIXY_UART_NUM, 19200));
	    ESP_ERROR_CHECK(uart_set_word_length(PIXY_UART_NUM, UART_DATA_8_BITS));
	    ESP_ERROR_CHECK(uart_set_parity(PIXY_UART_NUM, UART_PARITY_DISABLE));
	    ESP_ERROR_CHECK(uart_set_stop_bits(PIXY_UART_NUM, UART_STOP_BITS_1));
	    ESP_ERROR_CHECK(uart_set_hw_flow_ctrl(PIXY_UART_NUM, UART_HW_FLOWCTRL_DISABLE, 0));
	    ESP_ERROR_CHECK(uart_set_mode(PIXY_UART_NUM, UART_MODE_UART));

	    ESP_ERROR_CHECK(uart_set_pin(PIXY_UART_NUM, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));



		return 0;
	}

	void close() {
		//driver_error_t *error;

	    ESP_ERROR_CHECK(uart_driver_delete(PIXY_UART_NUM));

		//if ((error = spi_unsetup(spi_device))) {
		//	return;
		//}
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL) {
		//driver_error_t *error;

		/*uint8_t i, j;
		char c;
		if (cs)
			*cs = 0;
		for (i = 0; i < len; i++) {
			// wait for byte, timeout after 2ms
			// note for a baudrate of 19.2K, each byte takes about 500us
			for (j = 0; true; j++) {
				if (j == 200)
					return -1;

				//c = Serial1.read();
				uint8_t ret = uart_read(PIXY_UART, &c, 0);
				if (ret == 1)
					break;
				usleep(10);
			}
			buf[i] = c;

			if (cs)
				*cs += buf[i];
		}*/

		int ret = uart_read_bytes(PIXY_UART_NUM, buf, len, 20 / portTICK_RATE_MS);
		if (ret<0) return ret;
		if (cs) {
			*cs = 0;
			uint8_t i;
			for (i = 0; i < ret; i++) {
				*cs += buf[i];
			}
		}

		return ret;
	}

	int16_t send(uint8_t *buf, uint8_t len) {
		//driver_error_t *error;

		//if ((error = spi_bulk_write(spi_device, len, buf))) {
		//	return 0;
		//}

		int ret = uart_write_bytes(PIXY_UART_NUM, (char*)buf, len);

	    return ret;
	}

	void setArg(uint16_t arg) {
	}

private:
};

#endif //_PIXY2UART_H
