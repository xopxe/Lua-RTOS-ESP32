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

#ifndef _PIXY2SPI_SS_H
#define _PIXY2SPI_SS_H

//#include "TPixy2.h"
#include <drivers/gpio.h>
#include <drivers/spi.h>
#include "sdkconfig.h"

#define PIXY_SPI_CLOCKRATE       CONFIG_PIXY2_SPI_CLOCKRATE

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI (23)
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   22
#define SPI_ID CPU_SPI3

typedef struct {
	int spi_device; // SPI device
	uint8_t *buff; // Data buffer
	uint32_t len;   // Data buffer length
} spi_userdata;

class Pixy2Link  {
public:
	int8_t open(uint32_t arg) {
		driver_error_t *error;

		int id = SPI_ID;
		int is_master = 1;
		int cs = PIN_NUM_CS;
		uint32_t clock = PIXY_SPI_CLOCKRATE;
		//int data_bits = 8;
		int spi_mode = 3;
		int flags = SPI_FLAG_WRITE | SPI_FLAG_READ;
		error = spi_setup(id, is_master, cs, spi_mode, clock, flags,
				&spi_device);
		if (error==NULL)
			return 0;
		else
			return -1;
	}

	void close() {
		driver_error_t *error;

		if ((error = spi_unsetup(spi_device))) {
			return;
		}
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL) {
		driver_error_t *error;

	    // Select device
	    if ((error = spi_select(spi_device))) {
	        return 0;
	    }

		if ((error = spi_bulk_read(spi_device, len, buf))) {
			return 0;
		}

		if (cs) {
			*cs = 0;
			uint8_t i;
			for (i = 0; i < len; i++) {
				*cs += buf[i];
			}
		}

	    // Deselect device
	    if ((error = spi_deselect(spi_device))) {
			return 0;
	    }

		return len;
	}

	int16_t send(uint8_t *buf, uint8_t len) {
		driver_error_t *error;

	    // Select device
	    if ((error = spi_select(spi_device))) {
	        return 0;
	    }

		if ((error = spi_bulk_write(spi_device, len, buf))) {
			return 0;
		}

	    // Deselect device
	    if ((error = spi_deselect(spi_device))) {
			return 0;
	    }
	    return len;
	}

	void setArg(uint16_t arg) {
	}

private:
	int  spi_device;
};

#endif
