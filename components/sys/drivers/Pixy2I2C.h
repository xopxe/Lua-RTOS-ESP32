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

#ifndef _PIXY2I2C_H
#define _PIXY2I2C_H

//#include "TPixy2.h"
#include <drivers/i2c.h>
#include "sdkconfig.h"

#define PIXY_I2C_DEFAULT_ADDR           0x54
#define PIXY_I2C_MAX_SEND               16 // don't send any more than 16 bytes at a time

#define PIXY_I2C_SPEED 40000
#define I2C_CHANNEL 0

void i2c_print_driver_error(driver_error_t *error, int err) {
  if (error->msg) {
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:%s\r\n", err, error->type, error->unit, error->exception, error->msg);
  } else{
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:NULL\r\n", err, error->type, error->unit, error->exception);
  }
  free(error);
}

int i2c_readMulti(int i2cdevice, uint8_t address, uint8_t * dst, uint8_t count)
{
	//printf("+readmulti %d\n", count);

	driver_error_t *error;
    int transaction = I2C_TRANSACTION_INITIALIZER;

/*    if ((error=i2c_start(i2cdevice, &transaction))) {
    	i2c_print_driver_error(error, -1);
    	return -1;
  }
    if ((error = i2c_write_address(i2cdevice, &transaction, address, 0))) {
    	i2c_print_driver_error(error, -2);
    	return -2;
  }
  */
//  if ((error = i2c_write(i2cdevice, &transaction, (char *)&reg, 1))) {
//    	i2c_print_driver_error(error, -3);
//    	return -3;
// }

    if ((error = i2c_start(i2cdevice, &transaction))) {
		i2c_print_driver_error(error, -5);
		return -5;
	}
	if ((error = i2c_write_address(i2cdevice, &transaction, address, 1))) {
		i2c_print_driver_error(error, -6);
		return -6;
	}
	if ((error = i2c_read(i2cdevice, &transaction, (char*) dst, count))) {
		i2c_print_driver_error(error, -7);
		return -7;
	}
	if ((error = i2c_stop(i2cdevice, &transaction))) {
		i2c_print_driver_error(error, -8);
		return -8;
	}

	//printf("-readmulti %d %x\n", count, dst[0]);
	return 0;
}

int i2c_writeMulti(int i2cdevice, uint8_t address, uint8_t const * src, uint8_t count)
{
	//printf("+writemulti %d\n", count);

	driver_error_t *error;
    int transaction = I2C_TRANSACTION_INITIALIZER;
    if ((error = i2c_start(i2cdevice, &transaction))) {
    	i2c_print_driver_error(error, -1);
    	return -1;
  }

    if ((error = i2c_write_address(i2cdevice, &transaction, address, 0))) {
    	i2c_print_driver_error(error, -2);
    	return -2;
  }

//    if ((error = i2c_write(i2cdevice, &transaction, (char*)&reg, 1))) {
//    	i2c_util_print_driver_error(error, -3);
//    	return -3;
//  }

    if ((error = i2c_write(i2cdevice, &transaction, (char*)src, count))) {
    	i2c_print_driver_error(error, -4);
    	return -4;
  }

    if ((error = i2c_stop(i2cdevice, &transaction))) {
    	i2c_print_driver_error(error, -5);
    	return -5;
  }

    //printf("-writemulti %d\n", count);
    return 0;
}

class Pixy2Link  {
public:
	int8_t open(uint32_t arg) {
		driver_error_t *error;

		uint8_t i2c = I2C_CHANNEL;
		if ((error=i2c_attach(i2c, I2C_MASTER, PIXY_I2C_SPEED, 0, 0, &i2cdevice))) {
			i2c_print_driver_error(error, -10);
			return -1;
		}
		m_addr = PIXY_I2C_DEFAULT_ADDR;
		return 0;
	}

	void close() {
		driver_error_t *error;

		if ((error = i2c_detach(i2cdevice))) {
			return;
		}
	}

	int16_t recv(uint8_t *buf, uint8_t len, uint16_t *cs = NULL) {
		driver_error_t *error;

	    int ret = i2c_readMulti(i2cdevice, m_addr, buf, len);
		if (ret<0) {
			return -1;
		}

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
		driver_error_t *error;

		//int ret = i2c_writeMulti(i2cdevice, m_addr, buf, len);

		int8_t i, packet;
		for (i = 0; i < len; i += PIXY_I2C_MAX_SEND)
		{
			if (len - i < PIXY_I2C_MAX_SEND)
				packet = len - i;
			else
				packet = PIXY_I2C_MAX_SEND;
			int ret = i2c_writeMulti(i2cdevice, m_addr, buf+i, packet);

			if (ret < 0) {
				return -1;
			}
		}
		return len;

		return len;
	}

	void setArg(uint16_t arg) {
	}

private:
	int  i2cdevice;
	uint8_t m_addr;
};

#endif //_PIXY2I2C_H
