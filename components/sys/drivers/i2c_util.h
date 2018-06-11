#ifndef I2C_UTIL_H
#define I2C_UTIL_H

//#include <cstdint>
//#include <sys/driver.h>
#include <drivers/i2c.h>


static void i2c_util_print_driver_error(driver_error_t *error, int err) {
  if (error->msg) {
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:%s\r\n", err, error->type, error->unit, error->exception, error->msg);
  } else{
  	printf(" DRIVER ERROR [%d]: type: %d, unit: %d, exc: %d msg:NULL\r\n", err, error->type, error->unit, error->exception);
  }
  free(error);
}


// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
static int i2c_util_writeMulti(int i2cdevice, uint8_t address, uint8_t reg, uint8_t const * src, uint8_t count)
{
  driver_error_t *error;
    int transaction = I2C_TRANSACTION_INITIALIZER;
    if ((error = i2c_start(i2cdevice, &transaction))) {
    	i2c_util_print_driver_error(error, -1);
    	return -1;
  }

    if ((error = i2c_write_address(i2cdevice, &transaction, address, 0))) {
    	i2c_util_print_driver_error(error, -2);
    	return -2;
  }

    if ((error = i2c_write(i2cdevice, &transaction, (char*)&reg, 1))) {
    	i2c_util_print_driver_error(error, -3);
    	return -3;
  }

    if ((error = i2c_write(i2cdevice, &transaction, (char*)src, count))) {
    	i2c_util_print_driver_error(error, -4);
    	return -4;
  }

    if ((error = i2c_stop(i2cdevice, &transaction))) {
    	i2c_util_print_driver_error(error, -5);
    	return -5;
  }
  return 0;
}

// Write an 8-bit register
static void i2c_util_writeReg(int i2cdevice, uint8_t address, uint8_t reg, char value)
{
  i2c_util_writeMulti(i2cdevice, address, reg, (uint8_t*)&value, 1);
}

// Write a 16-bit register
static void i2c_util_writeReg16Bit(int i2cdevice, uint8_t address, uint8_t reg, uint16_t value)
{
  i2c_util_writeMulti(i2cdevice, address, reg, (uint8_t*)&value, 2);
}

// Write a 32-bit register
static void i2c_util_writeReg32Bit(int i2cdevice, uint8_t address, uint8_t reg, uint32_t value)
{
  i2c_util_writeMulti(i2cdevice, address, reg, (uint8_t*)&value, 4);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
static int i2c_util_readMulti(int i2cdevice, uint8_t address, uint8_t reg, uint8_t * dst, uint8_t count)
{
  driver_error_t *error;
    int transaction = I2C_TRANSACTION_INITIALIZER;

    if ((error=i2c_start(i2cdevice, &transaction))) {
    	i2c_util_print_driver_error(error, -1);
    	return -1;
  }
    if ((error = i2c_write_address(i2cdevice, &transaction, address, 0))) {
    	i2c_util_print_driver_error(error, -2);
    	return -2;
  }	
  if ((error = i2c_write(i2cdevice, &transaction, (char *)&reg, 1))) {
    	i2c_util_print_driver_error(error, -3);
    	return -3;
  }	
/*  if ((error = i2c_stop(i2cdevice, &transaction))) {
    	print_driver_error(error, -4);
    	return -4;
  }*/

    if ((error = i2c_start(i2cdevice, &transaction))) {
    	i2c_util_print_driver_error(error, -5);
    	return -5;
  }
    if ((error = i2c_write_address(i2cdevice, &transaction, address, 1))) {
    	i2c_util_print_driver_error(error, -6);
    	return -6;
  }	if ((error = i2c_read(i2cdevice, &transaction, (char *)dst, count))) {
    	i2c_util_print_driver_error(error, -7);
    	return -7;
  }	if ((error = i2c_stop(i2cdevice, &transaction))) {
    	i2c_util_print_driver_error(error, -8);
    	return -8;
  }

  return 0;
}

// Read an 8-bit register
static uint8_t i2c_util_readReg(int i2cdevice, uint8_t address, uint8_t reg)
{
  uint8_t value[1];
  i2c_util_readMulti(i2cdevice, address, reg, value, 1);
  return value[0];
}

// Read a 16-bit register
static uint16_t i2c_util_readReg16Bit(int i2cdevice, uint8_t address, uint8_t reg)
{
  uint16_t value;
  i2c_util_readMulti(i2cdevice, address, reg, (uint8_t *)&value, 2);
  return value;
}

// Read a 32-bit register
static uint32_t i2c_util_readReg32Bit(int i2cdevice, uint8_t address, uint8_t reg)
{
  uint32_t value;
  i2c_util_readMulti(i2cdevice, address, reg, (uint8_t *)&value, 4);
  return value;
}


#endif
