#ifndef _MPU6050_
#define _MPU6050_

#include <stdint.h>
#include "esp_err.h"

#define MPU6050_TAG "MPU6050"

#define NACK			0
#define ACK				1

//read/write macro used in i2c read/write function calls
#define WRITE(mpu6050_addr)		((mpu6050_addr << 1) | 0)
#define READ(mpu6050_addr)		((mpu6050_addr << 1) | 1)

typedef enum{
	MPU6050_ADDR0	= 0x68,
	MPU6050_ADDR1
}mpu6050_addr_t;

typedef enum{
	RXA_OFFS_USRH	= 0x06,
	RXA_OFFS_USRL	= 0x07,
	RYA_OFFS_USRH	= 0x08,
	RYA_OFFS_USRL	= 0x09,
	RZA_OFFS_USRH	= 0x0A,
	RZA_OFFS_USRL	= 0x0B,
	RXG_OFFS_USRH	= 0x13,
	RXG_OFFS_USRL	= 0x14,
	RYG_OFFS_USRH	= 0x15,
	RYG_OFFS_USRL	= 0x16,
	RZG_OFFS_USRH	= 0x17,
	RZG_OFFS_USRL	= 0x18,
	RSMPLRT_DIV		= 0x19,
	RCONFIG			= 0x1A,
	RGYRO_CONFIG	= 0x1B,
	RACCEL_CONFIG	= 0x1C,
	RFIFO_EN		= 0x23,
	RINT_PIN_CFG	= 0x37,
	RINT_ENABLE		= 0x38,
	RINT_STATUS		= 0x3A,
	RACCEL_XOUT_H	= 0x3B,
	RACCEL_XOUT_L	= 0x3C,
	RACCEL_YOUT_H	= 0x3D,
	RACCEL_YOUT_L	= 0x3E,
	RACCEL_ZOUT_H	= 0x3F,
	RACCEL_ZOUT_L	= 0x40,
	RTEMP_OUT_H		= 0x41,
	RTEMP_OUT_L		= 0x42,
	RGYRO_XOUT_H	= 0x43,
	RGYRO_XOUT_L	= 0x44,
	RGYRO_YOUT_H	= 0x45,
	RGYRO_YOUT_L	= 0x46,
	RGYRO_ZOUT_H	= 0x47,
	RGYRO_ZOUT_L	= 0x48,
	RUSER_CTRL		= 0x6A,
	RPWR_MGMT_1		= 0x6B,
	RPWR_MGMT_2		= 0x6C,
	RFIFO_COUNTH	= 0x72,
	RFIFO_COUNTL	= 0x73,
	RFIFO_R_W		= 0x74,
	R_WHO_AM_I		= 0x75
}mpu6050_reg_t;

esp_err_t mpu6050_i2c_init(void);
void mpu6050_i2c_deinit(void);
esp_err_t mpu6050_sync_default_init(mpu6050_addr_t addr);

esp_err_t mpu6050_write_byte(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data);
esp_err_t mpu6050_write_burst(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data, uint8_t data_len);
esp_err_t mpu6050_read_byte(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data);
esp_err_t mpu6050_read_burst(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data, uint8_t data_len);

esp_err_t mpu6050_read_sensors(mpu6050_addr_t addr, int16_t * data);
esp_err_t mpu6050_offsets_init(mpu6050_addr_t addr);
esp_err_t mpu6050_show_config(mpu6050_addr_t addr);

#endif  
