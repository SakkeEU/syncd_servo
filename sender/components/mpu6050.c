#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "driver/i2c.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

//TODO: add parameters for customization
esp_err_t mpu6050_i2c_init(void){
	
	esp_err_t err;
	const i2c_config_t cfg = {
		.mode 				= I2C_MODE_MASTER,
		.sda_io_num 		= GPIO_NUM_0,
		.sda_pullup_en		= GPIO_PULLUP_DISABLE,	//external pullup resistors needed
		.scl_io_num			= GPIO_NUM_2,
		.scl_pullup_en		= GPIO_PULLUP_DISABLE,	//external pullup resistors needed
		.clk_stretch_tick = 200
	};
	
	err = i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER);
    ESP_LOGD(MPU6050_TAG, "install %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
    if(err) 
		return err;
    err = i2c_param_config(I2C_NUM_0, &cfg);
	ESP_LOGD(MPU6050_TAG, "config %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	
	return err;
}

void mpu6050_i2c_deinit(void){
	i2c_driver_delete(I2C_NUM_0);
}

//set some register value for the mpu6050
esp_err_t mpu6050_sync_default_init(mpu6050_addr_t addr){
	
	esp_err_t err;
	 uint8_t reg_value = 0;
		
	//it needs the delays to work
	reg_value = 0x80; //reset the mpu, it requires 100 ms
	err = mpu6050_write_byte(addr, RPWR_MGMT_1, &reg_value);
	if(err)
		return err;
	vTaskDelay(100 / portTICK_RATE_MS);
	
	reg_value = 0x01; //wake up the mpu and set clk_sel
	err = mpu6050_write_byte(addr, RPWR_MGMT_1, &reg_value);
	if(err)
		return err;
	vTaskDelay(100 / portTICK_RATE_MS);
		
	reg_value = 0x01; //set sample rate divider
	err = mpu6050_write_byte(addr, RSMPLRT_DIV, &reg_value);
	if(err)
		return err;
	vTaskDelay(20 / portTICK_RATE_MS);
	
	reg_value = 0x01; //set Digital low Pass Filter
	err = mpu6050_write_byte(addr, RCONFIG, &reg_value);
	vTaskDelay(200 / portTICK_RATE_MS);
	
	return err;
}

esp_err_t mpu6050_write_byte(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data){
	
	esp_err_t err;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	//tell the mpu which reg is the target then write
	i2c_master_start(handle);
	i2c_master_write_byte(handle, WRITE(addr), ACK);
	i2c_master_write_byte(handle, reg, ACK);
	i2c_master_write_byte(handle, * data, ACK);
	i2c_master_stop(handle);
	err = i2c_master_cmd_begin(I2C_NUM_0, handle, 100 / portTICK_RATE_MS);
	ESP_LOGD(MPU6050_TAG, "write %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	i2c_cmd_link_delete(handle);
	
	return err;
}
//write burst mode
esp_err_t mpu6050_write_burst(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data, uint8_t data_len){
	
	esp_err_t err;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	//tell the mpu which reg is the target then write
	i2c_master_start(handle);
	i2c_master_write_byte(handle, WRITE(addr), ACK);
	i2c_master_write_byte(handle, reg, ACK);
	i2c_master_write(handle, data, data_len, ACK);
	i2c_master_stop(handle);
	err = i2c_master_cmd_begin(I2C_NUM_0, handle, 100 / portTICK_RATE_MS);
	ESP_LOGD(MPU6050_TAG, "write_burst %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	i2c_cmd_link_delete(handle);
	
	return err;
}

static inline esp_err_t mpu6050_write_static(mpu6050_addr_t addr, mpu6050_reg_t reg){
	
	esp_err_t err;
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	//tell the mpu which reg is the target
	i2c_master_start(handle);
	i2c_master_write_byte(handle, WRITE(addr), ACK);
	i2c_master_write_byte(handle, reg, ACK);
	i2c_master_stop(handle);
	err = i2c_master_cmd_begin(I2C_NUM_0, handle, 100 / portTICK_RATE_MS);
	ESP_LOGD(MPU6050_TAG, "write %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	i2c_cmd_link_delete(handle);
	
	return err;
}

esp_err_t mpu6050_read_byte(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data){
	
	esp_err_t err;
	err = mpu6050_write_static(addr, reg);
	if(err)
		return err;
	
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	//read the target reg
	i2c_master_start(handle);
	i2c_master_write_byte(handle, READ(addr), ACK);
	i2c_master_read_byte(handle, data, I2C_MASTER_NACK);
	i2c_master_stop(handle);
	err = i2c_master_cmd_begin(I2C_NUM_0, handle, 100 / portTICK_RATE_MS);
	ESP_LOGD(MPU6050_TAG, "read %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	i2c_cmd_link_delete(handle);
	
	return err;
}

//burst read mode
esp_err_t mpu6050_read_burst(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data, uint8_t data_len){
	
	esp_err_t err;
	err = mpu6050_write_static(addr, reg);
	if(err)
		return err;
		
	i2c_cmd_handle_t handle = i2c_cmd_link_create();
	
	//read starting from the target reg
	//reg value is automatically incremented after each read
	i2c_master_start(handle);
	i2c_master_write_byte(handle, READ(addr), ACK);
	
	i2c_master_read(handle, data, data_len, I2C_MASTER_LAST_NACK);
	i2c_master_stop(handle);
	err = i2c_master_cmd_begin(I2C_NUM_0, handle, 100 / portTICK_RATE_MS);
	ESP_LOGD(MPU6050_TAG, "BURST read %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	i2c_cmd_link_delete(handle);
	
	return err;
}

//calculate the bitwise OR of the 2 registers dedicated to each measurement
//to get the raw complete measure. data length > 7 needed.
esp_err_t mpu6050_read_sensors(mpu6050_addr_t addr, int16_t * data){
	
	esp_err_t err;
	uint8_t i;
	uint8_t len = 14; //number of internal sensors registers
	uint8_t data_temp[len];
	for(i = 0; i <= 14; i++)
		data_temp[i] = 0;
	err = mpu6050_read_burst(addr, RACCEL_XOUT_H, data_temp, len);
	if(err)
		return err;
	
	for(i = 0; i < len - 1; i += 2)
			data[i/2] = (data_temp[i] << 8) | data_temp[i + 1];	
	return err;
}

esp_err_t mpu6050_offsets_init(mpu6050_addr_t addr){
	
	esp_err_t err;
	//measurement max value: 32768
	printf("\nOffset registers initialization... \n");
	
	//save the config already loaded into the registers
	uint8_t accel_config = 0;
	uint8_t gyro_config = 0;
	err = mpu6050_read_byte(addr, RACCEL_CONFIG, &accel_config);
	if(err)
		return err;
	err = mpu6050_read_byte(addr, RGYRO_CONFIG, &gyro_config);
	if(err)
		return err;
	
	//set the configs to +-16g for the accel
	uint8_t temp_config = 0x18;
	err = mpu6050_write_byte(addr, RACCEL_CONFIG, &temp_config);
	if(err)
		return err;
	vTaskDelay(20 / portTICK_RATE_MS);
	//set the configs to +-1000deg/sec for the gyro
	temp_config = 0x10;
	err = mpu6050_write_byte(addr, RGYRO_CONFIG, &temp_config);
	if(err)
		return err;
	vTaskDelay(20 / portTICK_RATE_MS);
	
	//read the factory trim inside the accel offset registers
	uint8_t bias_8b[6] = {0};
	err = mpu6050_read_burst(addr, RXA_OFFS_USRH, bias_8b, 6);
	if(err)
		return err;
	
	int16_t accel_bias[3] = {0};
	accel_bias[0] = (bias_8b[0] << 8) | bias_8b[1];
	accel_bias[1] = (bias_8b[2] << 8) | bias_8b[3];
	accel_bias[2] = (bias_8b[4] << 8) | bias_8b[5];
	
	int32_t loops = 1600;
	int32_t accel_sens = 2048;
	int16_t readings[7] = {0};
	int32_t sums[7] = {0};
	
	//TODO: check for failed readings and add failed readings counter
	vTaskDelay(200 / portTICK_RATE_MS);
	for(uint16_t i = 0; i < loops; i++){
		mpu6050_read_sensors(addr, readings);

		sums[0] += (int32_t)readings[0];
		sums[1] += (int32_t)readings[1];
		sums[2] += ((int32_t)readings[2]) - accel_sens; //gravity compensation
		sums[4] += (int32_t)readings[4];
		sums[5] += (int32_t)readings[5];
		sums[6] += (int32_t)readings[6];
	}
	int16_t bias_16b[6] = {0};
	bias_16b[0] = (int16_t)((sums[0]/loops));
	bias_16b[1] = (int16_t)((sums[1]/loops));
	bias_16b[2] = (int16_t)((sums[2]/loops));
	bias_16b[4] = (int16_t)((sums[4]/loops));
	bias_16b[5] = (int16_t)((sums[5]/loops));
	bias_16b[6] = (int16_t)((sums[6]/loops));
	
	//sum sums bias to factory trim, first bit is reserved
	accel_bias[0] -= (bias_16b[0] & ~1);
	accel_bias[1] -= (bias_16b[1] & ~1);
	accel_bias[2] -= (bias_16b[2] & ~1);
	
	//load new accels biases in the accel bias registers
	bias_8b[0] = (accel_bias[0] >> 8) & 0xFF;
	bias_8b[1] = accel_bias[0] & 0xFF;
	bias_8b[2] = (accel_bias[1] >> 8) & 0xFF;
	bias_8b[3] = accel_bias[1] & 0xFF;
	bias_8b[4] = (accel_bias[2] >> 8) & 0xFF;
	bias_8b[5] = accel_bias[2] & 0xFF;
	err = mpu6050_write_burst(addr, RXA_OFFS_USRH, bias_8b, 6);
	if(err)
		return err;
	
	//load new gyro biases in the gyro bias registers
	bias_16b[4] = -bias_16b[4];
	bias_16b[5] = -bias_16b[5];
	bias_16b[6] = -bias_16b[6];
	bias_8b[0] = ((bias_16b[4] >> 8) & 0xFF);
	bias_8b[1] = bias_16b[4] & 0xFF;
	bias_8b[2] = ((bias_16b[5] >> 8) & 0xFF);
	bias_8b[3] = bias_16b[5] & 0xFF;
	bias_8b[4] = ((bias_16b[6] >> 8) & 0xFF);
	bias_8b[5] = bias_16b[6] & 0xFF;
	err = mpu6050_write_burst(addr, RXG_OFFS_USRH, bias_8b, 6);
	if(err)
		return err;
	
	//change back the settings
	err = mpu6050_write_byte(addr, RACCEL_CONFIG, &accel_config);
	if(err)
		return err;
	vTaskDelay(20 / portTICK_RATE_MS);
	err = mpu6050_write_byte(addr, RGYRO_CONFIG, &gyro_config);
	if(err)
		return err;
	vTaskDelay(200 / portTICK_RATE_MS);
	printf("\nDone.\n");
	
	return err;
}

//print the value of some register
esp_err_t mpu6050_show_config(mpu6050_addr_t addr){
	
	esp_err_t err;
	uint8_t reg_value = 0;
	
	printf("*********************\n          ::CONFIG VALUES::\n");
	err = mpu6050_read_byte(addr, RSMPLRT_DIV, &reg_value);
	if(err)
		return err;
	printf("SmplRD: %u\n", reg_value);
	err = mpu6050_read_byte(addr, RCONFIG, &reg_value);
	if(err)
		return err;
	printf("DLPF: %u\nFSYNC: %u\n", reg_value & 0x03, (reg_value >> 3) & 0x07);
	err = mpu6050_read_byte(addr, RGYRO_CONFIG, &reg_value);
	if(err)
		return err;
	printf("Gyro FS_SEL: %u\n", (reg_value >> 3) & 0x03);
	err = mpu6050_read_byte(addr, RACCEL_CONFIG, &reg_value);
	if(err)
		return err;
	printf("Accel FS_SEL: %u\n", (reg_value >> 3) & 0x03);
	err = mpu6050_read_byte(addr, RPWR_MGMT_1, &reg_value);
	if(err)
		return err;
	printf("CLK_SEL: %u\nTEMP_DIS: %u\n", reg_value & 0x07, (reg_value >> 3) & 0x01);
	printf("CYCLE: %u\nSLEEP: %u\n", (reg_value >> 5) & 0x01, (reg_value >> 6) & 0x01);
	err = mpu6050_read_byte(addr, RPWR_MGMT_2, &reg_value);
	if(err)
		return err;
	printf("STDBY_ZG: %u\nSTDBY_YG: %u\n", reg_value & 0x01, (reg_value >> 1) & 0x01);
	printf("STDBY_XG: %u\nSTDBY_ZA: %u\n", (reg_value >> 2) & 0x01, (reg_value >> 3) & 0x01);
	printf("STDBY_YA: %u\nSTDBY_XA: %u\n", (reg_value >> 4) & 0x01,(reg_value >> 5) & 0x01);
	printf("LP_WAKE_CTRL: %u\n", (reg_value >> 6) & 0x03);
	printf("*********************\n");
	return err;
}
