#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "driver/i2c.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

//TODO: find a better solution
static int16_t addr0_offset[7] = {0};
static int16_t addr1_offset[7] = {0};

//TODO: add parameters for customization
esp_err_t mpu6050_i2c_init(){
	
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
    if(err != 0) 
		return err;
    err = i2c_param_config(I2C_NUM_0, &cfg);
	ESP_LOGD(MPU6050_TAG, "config %s:%d", ((err == 0) ? "succeeded" : "failed"), err);
	
	return err;
}

void mpu6050_i2c_deinit(){
	i2c_driver_delete(I2C_NUM_0);
}

//set some register value for the mpu6050
void mpu6050_sync_default_init(mpu6050_addr_t addr){
	
	 uint8_t reg_value = 0;
		
	//it needs the delays to work
	reg_value = 0x80; //reset the mpu, it requires 100 ms
	mpu6050_write_byte(addr, RPWR_MGMT_1, &reg_value);
	vTaskDelay(100 / portTICK_RATE_MS);
	
	reg_value = 0x01; //wake up the mpu and set clk_sel
	mpu6050_write_byte(addr, RPWR_MGMT_1, &reg_value);
	vTaskDelay(100 / portTICK_RATE_MS);
		
	reg_value = 0x01; //set sample rate divider
	mpu6050_write_byte(addr, RSMPLRT_DIV, &reg_value);
	vTaskDelay(20 / portTICK_RATE_MS);
	
	reg_value = 0x01; //set Digital low Pass Filter
	mpu6050_write_byte(addr, RCONFIG, &reg_value);
	vTaskDelay(20 / portTICK_RATE_MS);
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
	if(err != 0)
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
	if(err != 0)
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
esp_err_t mpu6050_read_sensors(mpu6050_addr_t addr, int16_t * data, uint8_t with_offset){
	
	esp_err_t err;
	uint8_t i;
	uint8_t len = 14; //number of internal sensors registers
	uint8_t data_temp[len];
	for(i = 0; i <= 14; i++)
		data_temp[i] = 0;
	err = mpu6050_read_burst(addr, RACCEL_XOUT_H, data_temp, len);
	if(err != 0)
		return err;
	
	for(i = 0; i < len - 1; i += 2)
			data[i/2] = (data_temp[i] << 8) | data_temp[i + 1];	
	if(with_offset){
		if(addr == MPU6050_ADDR0){
			for(i = 0; i < len/2; i++)
				data[i] -= addr0_offset[i];
		}else{
			for(i = 0; i < len/2; i++)
				data[i] -= addr0_offset[i];
		}
	}	
	return err;
}

void mpu6050_offsets_init(mpu6050_addr_t addr){
	
	int32_t loops = 1600;
	int32_t wasted_loops = 100;
	int32_t accel_sens = 16384;
	int16_t readings[7] = {0};
	int32_t sums[7] = {0};
	
	for(uint16_t i = 0; i < loops; i++){
		
		mpu6050_read_sensors(addr, readings, 0);
		if(i < wasted_loops){
			continue;
		}else{
			sums[0] += (int32_t)readings[0];
			sums[1] += (int32_t)readings[1];
			sums[2] += ((int32_t)readings[2]) - accel_sens;
			sums[4] += (int32_t)readings[4];
			sums[5] += (int32_t)readings[5];
			sums[6] += (int32_t)readings[6];
		}
	}
	if(addr == MPU6050_ADDR0){
		for(uint8_t i = 0; i < 7; i++){
			if(i == 3) continue;
			addr0_offset[i] = (int16_t)(sums[i]/(loops - wasted_loops));
		}
	}else{
		for(uint8_t i = 0; i < 7; i++){
			if(i == 3) continue;
			addr1_offset[i] = (int16_t)(sums[i]/(loops - wasted_loops));
		}
	}
}

void mpu6050_get_offsets(mpu6050_addr_t addr, int16_t * offset){
	
	if(addr == MPU6050_ADDR0){
		for(uint8_t i = 0; i < 7; i++)
			offset[i] = addr0_offset[i];
	}else{
		for(uint8_t i = 0; i < 7; i++)
			offset[i] = addr0_offset[i];
	}
}

//offsets are loaded into an array as high_byte[i] low_byte[i + 1] etc.
void mpu6050_get_offsets_8bit(mpu6050_addr_t addr, uint8_t * offset){
	
	if(addr == MPU6050_ADDR0){
		for(uint8_t i = 0; i < 13; i++){
			offset[i] = (uint8_t)(addr0_offset[i/2] >> 8);
			offset[i + 1] = (uint8_t)(addr0_offset[i/2] & 0xFF);
		}
	}else{
		for(uint8_t i = 0; i < 13; i++){
			offset[i] = addr1_offset[i/2] >> 8;
			offset[i + 1] = addr1_offset[i/2] & 0xFF;
		}
	}
}

//print the value of some register
void mpu6050_show_config(mpu6050_addr_t addr){
	
	uint8_t reg_value;
	
	printf("*********************\n          ::CONFIG VALUES::\n");
	mpu6050_read_byte(addr, RSMPLRT_DIV, &reg_value);
	printf("SmplRD: %u\n", reg_value);
	mpu6050_read_byte(addr, RCONFIG, &reg_value);
	printf("DLPF: %u\nFSYNC: %u\n", reg_value & 0x03, (reg_value >> 3) & 0x07);
	mpu6050_read_byte(addr, RGYRO_CONFIG, &reg_value);
	printf("Gyro FS_SEL: %u\n", (reg_value >> 3) & 0x03);
	mpu6050_read_byte(addr, RACCEL_CONFIG, &reg_value);
	printf("Accel FS_SEL: %u\n", (reg_value >> 3) & 0x03);
	mpu6050_read_byte(addr, RPWR_MGMT_1, &reg_value);
	printf("CLK_SEL: %u\nTEMP_DIS: %u\n", reg_value & 0x07, (reg_value >> 3) & 0x01);
	printf("CYCLE: %u\nSLEEP: %u\n", (reg_value >> 5) & 0x01, (reg_value >> 6) & 0x01);
	mpu6050_read_byte(addr, RPWR_MGMT_2, &reg_value);
	printf("STDBY_ZG: %u\nSTDBY_YG: %u\n", reg_value & 0x01, (reg_value >> 1) & 0x01);
	printf("STDBY_XG: %u\nSTDBY_ZA: %u\n", (reg_value >> 2) & 0x01, (reg_value >> 3) & 0x01);
	printf("STDBY_YA: %u\nSTDBY_XA: %u\n", (reg_value >> 4) & 0x01,(reg_value >> 5) & 0x01);
	printf("LP_WAKE_CTRL: %u\n", (reg_value >> 6) & 0x03);
	printf("*********************\n");
}
