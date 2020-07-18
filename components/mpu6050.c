#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include "driver/i2c.h"
//#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

static i2c_ready_t i2c_ready = NOT_READY;

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
	
	i2c_ready = READY;
	return err;
}

void mpu6050_i2c_deinit(){
	i2c_driver_delete(I2C_NUM_0);
	i2c_ready = NOT_READY;
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
	vTaskDelay(20 / portTICK_RATE_MS);
	
	reg_value = 0x00; //set sample rate divider
	mpu6050_write_byte(addr, RSMPLRT_DIV, &reg_value);
	vTaskDelay(20 / portTICK_RATE_MS);
	
	reg_value = 0x01; //set Digital low Pass Filter
	mpu6050_write_byte(addr, RCONFIG, &reg_value);
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
esp_err_t mpu6050_read(mpu6050_addr_t addr, mpu6050_reg_t reg, uint8_t * data, uint8_t data_len){
	
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
