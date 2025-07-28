/*
 * ICM_42688_P.h
 *
 *  Created on: Aug 9, 2023
 *      Author: laure
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_





#include "spi.h"
#include <stdbool.h>

#include "enumerations.h"





#define read_imu 0x80
#define write_imu 0x00

#define WHO_AM_I 0x75
#define REG_BANK_SEL 0x76
#define ICM42688_ID 0x47




#define icm42688 (&hspi1)
#define icm42688_port GPIOC
#define icm42688_pin GPIO_PIN_5





typedef struct {

	uint8_t DMA_rxBuf[13];
	uint8_t DMA_txBuf[13];


	bool DMA_DONE;
	int N;



	//uint8_t DMA_tx;

	uint8_t readingDMA;
	bool processedDMA;

	float temperature;
	float acc_g[3];
	float gyr_dps[3];

	float acc_mps2[3];
	float gyr_rad[3];

	float offset_a[3];
	float offset_g[3];

	float gyro_factor;
	float accel_factor;

	bool init_completed;

}ICM42688;


typedef struct {

	float x;
	float y;
	float z;

}offset;




/* IF COMMON ENUM doesnt work, make accel/gyro enums functions(COMMON_LPF)
 *
typedef enum {
	_odr2,
	_400_odr4,
	_400_odr5,
	_400_odr8,
	_400_odr10,
	_400_odr16,
	_400_odr20,
	_400_odr40,
	_400_odr_lowlatency,
	_200_8odr_lowlatency
}accel_LPF; //max(400Hz, ODR)/(odrX)

typedef enum {
	_odr2,
	_400_odr4,//max(400Hz, ODR)/4
	_400_odr5,
	_400_odr8,
	_400_odr10,
	_400_odr16,
	_400_odr20,
	_400_odr40,
	_400_odr_lowlatency,
	_200_8odr_lowlatency
}gyro_LPF; //max(400Hz, ODR)/(odrX)*/

void icm42688_init(ICM42688 *imu);
bool icm42688_identify();
void icm42688_reset();
void icm42688_clock_sel();
void icm42688_boot();
void icm42688_FIFO_CONF();
void icm42688_FIFO_EN();
void Icm42688_INT1_START();
void Icm42688_INT1_STOP();

void ICM42688_WriteCalibration(ICM42688 *imu);

void ICM42688_gyro_config(gyro_scale scale, ODR odr, ICM42688 *imu);
void ICM42688_accel_config(accel_scale scale, ODR odr, ICM42688 *imu);
void accel_gyro_bandwidth(COMMON_LPF ABW, COMMON_LPF GBW);

void FIFO_READ(ICM42688 *imu);

uint8_t ICM42688_ReadDMA(ICM42688 *imu, BLOCKING *state, uint8_t *txBuf, uint8_t *rxBuf);
void ICM42688_ReadDMA_Complete(ICM42688 *imu,uint8_t *rxBuf);
void ICM42688_ReadData(ICM42688 *imu);

void ICM42688_InterruptSetup();

void ICM42688_CollectCalibration(ICM42688 *imu);
void ICM42688_Write_1REG(bank bx, uint8_t addr, uint8_t data);
void LP_LN(ICM42688 *imu);

/*
void ICM42688_Gyro_Read(axis* value);
void ICM42688_Gyro_Read_DPS(axis* value);
void ICM42688_Accel_Read(axis* value);
void ICM42688_Accel_Read_DPS(axis* value);*/

//User bank 0
#define DEVICE_CONFIG 0x11  //0x00
#define DRIVE_CONFIG 0x13 //0x05
#define INT_CONFIG 0x14 //0x00
#define FIFO_CONFIG 0x16 //0x00
#define TEMP_DATA1 0x1D //0x80
#define TEMP_DATA0 0x1E //0x00
#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24
#define GYRO_DATA_X1 0x25
#define GYRO_DATA_X0 0x26
#define GYRO_DATA_Y1 0x27
#define GYRO_DATA_Y0 0x28
#define GYRO_DATA_Z1 0x29
#define GYRO_DATA_Z0 0x2A
#define TMST_FSYNCH 0x2B
#define TMST_FSYNCL 0x2C
#define INT_STATUS 0x2D
#define FIFO_COUNTH 0x2E
#define FIFO_COUNTL 0x2F
#define FIFO_DATA 0x30
#define APEX_DATA0 0x31
#define APEX_DATA1 0x32
#define APEX_DATA2 0x33
#define APEX_DATA3 0x34
#define APEX_DATA4 0x35
#define APEX_DATA5 0x36
#define INT_STATUS2 0x37
#define INT_STATUS3 0x38
#define SIGNAL_PATH_RESET 0x4B
#define INTF_CONFIG0 0x4C
#define INTF_CONFIG1 0x4D
#define PWR_MGMT0 0x4E
#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define GYRO_CONFIG1 0x51
#define GYRO_ACCEL_CONFIG0 0x52
#define ACCEL_CONFIG1 0x53
#define TMST_CONFIG 0x54
#define APEX_CONFIG0 0x56
#define SMD_CONFIG 0x57
#define FIFO_CONFIG1 0x5F
#define FIFO_CONFIG2 0x60
#define FIFO_CONFIG3 0x61
#define FSYNC_CONFIG 0x62
#define INT_CONFIG0 0x63
#define INT_CONFIG1 0x64
#define INT_SOURCE0 0x65
#define INT_SOURCE1 0x66
#define INT_SOURCE3 0x68
#define INT_SOURCE4 0x69
#define FIFO_LOST_PKT0 0x6C
#define FIFO_LOST_PKT1 0x6D
#define SELF_TEST_CONFIG 0x70



//User bank 1

#define SENSOR_CONFIG0 0x03
#define GYRO_CONFIG_STATIC2 0x0B
#define GYRO_CONFIG_STATIC3 0x0C
#define GYRO_CONFIG_STATIC4 0x0D
#define GYRO_CONFIG_STATIC5 0x0E
#define GYRO_CONFIG_STATIC6 0x0F
#define GYRO_CONFIG_STATIC7 0x10
#define GYRO_CONFIG_STATIC8 0x11
#define GYRO_CONFIG_STATIC9 0x12
#define GYRO_CONFIG_STATIC10 0x13
#define XG_ST_DATA 0x5F
#define YG_ST_DATA 0x60
#define ZG_ST_DATA 0x61
#define TMSTVAL0 0x62
#define TMSTVAL1 0x63
#define TMSTVAL2 0x64
#define INTF_CONFIG4 0x7A
#define INTF_CONFIG5 0x7B
#define INTF_CONFIG6 0x7C


//User bank 2

#define ACCEL_CONFIG_STATIC2 0x03
#define ACCEL_CONFIG_STATIC3 0x04
#define ACCEL_CONFIG_STATIC4 0x05
#define XA_ST_DATA 0x3B
#define YA_ST_DATA 0x3C
#define ZA_ST_DATA 0x3D



//User bank 4

#define APEX_CONFIG1 0x40
#define APEX_CONFIG2 0x41
#define APEX_CONFIG3 0x42
#define APEX_CONFIG4 0x43
#define APEX_CONFIG5 0x44
#define APEX_CONFIG6 0x45
#define APEX_CONFIG7 0x46
#define APEX_CONFIG8 0x47
#define APEX_CONFIG9 0x48
#define ACCEL_WOM_X_THR 0x4A
#define ACCEL_WOM_Y_THR 0x4B
#define ACCEL_WOM_Z_THR 0x4C
#define INT_SOURCE6 0x4D
#define INT_SOURCE7 0x4E
#define INT_SOURCE8 0x4F
#define INT_SOURCE9 0x50
#define INT_SOURCE10 0x51
#define OFFSET_USER0 0x77
#define OFFSET_USER1 0x78
#define OFFSET_USER2 0x79
#define OFFSET_USER3 0x7A
#define OFFSET_USER4 0x7B
#define OFFSET_USER5 0x7C
#define OFFSET_USER6 0x7D
#define OFFSET_USER7 0x7E
#define OFFSET_USER8 0x7F


#endif /* INC_ICM42688_H_ */
