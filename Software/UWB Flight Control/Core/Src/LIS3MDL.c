/**
 * @file LIS3MDL.c
 * @author Talha Sarı (talha.sari@outlook.com.tr)
 * @brief LIS3MDL 3-axis magnetometer I2C library for STM32 environment
 * @version v1.1
 * @date 2021-06-07
 * 
 * @copyright Copyright (c) 2021 - GNU General Public License v3
 * 
 */

#include "LIS3MDL_Registers.h"
#include "LIS3MDL.h"

#include "include_all.h"
extern struct MagSensor thisMag;					// this magnetometer

int16_t HI_mag[3];

/* I2C R/W Function Prototypes */
static HAL_StatusTypeDef writeByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
		uint8_t register_addr, uint8_t data);
static HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
		uint8_t register_addr, uint8_t *data);
static HAL_StatusTypeDef readMultiBytes(I2C_HandleTypeDef *hi2c,
		uint8_t device_addr, uint8_t register_addr, uint8_t *data,
		uint16_t count);

/* Sensor Functions */
LIS3MDL_Result_t LIS3MDL_Init(LIS3MDL_t *hsensor, I2C_HandleTypeDef *hi2c,
		LIS3MDL_Device_t dev, LIS3MDL_Scale_t scale,
		LIS3MDL_OperationMode_t mode, LIS3MDL_ODR_t odr) {
	uint8_t data;
	hsensor->addr = (uint8_t) (dev << 1);
	hsensor->scale = (LIS3MDL_Scale_t) scale;

	if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t) hsensor->addr, 2, 5) != HAL_OK)
		return LIS3MDL_ERROR;

	readByte(hi2c, hsensor->addr, WHO_AM_I, &data);
	if (data != 0x3D)
		return LIS3MDL_ERROR;
	data = 0x00;

	readByte(hi2c, hsensor->addr, CTRL_REG1, &data);
	data &= ~0xFE;
	data |= 0x80 | ((uint8_t) mode << 5) | (uint8_t) odr;
	writeByte(hi2c, hsensor->addr, CTRL_REG1, data);

	readByte(hi2c, hsensor->addr, CTRL_REG2, &data);
	data &= ~0x60;
	data |= (uint8_t) scale;
	writeByte(hi2c, hsensor->addr, CTRL_REG2, data);

	readByte(hi2c, hsensor->addr, CTRL_REG3, &data);
	data &= ~0x03;
	data |= 0x00;
	writeByte(hi2c, hsensor->addr, CTRL_REG3, data);

	readByte(hi2c, hsensor->addr, CTRL_REG4, &data);
	data &= ~0x0C;
	data |= (uint8_t) (mode << 2);
	writeByte(hi2c, hsensor->addr, CTRL_REG4, data);

	readByte(hi2c, hsensor->addr, CTRL_REG5, &data);
	data &= ~0xC0;
	data |= 0x00;
	writeByte(hi2c, hsensor->addr, CTRL_REG5, data);

	readByte(hi2c, hsensor->addr, INT_CFG, &data);
	data &= ~0xFF;
	data |= 0x00;
	writeByte(hi2c, hsensor->addr, INT_CFG, data);

	switch (hsensor->scale) {
	case LIS3MDL_Scale_4G:
		hsensor->uTscale = 6842.0f;
		break;
	case LIS3MDL_Scale_8G:
		hsensor->uTscale = 3421.0f;
		break;
	case LIS3MDL_Scale_12G:
		hsensor->uTscale = 2281.0f;
		break;
	case LIS3MDL_Scale_16G:
		hsensor->uTscale = 1711.0f; //   LSB/gauss
		break;
	}



//    float hardiron[3] = { 5.8398 , -30.424 , -1.6664};
//
//
//
//    for (int i = 0; i < 3; i++){
//    	HI_mag[i] = (int16_t)(hardiron[i]*hsensor->uTscale);
//    }

//
//    float softiron[3][3] = {{    1.000864 ,  0.00646 ,  0.009619} ,
//    		{   0.00646  ,  1.0027 ,  0.04013},
//    		{    0.009619  , 0.04013 ,   0.9742}};
//
//    for (int i = 0; i < 3; i++){
//    	thisMagCal.fV[i] = hardiron[i];
//
//    for (int j = 0; j < 3; j++) {
//    	thisMagCal.finvW[i][j] = softiron[i][j];
//
//    	}
//    }

	return LIS3MDL_OK;
}

LIS3MDL_Result_t LIS3MDL_ReadMag(LIS3MDL_t *hsensor, I2C_HandleTypeDef *hi2c) {
	uint8_t data[6];

	readMultiBytes(hi2c, hsensor->addr, OUT_X_L, data, 6);

	//Mapping

//    hsensor->mag_raw[0] = ((int16_t)data[1] << 8) | data[0];
//    hsensor->mag_raw[1] = ((int16_t)data[3] << 8) | data[2];
//    hsensor->mag_raw[2] = ((int16_t)data[5] << 8) | data[4];

	/*
	 *
	 * Axis into filter = Axis on magnetometer
	 *
	 * Mx = -My,   My = -Mx,   Mz = Mz
	 *
	 */
	thisMag.iBpFast[1] = -((((int16_t) data[1] << 8) | data[0]) - HI_mag[0]); //My = -Mx
	thisMag.iBpFast[0] = -((((int16_t) data[3] << 8) | data[2]) - HI_mag[1]); //Mx = -My
	thisMag.iBpFast[2] = ((((int16_t) data[5] << 8) | data[4]) - HI_mag[2]); //Mz = Mz

	thisMag.iSumBpFast[0] += thisMag.iBpFast[0];
	thisMag.iSumBpFast[1] += thisMag.iBpFast[1];
	thisMag.iSumBpFast[2] += thisMag.iBpFast[2];

	hsensor->mag_raw[0] = thisMag.iBpFast[0];
	hsensor->mag_raw[1] = thisMag.iBpFast[1];
	hsensor->mag_raw[2] = thisMag.iBpFast[2];

	float magtemp[3];

	switch (hsensor->scale) {
	case LIS3MDL_Scale_4G:
		hsensor->uTscale = 6842.0f;
		magtemp[0] = (float) (hsensor->mag_raw[0] / 6842.0f);
		magtemp[1] = (float) (hsensor->mag_raw[1] / 6842.0f);
		magtemp[2] = (float) (hsensor->mag_raw[2] / 6842.0f);
		break;
	case LIS3MDL_Scale_8G:
		hsensor->uTscale = 3421.0f;
		magtemp[0] = (float) (hsensor->mag_raw[0] / 3421.0f);
		magtemp[1] = (float) (hsensor->mag_raw[1] / 3421.0f);
		magtemp[2] = (float) (hsensor->mag_raw[2] / 3421.0f);
		break;
	case LIS3MDL_Scale_12G:
		hsensor->uTscale = 2281.0f;
		magtemp[0] = (float) (hsensor->mag_raw[0] / 2281.0f);
		magtemp[1] = (float) (hsensor->mag_raw[1] / 2281.0f);
		magtemp[2] = (float) (hsensor->mag_raw[2] / 2281.0f);
		break;
	case LIS3MDL_Scale_16G:
		hsensor->uTscale = 1711.0f; //   LSB/gauss
		magtemp[0] = (float) (hsensor->mag_raw[0] / 1711.0f);
		magtemp[1] = (float) (hsensor->mag_raw[1] / 1711.0f);
		magtemp[2] = (float) (hsensor->mag_raw[2] / 1711.0f);
		break;
	}

	thisMag.fuTPerCount = 100 / hsensor->uTscale; //  uT/LSB
	thisMag.fCountsPeruT = 1 / thisMag.fuTPerCount; // LSB/uT
	float mtemp[3];
	mtemp[0] = ((magtemp[0] * 100)); //-offset[0]); // convert guass to uT
	mtemp[1] = ((magtemp[1] * 100)); //-offset[1]); // convert guass to uT
	mtemp[2] = ((magtemp[2] * 100)); //-offset[2]); // convert guass to uT

	float mag_alpha = 0.5;
	thisMag.fBpFast[0] = (1-mag_alpha)*thisMag.fBpFast[0] + mag_alpha*mtemp[0];
	thisMag.fBpFast[1] = (1-mag_alpha)*thisMag.fBpFast[1] + mag_alpha*mtemp[1];
	thisMag.fBpFast[2] = (1-mag_alpha)*thisMag.fBpFast[2] + mag_alpha*mtemp[2];

	thisMag.icounter++;

	if (thisMag.icounter == OVERSAMPLE_RATIO) {
		for (int i = 0; i < 3; i++) {

			thisMag.iBp[i] = (int16) (thisMag.iSumBpFast[i] / OVERSAMPLE_RATIO);
			thisMag.fBp[i] = thisMag.iBp[i] * thisMag.fuTPerCount;

			thisMag.iSumBpFast[i] = 0;

			thisMag.icounter = 0;

		}
	}

#define useNXPcal

#ifndef useNXPcal



//#define RED_FC
#define BLUE_FC
//#define NO_CAL

#if defined(RED_FC)
float hardiron[3] = {8.0023,  -22.5746,   -1.2222};

float softiron[3][3] = {{ 1.0283,    0.0230,   -0.0245} ,
		{  0.0230,    1.0361,   -0.0224},
		{   -0.0245,   -0.0224,    0.9401}};


#elif defined(BLUE_FC)

   float hardiron[3] = { 5.8398 , -30.424 , -1.6664};

   float softiron[3][3] = {{    1.000864 ,  0.00646 ,  0.009619} ,
   		{   0.00646  ,  1.0027 ,  0.04013},
   		{    0.009619  , 0.04013 ,   0.9742}};

#elif defined(NO_CAL)

float softiron[3][3] = {{ 1.0f,    0.0f,   0.0f} ,
		{  0.0f,    1.0f,   0.0f},
		{   0.0f,   0.0f,    1.0f}};
float hardiron[3] = {0,  0,  0};

#endif

   float mtemp2[3]= {0,0,0};
for (int i = 0; i < 3; i++) {
	for (int j = 0; j < 3; j++){
		mtemp2[i] += (mtemp[j] - hardiron[j])*softiron[i][j];
	}
}

float alpha = 1;
//hsensor->mag[0] = (1-alpha)*hsensor->mag[0] + alpha*mtemp2[1];
//hsensor->mag[1] = (1-alpha)*hsensor->mag[1] - alpha*mtemp2[0];
//hsensor->mag[2] = (1-alpha)*hsensor->mag[2] - alpha*mtemp2[2];


memcpy(hsensor->mag, mtemp2, sizeof(mtemp2));

hsensor->magfield = sqrt(hsensor->mag[0]*hsensor->mag[0] + hsensor->mag[1]*hsensor->mag[1] + hsensor->mag[2]*hsensor->mag[2]);
#else
	memcpy(hsensor->mag, mtemp, sizeof(mtemp));
#endif

	return LIS3MDL_OK;
}

LIS3MDL_Result_t LIS3MDL_ReadTemp(LIS3MDL_t *hsensor, I2C_HandleTypeDef *hi2c) {
	uint8_t data[2];

	readMultiBytes(hi2c, hsensor->addr, TEMP_OUT_L, data, 2);

	hsensor->temp_raw = ((int16_t) data[1] << 8) | data[0];
	hsensor->temp = (float) (hsensor->temp_raw / 8.0f);

	return LIS3MDL_OK;
}

/* I2C R/W Functions */
static HAL_StatusTypeDef writeByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
		uint8_t register_addr, uint8_t data) {
	uint8_t buffer[2];
	buffer[0] = register_addr;
	buffer[1] = data;
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) device_addr, (uint8_t*) buffer,
			2, 1000) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	return HAL_OK;
}

static HAL_StatusTypeDef readByte(I2C_HandleTypeDef *hi2c, uint8_t device_addr,
		uint8_t register_addr, uint8_t *data) {
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) device_addr, &register_addr, 1,
			1000) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	if (HAL_I2C_Master_Receive(hi2c, (uint16_t) device_addr, data, 1, 1000)
			!= HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	return HAL_OK;
}

static HAL_StatusTypeDef readMultiBytes(I2C_HandleTypeDef *hi2c,
		uint8_t device_addr, uint8_t register_addr, uint8_t *data,
		uint16_t count) {
	if (HAL_I2C_Master_Transmit(hi2c, (uint16_t) device_addr, &register_addr, 1,
			1000) != HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	if (HAL_I2C_Master_Receive(hi2c, (uint16_t) device_addr, data, count, 1000)
			!= HAL_OK) {
		if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF) {
		}
		return HAL_ERROR;
	}
	return HAL_OK;
}

