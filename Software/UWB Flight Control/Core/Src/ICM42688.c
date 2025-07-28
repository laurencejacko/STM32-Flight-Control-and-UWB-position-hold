/*
 * ICM_42688_P.c
 *
 *  Created on: Aug 9, 2023
 *      Author: Laurence Jackson
 */

#include "ICM42688.h"
#include "enumerations.h"

#include "include_all.h"

static float gyro_factor;
static float accel_factor;

uint8_t fifo_data[13];

uint8_t DMA_RxBuf[13];

static void CS_HIGH();
static void CS_LOW();
static void select_bank(bank bx);

extern float AccelB;
extern float GyroB;
float fastDeltaT;


static uint8_t ICM42688_Read_1REG(bank bx, uint8_t addr);
static uint8_t* ICM42688_Read_REGS(bank bx, uint8_t addr, uint8_t len);
static void ICM42688_Write_MULTREG(bank bx, uint8_t addr, uint8_t *data,
		uint8_t len);

void icm42688_init(ICM42688 *imu) {

	/*imu->handle = handle;
	 imu->csPinBank = csPinBank;
	 imu->csPin = csPin;*/

	while (!icm42688_identify())
		;

	icm42688_reset();
	icm42688_boot();
	icm42688_clock_sel();
	//icm42688_FIFO_CONF();
	//icm42688_FIFO_EN();

	ICM42688_gyro_config(_2000dps, _1khz, imu);
	ICM42688_accel_config(_16g, _1khz, imu);
//	LP_LN(imu);
	accel_gyro_bandwidth(_400_odr10, _400_odr10);

	HAL_Delay(50);

	ICM42688_InterruptSetup();

	//ICM42688_CollectCalibration(imu);
	//ICM42688_WriteCalibration(imu);

	imu->DMA_txBuf[0] = read_imu | ACCEL_DATA_X1;
	/*
	 imu->DMA_txBuf[1] = read_imu | ACCEL_DATA_X0;
	 imu->DMA_txBuf[2] = read_imu | ACCEL_DATA_Y1;
	 imu->DMA_txBuf[3] = read_imu | ACCEL_DATA_Y0;
	 imu->DMA_txBuf[4] = read_imu | ACCEL_DATA_Z1;
	 imu->DMA_txBuf[5] = read_imu | ACCEL_DATA_Z0;
	 imu->DMA_txBuf[6] = read_imu | GYRO_DATA_X1;
	 imu->DMA_txBuf[7] = read_imu | GYRO_DATA_X0;
	 imu->DMA_txBuf[8] = read_imu | GYRO_DATA_Y1;
	 imu->DMA_txBuf[9] = read_imu | GYRO_DATA_Y0;
	 imu->DMA_txBuf[10] = read_imu | GYRO_DATA_Z1;
	 imu->DMA_txBuf[11] = read_imu | GYRO_DATA_Z0;

	 imu->DMA_tx = read_imu | ACCEL_DATA_X1;*/

	imu->readingDMA = 0;
	imu->DMA_DONE = false;

	select_bank(b0);

	HAL_Delay(100);

}

void LP_LN(ICM42688 *imu) {

	HAL_Delay(50);
	uint8_t value = ICM42688_Read_1REG(b0, PWR_MGMT0);

	value |= 0x0F;

	ICM42688_Write_1REG(b0, PWR_MGMT0, value);

	HAL_Delay(50);

	value = ICM42688_Read_1REG(b0, PWR_MGMT0);

}

void Icm42688_INT1_START() {

	ICM42688_Write_1REG(b0, INT_SOURCE3, 0x00);	//Stops INT2
	ICM42688_Write_1REG(b0, INT_SOURCE0, 0x08);	//Sets INT1

}

void Icm42688_INT1_STOP() {

	ICM42688_Write_1REG(b0, INT_SOURCE0, 0x00);	//Stops INT1
}

bool icm42688_identify() {

	uint8_t txBuf[2] = { (0x80 | WHO_AM_I), 0x00 };
	uint8_t rxBuf[2];
//	uint8_t txBuf = (0x80 | WHO_AM_I);
//		uint8_t rxBuf;
	select_bank(b0);

	CS_LOW();
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

	HAL_SPI_TransmitReceive(icm42688, txBuf, rxBuf, 2, 1000);
//	HAL_SPI_Transmit(icm42688, (uint8_t*)txBuf,  1, 1000);
	//HAL_SPI_Receive(icm42688, &rxBuf, 1, 1000);

	CS_HIGH();
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);

	/*uint8_t send = read_imu | WHO_AM_I;
	 uint8_t receive;

	 select_bank(b0);

	 CS_LOW();
	 HAL_SPI_Transmit(icm42688, &send,1,100);
	 HAL_SPI_Receive(icm42688, receive, 1, 100);
	 CS_HIGH();*/

	if (rxBuf[1] == ICM42688_ID)
		return true;
	else
		return false;

}

void icm42688_reset() {

	uint8_t data = 0x01; //to reset the register need to set LSB to 1
	ICM42688_Write_1REG(b0, DEVICE_CONFIG, data);
	HAL_Delay(100);

}

void icm42688_FIFO_CONF() {
	uint8_t data = 0x00; //to reset the register need to set LSB to 1
	data |= 1 << 7;
	ICM42688_Write_1REG(b0, FIFO_CONFIG, data);
	HAL_Delay(100);
}

void icm42688_FIFO_EN() {
	uint8_t data = 0x03; //to reset the register need to set LSB to 1
	ICM42688_Write_1REG(b0, FIFO_CONFIG1, data);
	HAL_Delay(100);
}

void icm42688_clock_sel() {
	uint8_t data = 0x91;
	ICM42688_Write_1REG(b0, INTF_CONFIG1, data); //0x91 is the reset value, clk is set by default
	HAL_Delay(100);

}

void icm42688_boot() {

	uint8_t data = 0x0F;
	ICM42688_Write_1REG(b0, PWR_MGMT0, data); //0x1D is the reset value
	HAL_Delay(100);
}

/*
 *
 void self_test(axis* data0, axis* data1) {
 uint8_t data = 0x1F;
 ICM42688_Write_1REG(b0, SELF_TEST_CONFIG, data);
 }
 *
 */

void FIFO_READ(ICM42688 *imu) {

	uint8_t txBuf = read_imu | FIFO_DATA;
	uint8_t rxBuf[16];
	CS_LOW();
	HAL_SPI_Transmit(icm42688, &txBuf, 1, 1000);
	HAL_SPI_Receive(icm42688, rxBuf, 16, 1000);
	CS_HIGH();
	imu->acc_g[0] = (int16_t) ((rxBuf[1] << 8) | rxBuf[2]);
	imu->acc_g[1] = (int16_t) ((rxBuf[3] << 8) | rxBuf[4]);
	imu->acc_g[2] = (int16_t) ((rxBuf[5] << 8) | rxBuf[6]);

	imu->gyr_dps[0] = (int16_t) ((rxBuf[7] << 8) | rxBuf[8]);
	imu->gyr_dps[1] = (int16_t) ((rxBuf[9] << 8) | rxBuf[10]);
	imu->gyr_dps[2] = (int16_t) ((rxBuf[11] << 8) | rxBuf[12]);

	imu->acc_g[0] /= 2048;
	imu->acc_g[1] /= 2048;
	imu->acc_g[2] /= 2048;

	imu->gyr_dps[0] /= 16.4;
	imu->gyr_dps[1] /= 16.4;
	imu->gyr_dps[2] /= 16.4;

	int16_t temp = (((rxBuf[13] << 8) | rxBuf[14]) / 2.07) + 25;

}

void ICM42688_CollectCalibration(ICM42688 *imu) {

	int TOT = 10000;

	imu->offset_g[0] = 0;
	imu->offset_g[1] = 0;
	imu->offset_g[2] = 0;

	imu->offset_a[0] = 0;
	imu->offset_a[1] = 0;
	imu->offset_a[2] = 0;

	for (int i = 0; i < TOT; i++) {

		ICM42688_ReadData(imu);
		//ICM42688_ReadDMA_Complete

		imu->offset_g[1] += imu->gyr_dps[0];
		imu->offset_g[0] += imu->gyr_dps[1];
		imu->offset_g[2] += imu->gyr_dps[2];

		imu->offset_a[1] -= imu->acc_g[0];
		imu->offset_a[0] -= imu->acc_g[1];
		imu->offset_a[2] -= (imu->acc_g[2] - 1);

	}

	imu->offset_g[0] /= TOT;
	imu->offset_g[1] /= TOT;
	imu->offset_g[2] /= TOT;

	imu->offset_a[0] /= TOT;
	imu->offset_a[1] /= TOT;
	imu->offset_a[2] /= TOT;

}

void ICM42688_WriteCalibration(ICM42688 *imu) {

	//Function to write_imu calibration values to IMU

	int16_t gyro_offset[3];

	gyro_offset[0] = 32.00f * imu->offset_g[0];
	gyro_offset[1] = 32.00f * imu->offset_g[1];
	gyro_offset[2] = 32.00f * imu->offset_g[2];

	int16_t accel_offset[3];

	accel_offset[0] = 2000.00f * imu->offset_a[0];
	accel_offset[1] = 2000.00f * imu->offset_a[1];
	accel_offset[2] = 2000.00f * imu->offset_a[2];

	uint8_t nibble1, nibble2;

	uint8_t txBuf[9];
	uint8_t addr = write_imu | OFFSET_USER0; //write_imu starting address
	txBuf[0] = gyro_offset[0] & 0xFF; //GyroX[7:0]

	nibble1 = (gyro_offset[1] >> 8) & 0xFF;
	nibble2 = (gyro_offset[0] >> 8) & 0xFF;
	txBuf[1] = (nibble1 << 4) | (nibble2 & 0x0F); //[GyroY[11:8],GyroX[11:8]]

	txBuf[2] = gyro_offset[1] & 0xFF; //GyroY[7:0]
	txBuf[3] = gyro_offset[2] & 0xFF; //GyroZ[7:0]

	nibble1 = (accel_offset[0] >> 8) & 0xFF;
	nibble2 = (gyro_offset[2] >> 8) & 0xFF;
	txBuf[4] = (nibble1 << 4) | (nibble2 & 0x0F); //[AccelX[11:8], GyroZ[11:8]]

	txBuf[5] = accel_offset[0] & 0xFF; //AccelX[7:0]
	txBuf[6] = accel_offset[1] & 0xFF; //AccelY[7:0]

	nibble1 = (accel_offset[2] >> 8) & 0xFF;
	nibble2 = (accel_offset[1] >> 8) & 0xFF;
	txBuf[7] = (nibble1 << 4) | (nibble2 & 0x0F); //[AccelZ[11:8], AccelY[11:8]]

	txBuf[8] = accel_offset[2] & 0xFF; //AccelZ[7:0]

	uint8_t OFF0[2] = { (write_imu | OFFSET_USER0), txBuf[0] };
	uint8_t OFF1[2] = { (write_imu | OFFSET_USER1), txBuf[1] };
	uint8_t OFF2[2] = { (write_imu | OFFSET_USER2), txBuf[2] };
	uint8_t OFF3[2] = { (write_imu | OFFSET_USER3), txBuf[3] };
	uint8_t OFF4[2] = { (write_imu | OFFSET_USER4), txBuf[4] };
	uint8_t OFF5[2] = { (write_imu | OFFSET_USER5), txBuf[5] };
	uint8_t OFF6[2] = { (write_imu | OFFSET_USER6), txBuf[6] };
	uint8_t OFF7[2] = { (write_imu | OFFSET_USER7), txBuf[7] };
	uint8_t OFF8[2] = { (write_imu | OFFSET_USER8), txBuf[8] };

	select_bank(b4);

	CS_LOW();
	HAL_SPI_Transmit(icm42688, OFF0, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF1, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF2, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF3, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF4, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF5, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF6, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF7, 2, 100);
	HAL_SPI_Transmit(icm42688, OFF8, 2, 100);
	CS_HIGH();

	HAL_Delay(100);

	ICM42688_ReadData(imu);

}

//Might want to replace struct 'axis' for general IMU struct

uint8_t ICM42688_ReadDMA(ICM42688 *imu, BLOCKING *state, uint8_t *txBuf,
		uint8_t *rxBuf) {

	//uint8_t DMA_TxBuf[13];
	//DMA_TxBuf[0] = read_imu | ACCEL_DATA_X1;

//	uint8_t txBuf[13];
//	txBuf[0] = read_imu | TEMP_DATA1;
//  uint8_t temp[13];

	//HAL_SPI_Transmit_DMA(icm42688, &tx, 1);
	//if (HAL_SPI_Receive_DMA(icm42688, imu->DMA_rxBuf, 12) == HAL_OK) {

	txBuf[0] = read_imu | TEMP_DATA1;

	CS_LOW();
	if (HAL_SPI_TransmitReceive_DMA(icm42688, txBuf, rxBuf, 15) == HAL_OK) {

		imu->readingDMA = 1;
		return 1;
		//ICM42688_Read_1REG(b0, INT_STATUS); //Reads and clear interrupt registers after completing

	} else {

		CS_HIGH();
		state->state = spi_ready;
		return 0;
	}

}

void ICM42688_ReadDMA_Complete(ICM42688 *imu, uint8_t *rxBuf) {

	CS_HIGH();
	//imu->processedDMA = true;
	imu->readingDMA = 0;

	float g = 9.81f;
	float deg2rad = 0.0174533f;

	/*imu->acc_g[0] = (int16_t)((DMA_RxBuf[1] << 8) | DMA_RxBuf[2]) / (imu->accel_factor);
	 imu->acc_g[1] = (int16_t)((DMA_RxBuf[3] << 8) | DMA_RxBuf[4]) / (imu->accel_factor);
	 imu->acc_g[2] = (int16_t)((DMA_RxBuf[5] << 8) | DMA_RxBuf[6]) / (imu->accel_factor);

	 imu->gyr_dps[0] =  (int16_t)((DMA_RxBuf[7] << 8) | DMA_RxBuf[8]) / (imu->gyro_factor);
	 imu->gyr_dps[1] =  (int16_t)((DMA_RxBuf[9] << 8) | DMA_RxBuf[10]) / (imu->gyro_factor);
	 imu->gyr_dps[2] =  (int16_t)((DMA_RxBuf[11] << 8) | DMA_RxBuf[12]) / (imu->gyro_factor);

	 imu->acc_mps2[0] = g * imu->acc_g[0];
	 imu->acc_mps2[1] = g * imu->acc_g[1];
	 imu->acc_mps2[2] = g * imu->acc_g[2];

	 imu->gyr_rad[0] = deg2rad * imu->gyr_dps[0];
	 imu->gyr_rad[1] = deg2rad * imu->gyr_dps[1];
	 imu->gyr_rad[2] = deg2rad * imu->gyr_dps[2];*/

	/*	imu->acc_g[0] = (int16_t)((imu->DMA_rxBuf[1] << 8) | imu->DMA_rxBuf[2]) / (imu->accel_factor);
	 imu->acc_g[1] = (int16_t)((imu->DMA_rxBuf[3] << 8) | imu->DMA_rxBuf[4]) / (imu->accel_factor);
	 imu->acc_g[2] = (int16_t)((imu->DMA_rxBuf[5] << 8) | imu->DMA_rxBuf[6]) / (imu->accel_factor);

	 imu->gyr_dps[0] =  (int16_t)((imu->DMA_rxBuf[7] << 8) | imu->DMA_rxBuf[8]) / (imu->gyro_factor);
	 imu->gyr_dps[1] =  (int16_t)((imu->DMA_rxBuf[9] << 8) | imu->DMA_rxBuf[10]) / (imu->gyro_factor);
	 imu->gyr_dps[2] =  (int16_t)((imu->DMA_rxBuf[11] << 8) | imu->DMA_rxBuf[12]) / (imu->gyro_factor);

	 imu->acc_mps2[0] = g * imu->acc_g[0];
	 imu->acc_mps2[1] = g * imu->acc_g[1];
	 imu->acc_mps2[2] = g * imu->acc_g[2];

	 imu->gyr_rad[0] = deg2rad * imu->gyr_dps[0];
	 imu->gyr_rad[1] = deg2rad * imu->gyr_dps[1];
	 imu->gyr_rad[2] = deg2rad * imu->gyr_dps[2];

	 imu->DMA_DONE = true;
	 */
	float acc[3];
	float gyr[3];

	acc[0] = (int16_t) ((rxBuf[3] << 8) | rxBuf[4]) / (imu->accel_factor);
	acc[1] = (int16_t) ((rxBuf[5] << 8) | rxBuf[6]) / (imu->accel_factor);
	acc[2] = (int16_t) ((rxBuf[7] << 8) | rxBuf[8]) / (imu->accel_factor);

	gyr[0] = (int16_t) ((rxBuf[9] << 8) | rxBuf[10]) / (imu->gyro_factor);
	gyr[1] = (int16_t) ((rxBuf[11] << 8) | rxBuf[12]) / (imu->gyro_factor);
	gyr[2] = (int16_t) ((rxBuf[13] << 8) | rxBuf[14]) / (imu->gyro_factor);

	if (acc[0] && acc[1] && acc[2] != 0) {
		imu->acc_g[0] = (1.0 - AccelB) * imu->acc_g[0] + AccelB * (acc[0]);
		imu->acc_g[1] = (1.0 - AccelB) * imu->acc_g[1] + AccelB * (acc[1]);
		imu->acc_g[2] = (1.0 - AccelB) * imu->acc_g[2] + AccelB * (acc[2]);




	}

	if (gyr[0] && gyr[1] && gyr[2] != 0) {

		imu->gyr_dps[0] = (1.0 - GyroB) * imu->gyr_dps[0] + GyroB * (gyr[0]);
		imu->gyr_dps[1] = (1.0 - GyroB) * imu->gyr_dps[1] + GyroB * (gyr[1]);
		imu->gyr_dps[2] = (1.0 - GyroB) * imu->gyr_dps[2] + GyroB * (gyr[2]);
	}

	//	  B_accel = lowpass_to_beta(LP_accel, loop_freq);
	//	  AccX = (1.0 - B_accel) * AccX + B_accel * (imu.ax - cfg.imu_cal_ax);

	//	imu->acc_mps2[0] = g * imu->acc_g[0];
	//	imu->acc_mps2[1] = g * imu->acc_g[1];
	//	imu->acc_mps2[2] = g * imu->acc_g[2];

//		imu->gyr_rad[0] = deg2rad * imu->gyr_dps[0];
//   	imu->gyr_rad[1] = deg2rad * imu->gyr_dps[1];
//		imu->gyr_rad[2] = deg2rad * imu->gyr_dps[2];

	//CS_LOW();
	//CS_HIGH();

}

void ICM42688_InterruptSetup() {

	ICM42688_Write_1REG(b0, INT_CONFIG, 0x03);
	//ICM42688_Write_1REG(b0, INT_CONFIG0, 0x20);

	//uint8_t reg = ICM42688_Read_1REG(b0, INT_CONFIG1);
	//reg &= ~0x10; //Changes Bit  4 to a 0

	ICM42688_Write_1REG(b0, INT_CONFIG1, 0x10); //NEEDS CHECKING
	//HAL_Delay(10);

	ICM42688_Write_1REG(b0, INT_SOURCE3, 0x08);	//Sets INT2
	//ICM42688_Write_1REG(b0, INT_SOURCE0, 0x00);//Unsets INT1

}

void ICM42688_ReadData(ICM42688 *imu) {

	//uint8_t txBuf = read_imu | TEMP_DATA1;
	uint8_t temp[15];

	uint8_t txBuf[15];
	txBuf[0] = read_imu | TEMP_DATA1;

	select_bank(b0);

	CS_LOW();
	//HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
	//HAL_SPI_Receive(icm42688, temp, 14, 100);

	HAL_SPI_TransmitReceive(icm42688, txBuf, temp, 15, 100);

	CS_HIGH();

	imu->temperature = (int16_t) (temp[1] << 8 | temp[2]) / (128);
	imu->temperature += 25;

	float acc1[3];
	float gyr1[3];
	//Map to NED, N = +y, E = +x, D = +z
	acc1[1] = (int16_t) (temp[3] << 8 | temp[4]) / (imu->accel_factor);
	acc1[0] = (int16_t) (temp[5] << 8 | temp[6]) / (imu->accel_factor);
	acc1[2] = (int16_t) (temp[7] << 8 | temp[8]) / (imu->accel_factor);

	gyr1[1] = (int16_t) ((temp[9] << 8) | temp[10]) / (-imu->gyro_factor);
	gyr1[0] = (int16_t) ((temp[11] << 8) | temp[12]) / (-imu->gyro_factor);
	gyr1[2] = (int16_t) ((temp[13] << 8) | temp[14]) / (-imu->gyro_factor);


	if (acc1[0] && acc1[1] && acc1[2] != 0) {

	imu->acc_g[0] = (1.0 - AccelB) * imu->acc_g[0] + AccelB * (acc1[0]);
	imu->acc_g[1] = (1.0 - AccelB) * imu->acc_g[1] + AccelB * (acc1[1]);
	imu->acc_g[2] = (1.0 - AccelB) * imu->acc_g[2] + AccelB * (acc1[2]);

	imu->acc_mps2[0] = 9.81 * imu->acc_g[0];
	imu->acc_mps2[1] = 9.81 * imu->acc_g[1];
	imu->acc_mps2[2] = 9.81 * imu->acc_g[2];

	thisAccel.iGpFast[0] = (int32)(acc1[0]*(imu->accel_factor));
	thisAccel.iGpFast[1] = (int32)(acc1[1]*(imu->accel_factor));
	thisAccel.iGpFast[2] = (int32)(acc1[2]*(imu->accel_factor));

	thisAccel.iSumGpFast[0] += thisAccel.iGpFast[0];
	thisAccel.iSumGpFast[1] += thisAccel.iGpFast[1];
	thisAccel.iSumGpFast[2] += thisAccel.iGpFast[2];

	thisAccel.fSumGpFast[0] += imu->acc_g[0];
	thisAccel.fSumGpFast[1] += imu->acc_g[1];
	thisAccel.fSumGpFast[2] += imu->acc_g[2];


	thisAccel.fGpFast[0] = imu->acc_g[0] ;
	thisAccel.fGpFast[1] = imu->acc_g[1] ;
	thisAccel.fGpFast[2] = imu->acc_g[2] ;

	thisAccel.icounter++;

    if (thisAccel.icounter == OVERSAMPLE_RATIO) {
    	   for (int i = 0; i < 3; i++) {

    		   thisAccel.iGp[i] = (int16)(thisAccel.iSumGpFast[i] / OVERSAMPLE_RATIO);
//    		   thisAccel.fGp[i] = thisAccel.iGp[i] / imu->accel_factor;

    		   thisAccel.fGp[i] = thisAccel.fSumGpFast[i] / OVERSAMPLE_RATIO;


			   thisAccel.iSumGpFast[i] = 0;
			   thisAccel.fSumGpFast[i] = 0;



              }
    	   thisAccel.icounter = 0;
       }

	}

	if (gyr1[0] && gyr1[1] && gyr1[2] != 0) {
	imu->gyr_dps[0] = (1.0 - GyroB) * imu->gyr_dps[0] + GyroB * (gyr1[0]);
	imu->gyr_dps[1] = (1.0 - GyroB) * imu->gyr_dps[1] + GyroB * (gyr1[1]);
	imu->gyr_dps[2] = (1.0 - GyroB) * imu->gyr_dps[2] + GyroB * (gyr1[2]);
	}
//	imu->acc_g[2] *= -1;
//	imu->gyr_dps[2] *= -1;

	/*	float g = 9.81f;
	 float deg2rad = 0.0174533f;

	 imu->acc_mps2[0] = g * imu->acc_g[0];
	 imu->acc_mps2[1] = g * imu->acc_g[1];
	 imu->acc_mps2[2] = g * imu->acc_g[2];

	 imu->gyr_rad[0] = deg2rad * imu->gyr_dps[0];
	 imu->gyr_rad[1] = deg2rad * imu->gyr_dps[1];
	 imu->gyr_rad[2] = deg2rad * imu->gyr_dps[2];
	 */

	//imu->N = imu->N + 1;
	imu->readingDMA = true;
}
/*
 void ICM42688_Gyro_Read(axis* value){
 //uint8_t* temp = ICM42688_Read_REGS(b0, GYRO_DATA_X1,6 );

 //uint8_t txBuf = {(GYRO_DATA_X1 | read_imu)};//,(GYRO_DATA_X0|read_imu),(GYRO_DATA_Y1|read_imu),(GYRO_DATA_Y0|read_imu),(GYRO_DATA_Z1|read_imu),(GYRO_DATA_Z0|read_imu)};
 /*uint8_t temp[6]= {0x00, 0x00,0x00,0x00,0x00,0x00};

 CS_LOW();
 HAL_SPI_TransmitReceive(icm42688, &txBuf, temp, 6, 1000);
 CS_HIGH();

 uint8_t txBuf = read_imu | GYRO_DATA_X1;
 uint8_t temp[6];



 select_bank(b0);

 CS_LOW();
 HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
 HAL_SPI_Receive(icm42688, temp, 6, 100);
 CS_HIGH();



 value->x =  (int16_t)((temp[0] << 8) | temp[1]);
 value->y =  (int16_t)((temp[2] << 8) | temp[3]);
 value->z =  (int16_t)((temp[4] << 8) | temp[5]);
 }

 void ICM42688__Read_DPS(ICM42688 *imu){

 ICM42688_Gyro_Read(imu);

 imu->x /=  gyro_factor;
 value->y /=  gyro_factor;
 value->z /=  gyro_factor;
 }


 void ICM42688_Accel_Read(axis* value){


 //uint8_t txBuf = {(ACCEL_DATA_X1 | read_imu)};//,(ACCEL_DATA_X0|read_imu),(ACCEL_DATA_Y1|read_imu),(ACCEL_DATA_Y0|read_imu),(ACCEL_DATA_Z1|read_imu),(ACCEL_DATA_Z0|read_imu)};
 /*uint8_t temp[6]= {0x00, 0x00,0x00,0x00,0x00,0x00};

 CS_LOW();
 HAL_SPI_TransmitReceive(icm42688, &txBuf, temp, 6, 1000);
 CS_HIGH();

 uint8_t txBuf = read_imu | ACCEL_DATA_X1;
 uint8_t temp[6]; //might need changing to a constant like '3'

 select_bank(b0);

 CS_LOW();
 HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
 HAL_SPI_Receive(icm42688, temp, 6, 100);
 CS_HIGH();


 value->x =  (int16_t)(temp[0] << 8 | temp[1]);
 value->y =  (int16_t)(temp[2] << 8 | temp[3]);
 value->z =  (int16_t)(temp[4] << 8 | temp[5]);


 /*
 uint8_t* temp = ICM42688_Read_REGS(b0, ACCEL_DATA_X1,6 );
 value->x =  (int16_t)(temp[0] << 8 | temp[1]);
 value->y =  (int16_t)(temp[2] << 8 | temp[3]);
 value->z =  (int16_t)(temp[4] << 8 | temp[5]);
 }

 void ICM42688_Accel_Read_DPS(axis* value){



 ICM42688_Accel_Read(value);

 value->x /=  accel_factor;
 value->y /=  accel_factor;
 value->z /=  accel_factor;
 }*/

void ICM42688_gyro_config(gyro_scale scale, ODR odr, ICM42688 *imu) {

	uint8_t value = ICM42688_Read_1REG(b0, GYRO_CONFIG0);

	switch (scale) {

	case _15_625dps:
		value |= 7 << 5;
		imu->gyro_factor = 2097.2;
		break;
	case _31_25dps:
		value |= 6 << 5;
		imu->gyro_factor = 1048.6;
		break;
	case _62_5dps:
		value |= 5 << 5;
		imu->gyro_factor = 524.3;
		break;
	case _125dps:
		value |= 4 << 5;
		imu->gyro_factor = 262;
		break;
	case _250dps:
		value |= 3 << 5;
		imu->gyro_factor = 131;
		break;
	case _500dps:
		value |= 2 << 5;
		imu->gyro_factor = 65.5;
		break;
	case _1000dps:
		value |= 1 << 5;
		imu->gyro_factor = 32.8;
		break;
	case _2000dps:
		value |= 0 << 5;
		imu->gyro_factor = 16.4;
		break;
	}

	switch (odr) {
	case _500hz:
		value |= 15;
		fastDeltaT = 1.0f/500;
		break;
	case _12_5hz:
		value |= 11;
		fastDeltaT = 1.0f/12.5;
		break;
	case _25hz:
		value |= 10;
		fastDeltaT = 1.0f/25;
		break;
	case _50hz:
		value |= 9;
		fastDeltaT = 1.0f/50;
		break;
	case _100hz:
		value |= 8;
		fastDeltaT = 1.0f/100;
		break;
	case _200hz:
		value |= 7;
		fastDeltaT = 1.0f/200;
		break;
	case _1khz:
		value |= 6;
		fastDeltaT = 1.0f/1000;
		break;
	case _2khz:
		value |= 5;
		fastDeltaT = 1.0f/2000;

		break;
	case _4khz:
		value |= 4;
		fastDeltaT = 1.0f/4000;

		break;
	case _8khz:
		value |= 3;
		fastDeltaT = 1.0f/8000;

		break;
	case _16khz:
		value |= 2;
		fastDeltaT = 1.0f/16000;

		break;
	case _32khz:
		value |= 1;
		fastDeltaT = 1.0f/32000;

		break;
	}

	ICM42688_Write_1REG(b0, GYRO_CONFIG0, value);

	HAL_Delay(20);
//	value = ICM42688_Read_1REG(b0, GYRO_CONFIG0);

}

void ICM42688_accel_config(accel_scale scale, ODR odr, ICM42688 *imu) {

	uint8_t value = ICM42688_Read_1REG(b0, ACCEL_CONFIG0);

	switch (scale) {

	case _2g:
		value |= 3 << 5;
		imu->accel_factor = 16384;
		break;
	case _4g:
		value |= 2 << 5;
		imu->accel_factor = 8192;
		break;
	case _8g:
		value |= 1 << 5;
		imu->accel_factor = 4096;
		break;
	case _16g:
		value |= 0 << 5;
		imu->accel_factor = 2048;
		break;
	}

	switch (odr) {
	case _500hz:
		value |= 15;
		break;
	case _1_5625hz:
		value |= 14;
		break;
	case _3_125hz:
		value |= 13;
		break;
	case _6_25hz:
		value |= 12;
		break;
	case _12_5hz:
		value |= 11;
		break;
	case _25hz:
		value |= 10;
		break;
	case _50hz:
		value |= 9;
		break;
	case _100hz:
		value |= 8;
		break;
	case _200hz:
		value |= 7;
		break;
	case _1khz:
		value |= 6;
		break;
	case _2khz:
		value |= 5;
		break;
	case _4khz:
		value |= 4;
		break;
	case _8khz:
		value |= 3;
		break;
	case _16khz:
		value |= 2;
		break;
	case _32khz:
		value |= 1;
		break;
	}

	ICM42688_Write_1REG(b0, ACCEL_CONFIG0, value);
	HAL_Delay(20);
	//value = ICM42688_Read_1REG(b0, ACCEL_CONFIG0);

}

void accel_gyro_bandwidth(COMMON_LPF ABW, COMMON_LPF GBW) {

	uint8_t value = ICM42688_Read_1REG(b0, GYRO_ACCEL_CONFIG0);

	switch (ABW) {

	case _odr2:
		value |= 0 << 4;
		break;
	case _400_odr4:
		value |= 1 << 4;
		break;
	case _400_odr5:
		value |= 2 << 4;
		break;
	case _400_odr8:
		value |= 3 << 4;
		break;
	case _400_odr10:
		value |= 4 << 4;
		break;
	case _400_odr16:
		value |= 5 << 4;
		break;
	case _400_odr20:
		value |= 6 << 4;
		break;
	case _400_odr40:
		value |= 7 << 4;
		break;
	case _400_odr_lowlatency:
		value |= 14 << 4;
		break;
	case _200_8odr_lowlatency:
		value |= 15 << 4;
		break;

	}

	switch (GBW) {

	case _odr2:
		value |= 0;
		break;
	case _400_odr4:
		value |= 1;
		break;
	case _400_odr5:
		value |= 2;
		break;
	case _400_odr8:
		value |= 3;
		break;
	case _400_odr10:
		value |= 4;
		break;
	case _400_odr16:
		value |= 5;
		break;
	case _400_odr20:
		value |= 6;
		break;
	case _400_odr40:
		value |= 7;
		break;
	case _400_odr_lowlatency:
		value |= 14;
		break;
	case _200_8odr_lowlatency:
		value |= 15;
		break;
	}

	ICM42688_Write_1REG(b0, GYRO_ACCEL_CONFIG0, value);

}

static void CS_HIGH() {
	HAL_GPIO_WritePin(icm42688_port, icm42688_pin, GPIO_PIN_SET);
}

static void CS_LOW() {
	HAL_GPIO_WritePin(icm42688_port, icm42688_pin, GPIO_PIN_RESET);
}

static void select_bank(bank bx) {
	uint8_t txBuf[2];
	txBuf[0] = write_imu | REG_BANK_SEL;
	txBuf[1] = bx;

	CS_LOW();
	HAL_SPI_Transmit(icm42688, txBuf, 2, 100);
	CS_HIGH();
}

static uint8_t ICM42688_Read_1REG(bank bx, uint8_t addr) {

	uint8_t txBuf = read_imu | addr;
	uint8_t rxBuf;

	select_bank(bx);

	CS_LOW();
	HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
	HAL_SPI_Receive(icm42688, rxBuf, 1, 100);
	CS_HIGH();

	return rxBuf;

}

static uint8_t* ICM42688_Read_REGS(bank bx, uint8_t addr, uint8_t len) {

	uint8_t txBuf = read_imu | addr;

	uint8_t rxBuf[6]; //might need changing to a constant like '3'

	select_bank(bx);

	CS_LOW();
	HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
	HAL_SPI_Receive(icm42688, rxBuf, 6, 100);
	CS_HIGH();

	return rxBuf;

}

void ICM42688_Write_1REG(bank bx, uint8_t addr, uint8_t data) {

	uint8_t txBuf[2];
	txBuf[0] = write_imu | addr;
	txBuf[1] = data;

	select_bank(bx);

	CS_LOW();
	HAL_SPI_Transmit(icm42688, txBuf, 2, 100);
	CS_HIGH();

}

static void ICM42688_Write_MULTREG(bank bx, uint8_t addr, uint8_t *data,
		uint8_t len) {

	uint8_t txBuf = write_imu | addr;
	select_bank(bx);

	CS_LOW();
	HAL_SPI_Transmit(icm42688, &txBuf, 1, 100);
	HAL_SPI_Transmit(icm42688, data, len, 100);
	CS_HIGH(); //"&" passes the memory address of the variable, allows the function to read_imu and update the original variable
			   //"*" passes the value stored at the memory address, can not modify the original variable

}
/* DEFINED IN main.c
 *
 uint8_t ICM42688_Read(uint8_t addr, uint8_t *data) {

 uint8_t txBuf[2] = { (addr | read_imu), 0x00 };
 uint8_t rxBuf[2];

 CS_LOW();
 uint8_t status = (HAL_SPI_TransmitReceive(icm42688, txBuf, rxBuf, 2, 100) == HAL_OK);
 CS_HIGH();
 *data = rxBuf[1];

 return status;

 }

 uint8_t ICM42688_Write(uint8_t addr, uint8_t *data) {

 uint8_t txBuf[2] = { (addr | write_imu), 0x00 };
 //uint8_t rxBuf[2];

 HAL_GPIO_WritePin(SPI4_NCS_GPIO_Port, SPI4_NCS_Pin, GPIO_PIN_RESET);
 uint8_t status = (HAL_SPI_Transmit(icm42688, txBuf, 1, 100)
 == HAL_OK);
 HAL_GPIO_WritePin(SPI4_NCS_GPIO_Port, SPI4_NCS_Pin, GPIO_PIN_SET);

 *data = txBuf[1];

 return status;

 }
 */
