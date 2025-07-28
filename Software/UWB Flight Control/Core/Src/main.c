/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "stdio.h"
#include "servo_lib.h"
#include "LIS3MDL.h"
#include "ICM42688.h"
#include "EKF_3D.h"
#include <MS5607SPI.h>

#include "GNSS.h"
#include "Stateest.h"

#include "w25qxx_littlefs.h"
#include "lfs.h"

//Radio includes
#include "mode.h"
#include "delay.h"
#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_radio.h"
#include <stdint.h>
#include <stdbool.h>
#include "stdio.h"

#include "dshot.h"

// Sensor Fusion Headers
//#include "sensor_fusion_class.h"
//#include "board.h"   // hardware-specific settings. Edit as needed for board & sensors.
//#include "build.h"   // sensor fusion configuration options. Edit as needed.

//#include "micros.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define BUFFER_SIZE 200
#define RF_BL_ADV_CHANNEL_38                        2478000000 // Hz
#define RF_FREQUENCY                                RF_BL_ADV_CHANNEL_38 // Hz
#define TX_OUTPUT_POWER                             13
#define TX_TIMEOUT_VALUE                            1000 // ms

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
LIS3MDL_t mag;
ICM42688 imu;
BLOCKING BLOCK;
AHRS0 ahrs;
State_Vector state;
LoraMotor LM;
void OnTxDone(void);
void OnRxDone(void);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(IrqErrorCode_t);
void radio_functions(void);

float linear_interp(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min,
		float out_max);
float lowpass_to_beta(float f0, float fs);
RadioCallbacks_t Callbacks = { &OnTxDone,        // txDone
		&OnRxDone,        // rxDone
		NULL,             // syncWordDone
		NULL,             // headerDone
		&OnTxTimeout,     // txTimeout
		&OnRxTimeout,     // rxTimeout
		&OnRxError,       // rxError
		NULL,             // rangingDone
		NULL,             // cadDone
		};

uint8_t BufferSize = BUFFER_SIZE;

uint8_t Buffer[BUFFER_SIZE];
int8_t rssi = 0;
uint8_t Tx_Msg[4], Rx_Msg[4];
uint16_t RxIrqMask = IRQ_RX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR;
uint16_t TxIrqMask = IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT | IRQ_CRC_ERROR;
AppStates_t AppState = APP_LOWPOWER;
PacketStatus_t packetStatus;

pkt_t pkt;
uint8_t mode = 0;             // 0 lora 1 GFSK 2 FLRC
size_t packet = BUFFER_SIZE;
//uint8_t txTEST[] = "PING PONG DING DONG";

#include <stdio.h>
int _write(int file, char *ptr, int len) {
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}
bool armed_drone = false;
bool collectpsi = true;
/*
 void testPC(){
 float data1 = 123;
 float data2 = 456;
 char buffer[100];
 sprintf(buffer, "%0.3f, %0.3f \r\n",data1, data2);

 test_run_info((char)buffer);
 }*/

void test_run_info(unsigned char *data) {
	uint16_t data_length;

	data_length = strlen((const char*) data);
	CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
	//CDC_Transmit_FS((uint8_t*) "\n\r", 2); /*Transmit end of line through USB - Virtual port*/
}
void test_run_info_rxBuf(unsigned char *data, uint16_t data_length) {
	//uint16_t data_length;

	//data_length = strlen((const char*) data);
	CDC_Transmit_FS(data, data_length); /*Transmit the data through USB - Virtual port*/
//	CDC_Transmit_FS((uint8_t*) "\n\r", 2); /*Transmit end of line through USB - Virtual port*/
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SENSOR_REFRESH_RATE 100
#define UTIL_REFRESH_RATE	100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int CrsfChannels[CRSF_NUM_CHANNELS];
servo_config M1;  // create servo ----
servo_config M2;  // create servo ----
servo_config M3;  // create servo ----
servo_config M4;  // create servo ----

uint32_t micros_us();
long weightA;
long weightB;
int count;

void GNSS_READ(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern GNSS_StateHandle GNSS_Handle;
extern LAT_LONG lat;
extern LAT_LONG lon;

kalman_filter_t EKF_X;
kalman_filter_t EKF_Y;
kalman_filter_t EKF_Z;

#include <stdio.h>

//int throttle = 0;
float roll = 0;
float pitch = 0;
float yaw = 0;
int M1_c, M2_c, M3_c, M4_c;
extern int throttle;

float magValue[3];
float mag_temp;
uint8_t txBufimu[15];
uint8_t rxBufimu[15];
uint32_t dt = 0;

uint32_t time1;
int Nm = 0;
int Na = 0;
int sample_tot = 100;

float m0[3] = { 0, 0, 0 };
float a0[3] = { 0, 0, 0 };

uint32_t disconnect_time = 0;

uint8_t SECTOR[0x800]; //4bytes
uint32_t addrBegin = 0;
uint8_t endMarker[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint32_t sectorsize = 0x800;
uint32_t finalIndex = 0;
uint32_t addrFLASH = 0;
extern struct MS5607Readings readings;
#define GPSconv 1e9
uint32_t counter_FLASH = 0;
void FLASH_LOG();

char formatSTRING[] = "%d,%d, %0.1f, %0.1f, %0.3f, %0.3f,";

float rollPID_l, pitchPID_l, yawPID_l;
float psi_init = 0;
bool psi_initval = false;
float PitchB, RollB;

float angleki_store;

char txBuf_LittleFS[256];

uint32_t initT;

#include "include_all.h"

struct MagSensor thisMag;					// this magnetometer
struct MagCalibration thisMagCal;	// hard and soft iron magnetic calibration
struct MagneticBuffer thisMagBuffer;		// magnetometer measurement buffer

struct AccelSensor thisAccel;				// this accelerometer

void PID_Execute_Acro(void);
void PID_Execute(void);

lfs_file_t file; // = flightstats, loratelem, airbrakedata, catsekf
extern lfs_t littlefs;

char filename[] = "Drone Flights (UWB)";
uint32_t sync_counter = 0;
lfs_soff_t sizereturned = 0;

uint32_t tick_debug = 0;
uint32_t initTime = 0;

float gyro_data[3][100];
bool imu_error = false;

float e[3];
extern float r[3];
extern float mot_offset;
float esum[3];
float kdPID[3];

float kp = 0.3;
float ki = 0;
float kd = 0;
float rollPID, pitchPID, yawPID;

#if defined(dataPrint)
int printData = 1;
#else
int printData = 0;
#endif

float RateB;

float AngleKP = 0.2; //kp for angle control (roll, pitch)
float AngleKI = 0.05; //ki for angle control (roll, pitch)

int i_limit = 25; //max integral term for angle control

float GyroKP = 0.25; //kp for rate control (roll, pitch)
float GyroKI = 0.1;
float GyroKD = 0.0002; //kd for rate control (roll, pitch)

//float GyroKD = 0; //kd for rate control (roll, pitch)

//float AngleKP = 0; //kp for angle control (roll, pitch)
//float AngleKI = 0; //ki for angle control (roll, pitch)
//
//int i_limit = 25; //max integral term for angle control
//
//float GyroKP = 0; //kp for rate control (roll, pitch)
//float GyroKD = 0; //kd for rate control (roll, pitch)

float velKP = 10; //kp for velocity controller
float velKI = 0; //ki for velocity controller

float GyroKP_Yaw = 0.3;
float GyroKI_Yaw = 0.05;
float GyroKD_Yaw = 0.0002;

//float GyroKD_Yaw = 0;

float GyroPID[3];

float angleR[3];

float Kl = 30.0; // 1 deg of angle error should be corrected at X deg per second of angle rate --OR-- 1 deg of error should be corrected in 1/X seconds
bool Uwbtoggleon = false;
bool UWB_data;
int uwbLock;
bool setpoint_collected = false;

int throttleUWB_setP;

int uwb_en;

float Xacc, Yacc, Zacc;
float Xdes, Ydes;
int armed_status = 0;

float rE[2];

void FLASH_LOG() {

//#define debuggin
#ifndef debuggin
	char txBuf_FLASHA[600];
	char txBuf_FLASHB[300];
	//need to combine data into a buffer
	memset(txBuf_FLASHA, 0, sizeof(txBuf_FLASHA));
	memset(txBuf_FLASHB, 0, sizeof(txBuf_FLASHB));

	counter_FLASH++;

	state.time = HAL_GetTick() - initTime;

	if ((UWB_data == true) && (setpoint_collected == true)) {
		uwbLock = 1;
	} else {
		uwbLock = 0;
	}

#if defined(LittleFS_Flash)
	snprintf(txBuf_FLASHA, sizeof(txBuf_FLASHA),

			"%0.3f, %0.3f, %0.3f, "

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"
					"%0.3f, %0.3f,"

					"%0.3f,"

					"%d,"

					"%d,"

					"%0.3f, %0.3f,"
					"%0.3f,"

					"%0.3f, %0.3f, %0.3f,"
					"%0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"
					"%0.3f, %0.3f,"

					"%0.3f"
					//	"%0.3f, %0.3f"
					//	"%d,%d"
					"\r",

/////////

			//	ahrs.q0[0], ahrs.q0[1], ahrs.q0[2], state.pressure,

			//	 imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],
			//	 state.filtYawRate,imu.gyr_dps[2], ahrs.gyro_offset[2],

//			imu.acc_g[0], imu.acc_g[1], imu.acc_g[2],
//			state.roomInertial[0], state.roomInertial[1], state.roomInertial[2],
//			imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],

			/*
			 * 3
			 */
			state.UWB_XY[0], state.UWB_XY[1],
			state.POS[2],			//Position Error

			//	M1_c, M2_c, M3_c, M4_c,

			/*
			 * 2*(2,3)
			 */
			state.uwbKpV[0],
			state.UWB_VXY[0], //X velocity Setpoint and Measurement
			state.uwbKp[0], state.uwbKi[0],
			state.uwbKd[0], //X velocity PID

			state.uwbKpV[1],
			state.UWB_VXY[1], //Y velocity Setpoint and Measurement
			state.uwbKp[1], state.uwbKi[1],
			state.uwbKd[1], //Y velocity PID

			/*
			 * 2,2,2,3
			 */
			rE[0],
			ahrs.phi_deg, //Roll SP and Measurement
			state.kp_angle[0],
			state.ki_angle[0], //ROLL PI

			state.angleR_angle[0],
			-imu.gyr_dps[0], //GYRO ROLL SETPOINT AND MEASUREMENT
			state.kp_gyro[0], state.ki_gyro[0],
			state.kd_gyro[0], //GYRO ROLL PID

			/*
			 * 2,2,2,3
			 */
			rE[1],
			ahrs.theta_deg, //PITCH SP and Measurement
			state.kp_angle[1],
			state.ki_angle[1], //PITCH PI

			state.angleR_angle[1],
			imu.gyr_dps[1], //GYRO PITCH SETPOINT AND MEASUREMENT
			state.kp_gyro[1], state.ki_gyro[1],
			state.kd_gyro[1], //GYRO PITCH PID

			//////////////
			/*
			 * 2,2,2,3
			 */
			psi_init,
			ahrs.psi_rad, //Yaw SP and Measurement
			state.YawKp,
			state.YawKi, //Yaw PI

			state.YawNewAngle,
			imu.gyr_dps[2], //YAW GYRO SP AND MEASUREMENT
			state.kp_gyro[2], state.ki_gyro[2],
			state.kd_gyro[2], //GYRO YAW PID

			/*
			 * 2,2,3
			 */
			state.ZPiD[0],
			state.UWB_VXY[2], //Z velocity SP and Measurement
			state.ZPiD[1],
			state.ZAccSP, //Z acceleration SP and Measurement

			state.ZAccelPID[0], state.ZAccelPID[1],
			state.ZAccelPID[2], //Z ACCEL PID

			/*
			 * 3,3
			 */
			state.E2_NoFiltNoAngle, state.Kt,
			state.ZPiD[2], //PID/Kt,  Kt,  ThrottleCorrection
			state.Zdotdot, state.C,
			ahrs.R_Matrix_Pos[2][2], //Z accel 'correction' , Total Accel, tilt angle

			/*
			 * 2,2
			 */
			state.x_pid,
			EKF_X.measured_acceleration, //PID sum for X acceleration, True X acceleration
			state.y_pid,
			EKF_Y.measured_acceleration, //PID sum for Y acceleration, True Y acceleration

			state.UWB_setpoint[2],
			//		state.uwbAngle[0], state.uwbAngle[1], state.Zpid,

			//		throttle, throttleUWB_setP, state.UWB_setpoint[2],

//			EKF_X.measured_AGL, EKF_Y.measured_AGL, EKF_Z.measured_AGL,

			//	thisMag.fBcFast[0], thisMag.fBcFast[1], thisMag.fBcFast[2],
			//	EKF_X.x_hat_data[1], EKF_Y.x_hat_data[1] ,EKF_Z.x_hat_data[1],
//			state.positionUWB[0],state.positionUWB[1],state.positionUWB[2],
//			state.positionUWB[3],state.positionUWB[4],state.positionUWB[5],

			//		state.UWB_XY[0], state.UWB_XY[1], state.UWB_XY[2],

			//		state.ZPiD[0], state.ZPiD[1], state.ZPiD[2],

			//		GyroPID[0], GyroPID[1], GyroPID[2],

			//	state.kp_angle[0], state.ki_angle[0], state.Kpcontrol,
			//	state.kp_gyro[0], state.ki_gyro[0], state.kd_gyro[0],
			//		imu.acc_g[0], imu.acc_g[1], imu.acc_g[2],
			//		ahrs.gyro_offset[0], ahrs.gyro_offset[1], ahrs.gyro_offset[2],

			//		EKF_X.measured_acceleration,EKF_Y.measured_acceleration,EKF_Z.measured_acceleration,
			//		Xdes, Ydes, Zacc,

			//		state.ZAccelPID[0], state.ZAccelPID[1], state.ZAccelPID[2],
			//		state.uwb_ekf_x[2], state.uwb_ekf_y[2], state.uwb_ekf_z[2],
			//		imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],

			//		state.KpUWB_Z , state.uwbKP ,

			armed_status,
//			r[0], r[1], LM.kp, M1_c, M2_c,
//			M3_c, M4_c, throttle,
//			state.UWB_XY[0], state.UWB_XY[1], state.time,
//			state.UWB_VXY[0], state.UWB_VXY[1],
//			state.kp_angle[0],	state.kp_angle[1],
//			state.velR[0], state.velR[1],
//			state.gyroPID[0],state.gyroPID[1], state.gyroPID[2],
//			//	state.uwbAngle[0], state.uwbAngle[1],
//			state.velR[0], state.velR[1], uwbLock, 0
			uwb_en, state.ZVelP, state.ZVelI, state.Pos_error,
			state.positionUWB[0], state.positionUWB[1], state.positionUWB[2],
			state.positionUWB[3], state.positionUWB[4],

			state.RSSI_UWB[0], state.RSSI_UWB[1], state.RSSI_UWB[2],
			state.RSSI_UWB[3], state.RSSI_UWB[4],

			state.pressure);
//	sprintf(txBuf_FLASH,  "%0.3f, %0.3f, %0.3f,"
//
//				"%0.3f, %0.3f, %0.3f,"
//				"%0.3f, %0.3f, %0.3f,"
//				"%0.3f, %0.3f, %0.3f,"
//
//				"%0.3f, %0.3f, %0.3f,"
//				"%0.3f, %0.3f, %0.3f,"
//				"%d, %0.3f\r\n",
//
//				thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2],
//
//				thisMagCal.finvW[0][0],thisMagCal.finvW[0][1],thisMagCal.finvW[0][2],
//				thisMagCal.finvW[1][0],thisMagCal.finvW[1][1],thisMagCal.finvW[1][2],
//				thisMagCal.finvW[2][0],thisMagCal.finvW[2][1],thisMagCal.finvW[2][2],
//
//				thisMagCal.fB, thisMagCal.fFourBsq, thisMagCal.fFitErrorpc,
//
//				thisMag.fBpFast[0],thisMag.fBpFast[1], thisMag.fBpFast[2],
//
//				thisMagCal.iValidMagCal, state.euler[2]);

#endif

//	sprintf(txBuf_FLASH, "%d, %d, %d, %d, %0.3f,%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f,%0.3f, %0.3f\r\n",
//			M1_c, M2_c, M3_c, M4_c, angleki_store, imu.acc_g[0], imu.acc_g[1],imu.acc_g[2], rollPID_l,pitchPID_l,LM.kp,ahrs.theta_deg, ahrs.phi_deg  );

//	sprintf(txBuf_FLASH, "%0.3f, %0.3f", state.UWB_XY[0], state.UWB_XY[1]);
#if defined(printQuat)
	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f, %0.3f\r\n", ahrs.q0[0],ahrs.q0[1],  ahrs.q0[2], ahrs.q0[3]);
#endif

#if defined(printEuler)
	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f, %0.3f, %d\r\n", state.euler[0],state.euler[1],  state.euler[2], thisMagCal.fB, thisMagCal.iValidMagCal);
#endif

#if defined(printMagVal)
	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f,%0.3f\r\n", mag.mag[0],mag.mag[1],  mag.mag[2], mag.magfield);
#endif

#if defined(printMagCalibrated)
	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f,%0.3f\r\n", thisMag.fBcFast[0],thisMag.fBcFast[1], thisMag.fBcFast[2], mag.magfield);
#endif

#if defined(printLinAccel)

	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f, %0.3f, %d\r\n", ahrs.linAccel[0],ahrs.linAccel[1],  ahrs.linAccel[2], state.euler[2], time1);

	#endif
#if defined(printRoomInertial)

//	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f, %0.3f, %d\r\n", state.roomInertial[0],state.roomInertial[1],  state.roomInertial[2], state.euler[2], time1);
	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f, %0.3f, %d\r\n", EKF_X.measured_acceleration, EKF_Y.measured_acceleration, EKF_Z.measured_acceleration, state.euler[2], time1);

	#endif

#if defined(printXYuwbPID)
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f\r\n", state.x_pid, state.y_pid, ahrs.theta_deg, ahrs.phi_deg);
#endif

#if defined(printImuAccel)



//	sprintf(txBuf_FLASH, "%0.3f\t%0.3f\t %0.3f\t %0.3f\t %d\r\n", imu.acc_g[0], imu.acc_g[1],imu.acc_g[2], state.euler[2], time1);
	sprintf(txBuf_FLASH, "%0.3f,%0.3f, %0.3f, %0.3f, %d\r\n", 	thisAccel.fGp[0],	thisAccel.fGp[1],	thisAccel.fGp[2], state.euler[2], time1);


	#endif

#if defined(printGyro)

	float gyroCal[3];
	for (int i = 0; i < 3; i++) {
		gyroCal[i]= imu.gyr_dps[i] - ahrs.gyro_offset[i] ;
	}
	sprintf(txBuf_FLASH, "%0.3f, %0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d\r\n", imu.gyr_dps[0],imu.gyr_dps[1],  imu.gyr_dps[2], gyroCal[0], gyroCal[1], gyroCal[2],time1);
#endif

#if defined(printUWB)

	sprintf(txBuf_FLASH, "%0.3f, %0.3f, %0.3f,%0.3f, %0.3f, %0.3f,"
			" %0.3f, %0.3f,%0.3f, "
			" %0.3f, %0.3f,%0.3f  \r\n",
			state.positionUWB[0], state.positionUWB[1], state.positionUWB[2],
			state.positionUWB[3], state.positionUWB[4], state.positionUWB[5],
			EKF_X.measured_AGL, EKF_Y.measured_AGL,EKF_Z.measured_AGL,
			state.uwb_ekf_x[0], state.uwb_ekf_y[0], state.uwb_ekf_z[0]);


#endif

#if defined(printMagCal)
	sprintf(txBuf_FLASH,  "%0.3f, %0.3f, %0.3f,"

			"%0.3f, %0.3f, %0.3f,"
			"%0.3f, %0.3f, %0.3f,"
			"%0.3f, %0.3f, %0.3f,"

			"%0.3f, %0.3f, %0.3f,"
			"%d, %0.3f\r\n",

			thisMagCal.fV[0],thisMagCal.fV[1],thisMagCal.fV[2],

			thisMagCal.finvW[0][0],thisMagCal.finvW[0][1],thisMagCal.finvW[0][2],
			thisMagCal.finvW[1][0],thisMagCal.finvW[1][1],thisMagCal.finvW[1][2],
			thisMagCal.finvW[2][0],thisMagCal.finvW[2][1],thisMagCal.finvW[2][2],

			thisMagCal.fB, thisMagCal.fFourBsq, thisMagCal.fFitErrorpc,

			thisMagCal.iValidMagCal, state.euler[2]);

#endif

#ifdef CRSFvals
	sprintf(txBuf_FLASH, "%d, %d, %d\r\n", CrsfChannels[0], CrsfChannels[1], CrsfChannels[2]);
#endif

#ifdef PrintAccControl
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f, %0.3f,%0.3f, %0.3f, %0.3f\r\n", Xacc, Yacc, Zacc, Xdes, Ydes, state.UWB_XY[0], state.UWB_XY[1], state.UWB_XY[2]);


#endif

#ifdef PrintMeasuredAGL
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f\r\n", EKF_X.measured_AGL, EKF_Y.measured_AGL, EKF_Z.measured_AGL, state.UWB_dtUpdate);
#endif

#ifdef PrintEKFaccel
	float yawError = (180/M_PI)*(ahrs.psi_rad - psi_init);
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f\r\n", EKF_X.measured_acceleration, EKF_Y.measured_acceleration, EKF_Z.measured_acceleration, yawError);
#endif

#ifdef PrintVelocity
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f\r\n", state.UWB_VXY[0], state.UWB_VXY[1], state.UWB_VXY[2], state.UWB_dtUpdate);
#endif

#ifdef PrintUWB_XYZ
	sprintf(txBuf_FLASH, " %0.3f, %0.3f, %0.3f, %0.3f\r\n", state.UWB_XY[0], state.UWB_XY[1], state.UWB_XY[2], state.UWB_dtUpdate);

#endif

#if defined(LittleFS_Flash)

	if ((throttle >= (148))) {
//	int32_t ret = lfs_file_open(&littlefs, &file, filename, LFS_O_RDWR | LFS_O_CREAT);
//	lfs_file_rewind(&littlefs, &file);
		lfs_file_write(&littlefs, &file, &txBuf_FLASHA,
				strlen(txBuf_FLASHA) + 1);

		sync_counter++;

//		lfs_file_write(&littlefs, &file, &txBuf_FLASHB, strlen(txBuf_FLASHB) + 1);
//
//		sync_counter++;
//

		if ((sync_counter % 32) == 0) {
//	lfs_file_sync(&littlefs, &file);
			lfs_file_sync(&littlefs, &file);
			sizereturned = lfs_file_size(&littlefs, &file);

		}
//	int ret4 = lfs_file_close(&littlefs, &file);

		tick_debug = HAL_GetTick() - initTime;
	}
#endif

#if defined(LogFlash)
	if (CSP_QSPI_WriteMemory(txBuf_FLASH, finalIndex + addrFLASH,
			strlen(txBuf_FLASH)) != HAL_OK)
		Error_Handler();

	addrFLASH += sizeFLASH;
#endif

#ifndef LittleFS_Flash
	test_run_info((unsigned char*)txBuf_FLASH);

#endif
	//test_run_info((unsigned char*) posChar); //writes UWB data to USB

	//uint32_t timer = (uS_Time());
#endif
}

float dt_us = 0;

uint32_t current_tick = 0;
static inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void) {
	return ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
			== (SysTick_CTRL_COUNTFLAG_Msk));
}

uint32_t prevUs;

uint32_t getCurrentMicros(void) {
	/* Ensure COUNTFLAG is reset by reading SysTick control and status register */
	LL_SYSTICK_IsActiveCounterFlag();
	uint32_t m = uwTick;
	const uint32_t tms = SysTick->LOAD + 1;
	__IO uint32_t u = tms - SysTick->VAL;
	if (LL_SYSTICK_IsActiveCounterFlag()) {
		m = uwTick;
		u = tms - SysTick->VAL;
	}
	return (m * 1000 + (u * 1000) / tms);

}

uint32_t GetMicros() {
	uint32_t ms;
	uint32_t st;
	uint32_t range;

	do {
		ms = uwTick;
		st = SysTick->VAL;
		asm volatile("nop");
		asm volatile("nop");
	} while (ms != uwTick);

	range = (SysTick->LOAD + 1);
	return (ms * 1000) + ((range - st) / (range / 1000)); // will hardfault if range = 1 by causing divide by 0 error
}

uint32_t dt_UWB;

uint32_t currentT;
uint32_t sample_time = 0;
extern float fastDeltaT;
uint32_t timetot;

uint32_t Timer = 0;
uint32_t deltaTus;
#if defined(Angle_Mode)
bool AngleMode_bl = true;
#else
bool AngleMode_bl = false;
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == SPI1_INT_Pin) {

		// ISR code

		if (imu.init_completed == true) {
			if (Na < sample_tot) {
				ICM42688_ReadData(&imu);

				for (int i = 0; i < 3; i++) {
					if (isinf(imu.acc_g[i])) {
						imu_error = true;
					}
				}
				if (imu_error == false) {
					a0[0] += imu.acc_g[0];
					a0[1] += imu.acc_g[1];
					a0[2] += imu.acc_g[2];
					Na++;
				}
			} else if (Na == sample_tot) {
				a0[0] /= sample_tot;
				a0[1] /= sample_tot;
				a0[2] /= sample_tot;
				Na++;
			} else {

				// ICM42688_ReadDMA(&imu, &BLOCK, txBufimu, rxBufimu);

				//Check aren't doing a filter calculation, and at least 1ms passed from last reading
				if ( //(ahrs.calculating == false)
					 //	((GetMicros() - current_tick) > 250)	&&		  (ahrs.Num1 < 49)){// && ((GetMicros() - current_tick) < 1250)) {//&&
				(ahrs.Num1 < 49)) {	// && ((GetMicros() - current_tick) < 1250)) {//&&

					uint32_t T1 = getCurrentMicros();

					//			if (	 ((getCurrentMicros() - current_tick) > 250)) {

					//	current_tick = GetMicros();

					//			if (ahrs.calculating == false) {
					ICM42688_ReadData(&imu);

					//	time1 = GetMicros() - dt;
					//		dt = GetMicros();
					time1 = HAL_GetTick() - dt;
					dt = HAL_GetTick();

					for (int i = 0; i < 3; i++) {

						//without the gyro_offset the yaw angle is unstable

						if (i == 2) {
							ahrs.gyro_data[i][ahrs.Num1] = ((imu.gyr_dps[i]
									- ahrs.gyro_offset[i])) * (0.001);

						} else {
							ahrs.gyro_data[i][ahrs.Num1] = ((imu.gyr_dps[i]
									- ahrs.gyro_offset[i])) * (0.001);
						}
					}

					for (int i = 0; i < 3; i++) {
						//state.roomInertial[i] = rotM[0][i]*imu.acc_g[0] +  rotM[1][i]*imu.acc_g[1] +  rotM[2][i]*imu.acc_g[2];

						state.roomInertial[i] = 9.81
								* (state.uwbRotM[i][0] * imu.acc_g[0]
										+ state.uwbRotM[i][1] * imu.acc_g[1]
										+ state.uwbRotM[i][2] * imu.acc_g[2]);
						state.roomInertialAvg[i] = state.uwbRotM[i][0]
								* thisAccel.fGp[0]
								+ state.uwbRotM[i][1] * thisAccel.fGp[1]
								+ state.uwbRotM[i][2] * thisAccel.fGp[2];

					}

					StateUpdatePrediction();

					if (ahrs.Num1 == 0) {
						timetot = 0;
					}
					timetot += time1;
					ahrs.Num1++;

					if (AngleMode_bl == true) {
						PID_Execute();
					} else {
						PID_Execute_Acro();
					}

					deltaTus = getCurrentMicros() - T1;

				}
			}

		}

	} else if (GPIO_Pin == I2C2_DRDY_Pin) {
		LIS3MDL_ReadMag(&mag, &hi2c2);

	}

}

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[64];
uint8_t RxData[64];
int indx = 0;

int newData;
extern uint32_t tick1, tick2, rtos_on;
int CANcounter = 0;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		/* Retreive Rx messages from RX FIFO0 */
		if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData)
				!= HAL_OK) {
			/* Reception Error */
			Error_Handler();
		}

		if (RxHeader.Identifier == 0x500) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[0] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[0] = f_ptr[0];
			state.RSSI_UWB[0] = f_ptr[1];

			newData = 1;
			CANcounter++;

		} else if (RxHeader.Identifier == 0x501) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[1] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[1] = f_ptr[0];
			state.RSSI_UWB[1] = f_ptr[1];

			newData = 1;
			CANcounter++;

		} else if (RxHeader.Identifier == 0x502) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[2] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[2] = f_ptr[0];
			state.RSSI_UWB[2] = f_ptr[1];

			newData = 1;
			CANcounter++;

		} else if (RxHeader.Identifier == 0x503) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[3] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[3] = f_ptr[0];
			state.RSSI_UWB[3] = f_ptr[1];

			newData = 1;
			CANcounter++;

		} else if (RxHeader.Identifier == 0x504) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[4] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[4] = f_ptr[0];
			state.RSSI_UWB[4] = f_ptr[1];

			newData = 1;
			CANcounter++;

		} else if (RxHeader.Identifier == 0x505) {

//			double d_ptr = *((double*) RxData);
//			state.positionUWB[5] = (float) d_ptr;

			float *f_ptr = (float*) RxData;
			state.positionUWB[5] = f_ptr[0];
			state.RSSI_UWB[5] = f_ptr[1];

			newData = 1;
			CANcounter++;

		}

		if (CANcounter >= 1) {

			PositionCompUWB();

			EKFUpdate();
			newData = 0;
			CANcounter = 0;

			if (rtos_on == 1) {
				tick2 = osKernelSysTick() - tick1;
				tick1 = osKernelSysTick();
				state.UWB_dtUpdate = (float) tick2;

#ifdef PrintMeasuredAGL
	//		FLASH_LOG();
#endif

			}
		}

		if (HAL_FDCAN_ActivateNotification(hfdcan,
		FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
			/* Notification Error */
			Error_Handler();
		}
#if defined(board2)
    sprintf ((char *)TxData, "FDCAN2TX %d", indx++);

  	  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData)!= HAL_OK)
  	  {
  		  Error_Handler();
  	  }
#endif

	}
}

uint8_t UART_Buffer[80];

#define RxBuf_SIZE   256
#define MainBuf_SIZE 256

uint8_t RxBuf_uart[RxBuf_SIZE];
uint8_t MainBuf[MainBuf_SIZE];

uint16_t oldPos = 0;
uint16_t newPos = 0;

int isOK = 0;

int var = 0;
int end = 0;

extern DMA_HandleTypeDef hdma_usart1_rx;
void UART_FREERTOS() {
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) RxBuf_uart, RxBuf_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

}

#if defined(UseMatrixW)
arm_matrix_instance_f32 A;

float32_t A_uwb[6][3] = { { 1.0000,   -4.1792,   -1.2264},
	    {1.0000,   -1.0160,   -3.6849},
	    {1.0000,   -0.1841,    3.9182},
	    {1.0000,    5.0999,   -1.6664},
	    {1.0000,    4.3429,    1.3272},
	    {1.0000,   -4.0635,    1.3323}}; //A matrix from MATLAB

float anchor[6][2] =   { { 2.0896,    0.6132},
		{    0.5080,    1.8424},
		{    0.0920 ,  -1.9591},
		{   -2.5499 ,   0.8332},
		{  -2.1715  , -0.6636},
		{  2.0317 ,  -0.6661}};  //2D anchor coordinates from MATLAB



 uint16_t rows = 6; // Number of rows in A
uint16_t cols = 4; // Number of columns in A

 uint16_t m = rows; // Number of rows in A
uint16_t n = cols; // Number of columns in A


// Matrix Definitions
float32_t AT[cols * rows]; // Transpose of A
float32_t ATC[cols * rows]; // Transpose(A) * C
float32_t ATCA[cols * cols]; // Transpose(A) * C * A
float32_t ATCA_inv[cols * cols]; // Inverse of (Transpose(A) * C * A)
float32_t ATCb[cols]; // Transpose(A) * C * b

arm_matrix_instance_f32 matA = {m, n, (float32_t*)A};
arm_matrix_instance_f32 matAT = {n, m, AT};
arm_matrix_instance_f32 matC = {m, m, (float32_t*)C};
arm_matrix_instance_f32 matATC = {n, m, ATC};
arm_matrix_instance_f32 matATCA = {n, n, ATCA};
arm_matrix_instance_f32 matATCA_inv = {n, n, ATCA_inv};
arm_matrix_instance_f32 matATCA_invATC = {n, 1, ATCb};
//arm_matrix_instance_f32 positionresult = {n, 1, Xpdw};

#else

#endif
/*------For PsuedoInverse of A------*/

/*
 * A B C D E F
 */

//#define ABCDEF
#ifdef ABCDEF
float pseudodataA[3][6] =    {{ 0.2008 ,   0.1566  ,  0.2184  ,  0.1102  ,  0.1348  ,  0.1792},
	{  -0.0654,   -0.0057  , -0.0729   , 0.0854 ,   0.0606,   -0.0020},
		{        -0.0166 , -0.0791,    0.0636 ,  -0.0496  ,  0.0137  ,  0.0680}};//Psuedoinv of A matrix

#define anchorlen 6
arm_matrix_instance_f32 pinvA = {3, 6, (float32_t*)pseudodataA};
float b[6];
arm_matrix_instance_f32 matb = {6, 1, (float32_t*)b};
float anchorsquare[6] = {   4.5104 ,   7.5472 ,   2.5824 ,   5.1565 ,   5.3337 ,   5.0990};// B = S(i)^2 - sum(anchor.^2) ----> anchorsquare = sum(anchor.^2)
float dataUWB[6];

#else

#endif

/*----------------------------------*/
//void Kalman_X(float32_t pos, float32_t accel);
//void Kalman_Y(float32_t pos, float32_t accel);

uint32_t uwbT0, uwbT1, uwb_dt;

float uwbalpha = 0.85;

/*
 *
 * MOVED UWB TO TASKS.C
 *
 *
 */
float ts;

extern int elrs_error;
char chPC[250];

char txPC[100];

extern CrsfLinkStatistics_t LinkStatistics;
extern char txStats[200];

float AccelB;
float GyroB;
uint16_t my_motor_value[4] = { 100, 200, 400, 1000 };

#if defined(clearMem)
bool eraseFlash= 1;
#else
bool eraseFlash = 0;
#endif

#if defined(transmitter)
bool isMaster = 1;
#else
bool isMaster = 0;
#endif

float gyro_d_beta;

float bounds[2];
float uwb_M;
float uwb_C;
float setP_bounds[2];

float offset_B;

extern uint32_t CRSFtime;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	Crc_init(0xD5);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	imu.init_completed = false;
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C2_Init();
	MX_QUADSPI_Init();
	MX_SPI1_Init();
	MX_SPI2_Init();
	MX_SPI3_Init();
	MX_TIM1_Init();
	MX_UART4_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_ADC3_Init();
	MX_TIM5_Init();
	MX_FDCAN1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM14_Init();
	/* USER CODE BEGIN 2 */

	memset(&state, 0, sizeof(state));
	memset(&imu, 0, sizeof(imu));
	memset(&LM, 0, sizeof(LM));

	state.uwbinit = false;

	/*--------- FLASH --------*/
#if defined(LogFlash)




     	if (CSP_QUADSPI_Init() != HAL_OK)
     		Error_Handler();

     //	if (CSP_QSPI_EnableMemoryMappedMode() != HAL_OK) {

     	//	while (1)
     			; //breakpoint - error detected
     //	}


     	if (!eraseFlash) {

     		while (1) {

     			int returnflag = 0;

     			CSP_QSPI_Read(SECTOR, addrBegin, sizeof(SECTOR));

     			for (int i = 0; i <= (sectorsize - sizeof(endMarker)); i++) {
     				if (memcmp(SECTOR + i, endMarker, sizeof(endMarker)) == 0) {
     					finalIndex = addrBegin + i+10;
     					returnflag = 1;
     					break;
     				}
     			}

     			addrBegin += sectorsize;

     			if (addrBegin >= MEMORY_FLASH_SIZE) {
     				osDelay(1);
     				break;
     			}

     			if (returnflag == 1) {
     				break;
     			}
     		}
     	} else {

     		//Erases the content of the chip
     		if (CSP_QSPI_Erase_Chip() != HAL_OK)
     			Error_Handler();

     	}

     	CSP_QSPI_Read(SECTOR, 0, sizeof(SECTOR));

     #endif

	state.calAxis = 0;
	/*--------- LittleFS --------*/
#if defined(LittleFS_Flash)

	char txBuf_LittleFS_init[256];
	memset(txBuf_LittleFS_init, 0, 256);

	snprintf(txBuf_LittleFS_init, sizeof(txBuf_LittleFS_init),
			//	"%0.3f, %0.3f, %0.3f, %0.3f,"
			"%0.3f, %0.3f, %0.3f, "
			//			"%0.3f, %0.3f, %0.3f, "
			//			"%0.3f, %0.3f, %0.3f,"
			//			" %0.3f, %0.3f, %0.3f,"
			//	"%d,%d,%d,%d,%d,"
					"%0.3f, %0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"

			//"%0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"
			//	"%d,%d,%d,%d,"

					"%0.3f, %0.3f, %0.3f,"
					"%0.3f, %0.3f, %0.3f,"
					"%d, %d, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"

			//			"%0.3f, %0.3f, %0.3f,"
			//			"%0.3f, %0.3f, %0.3f,"

			//		"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f, %0.3f,"

					"%0.3f, %0.3f,"

					//	"%0.3f, %0.3f, %0.3f,"

					"%d,"

					"%d"
					//	"%0.3f, %0.3f"
					//	"%d,%d"
					"\r",
			//	ahrs.q0[0], ahrs.q0[1], ahrs.q0[2], state.pressure,
			ahrs.phi_deg, ahrs.theta_deg, ahrs.psi_deg,
			//	 imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],
			//	 state.filtYawRate,imu.gyr_dps[2], ahrs.gyro_offset[2],

			//			imu.acc_g[0], imu.acc_g[1], imu.acc_g[2],
			//			state.roomInertial[0], state.roomInertial[1], state.roomInertial[2],
			//			imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],

			state.UWB_XY[0], state.UWB_XY[1], state.UWB_XY[2], state.UWB_VXY[0],
			state.UWB_VXY[1], state.UWB_VXY[2],

			rE[0], rE[1], e[2],

			//	M1_c, M2_c, M3_c, M4_c,

			state.uwbKp[1], state.uwbKi[1], state.uwbKd[1],

			//		state.uwbAngle[0], state.uwbAngle[1], state.Zpid,
			state.x_pid, state.y_pid, state.Zpid,

			throttle, throttleUWB_setP, state.UWB_setpoint[2],

			//			EKF_X.measured_AGL, EKF_Y.measured_AGL, EKF_Z.measured_AGL,

			//	thisMag.fBcFast[0], thisMag.fBcFast[1], thisMag.fBcFast[2],
			EKF_X.x_hat_data[1], EKF_Y.x_hat_data[1], EKF_Z.x_hat_data[1],
			//			state.positionUWB[0],state.positionUWB[1],state.positionUWB[2],
			//			state.positionUWB[3],state.positionUWB[4],state.positionUWB[5],

			//		state.UWB_XY[0], state.UWB_XY[1], state.UWB_XY[2],

			state.ZPiD[0], state.ZPiD[1], state.ZPiD[2],

			GyroPID[0], GyroPID[1], GyroPID[2],

			//		imu.acc_g[0], imu.acc_g[1], imu.acc_g[2],
			//		ahrs.gyro_offset[0], ahrs.gyro_offset[1], ahrs.gyro_offset[2],

			EKF_X.measured_acceleration, EKF_Y.measured_acceleration,
			EKF_Z.measured_acceleration,
			//		Xdes, Ydes, Zacc,

			state.ZAccelPID[0], state.ZAccelPID[1], state.ZAccelPID[2],
			//		state.uwb_ekf_x[2], state.uwb_ekf_y[2], state.uwb_ekf_z[2],
			//		imu.gyr_dps[0], imu.gyr_dps[1],imu.gyr_dps[2],

			state.KpUWB_Z, state.uwbKP,

			armed_status,
			//			r[0], r[1], LM.kp, M1_c, M2_c,
			//			M3_c, M4_c, throttle,
			//			state.UWB_XY[0], state.UWB_XY[1], state.time,
			//			state.UWB_VXY[0], state.UWB_VXY[1],
			//			state.kp_angle[0],	state.kp_angle[1],
			//			state.velR[0], state.velR[1],
			//			state.gyroPID[0],state.gyroPID[1], state.gyroPID[2],
			//			//	state.uwbAngle[0], state.uwbAngle[1],
			//			state.velR[0], state.velR[1], uwbLock, 0
			uwb_en);

	uint32_t initlength = strlen(txBuf_LittleFS_init);
	uint32_t beginTimer = HAL_GetTick();

	int ret = w25qxx_littlefs_init();

	/*
	 * writes an init line
	 */

	lfs_file_open(&littlefs, &file, filename,
			LFS_O_RDWR | LFS_O_CREAT | LFS_O_APPEND);
	lfs_file_rewind(&littlefs, &file);
	lfs_file_write(&littlefs, &file, &txBuf_LittleFS_init,
			strlen(txBuf_LittleFS_init) + 1);

	sync_counter++;
//  	lfs_file_close(&littlefs, &file);
	lfs_file_sync(&littlefs, &file);

	HAL_Delay(100);

	/*
	 *
	 */

	char readbuffer[256];

#endif

	/*-----Init State------*/
	memset(&state, 0, sizeof(state));

	/*-------ELRS--------*/
#if defined(useELRS)

	HAL_UART_Receive_DMA(&huart6, (uint8_t*) RxBuffer, LENGTH);

#endif

	/*------DSHOT---------*/
#if defined(DShot)
	dshot_init(DSHOT300);
#endif

	/*-------- GPS ---------*/
#if defined(useGPS)
	GNSS_Init(&GNSS_Handle, &huart4);
	HAL_Delay(1000);
	//	osDelay(1000);
	GNSS_LoadConfig(&GNSS_Handle);
#endif

	/*---------- BAROMETER ------*/
#if defined(useBaro)
	MS5607_Init(&hspi3, SPI3_CS_GPIO_Port, SPI3_CS_Pin); //WORKS

#endif

	/*------ LORA ---------*/
#if defined(useLora)
	Radio.Init(&Callbacks);
	Radio.SetRegulatorMode(USE_LDO); // Can also be set in LDO mode but consume more power
	memset(&Buffer, 0x00, BufferSize);

	//FLRC_SetParams();
	//GFSK_SetParams();
	LORA_SetParams();
	SX1280_Init();
#endif
	/*------- CAN  --------*/
	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			0) != HAL_OK) {
		/* Notification Error */
		Error_Handler();
	}

#if defined(FDtestB1)
				 				 	    TxHeader.Identifier = 0x11;
				 				 	     TxHeader.IdType = FDCAN_STANDARD_ID;
				 				 	     TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				 				 	     TxHeader.DataLength = FDCAN_DLC_BYTES_12;
				 				 	     TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				 				 	     TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
				 				 	     TxHeader.FDFormat = FDCAN_CLASSIC_CAN; //change this to classical can if needed, and change the config in .ioc file
				 				 	     TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				 				 	     TxHeader.MessageMarker = 0;
				 				 	  #endif

#if defined(FDtestB2)
				 				 	  		 TxHeader.Identifier = 0x22;
				 				 	  		   TxHeader.IdType = FDCAN_STANDARD_ID;
				 				 	  		   TxHeader.TxFrameType = FDCAN_DATA_FRAME;
				 				 	  		   TxHeader.DataLength = FDCAN_DLC_BYTES_64;
				 				 	  		   TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				 				 	  		   TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
				 				 	  		   TxHeader.FDFormat = FDCAN_CLASSIC_CAN; //change this to classical can if needed, and change the config in .ioc file
				 				 	  		   TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				 				 	  		   TxHeader.MessageMarker = 0;
				 				 	  #endif

	/*---------UWB MATRICIES----------------*/
#if defined(useUWB)
#if defined(UseMatrixW)
				 				 	  		   // Transpose A
				 				 	  		arm_mat_trans_f32(&matA, &matAT);

				 				 	  		// ATC = Transpose(A) * C
				 				 	  		arm_mat_mult_f32(&matAT, &matC, &matATC);

				 				 	  		// ATCA = Transpose(A) * C * A
				 				 	  		arm_mat_mult_f32(&matATC, &matA, &matATCA);

				 				 	  		// ATCA_inv = inv(Transpose(A) * C * A)
				 				 	  		status = arm_mat_inverse_f32(&matATCA, &matATCA_inv);

				 				 	  		if (status != ARM_MATH_SUCCESS) {
				 				 	  		    // Handle matrix inversion failure
				 				 	  		    return;
				 				 	  		}

				 				 	  		// ATCA_inv_ATC = (ATCA_inv * Transpose(A) * C)
				 				 	  		arm_mat_mult_f32(&matATCA_inv, &matATC, &matATCA_invATC);
#endif

	collectpsi = false;
	state.UWB_setpoint[0] = 0; //set point for drone to go to
	state.UWB_setpoint[1] = 0.9;
	state.UWB_setpoint[2] = 1.64;

	dt_UWB = HAL_GetTick();
#else
				 				 	  UWB_data = false;
				 				 	  	#endif

//	bounds[0] = 0.05;
//	bounds[1] = 1;
//	setP_bounds[0] = 0; //minimum setpoint that will be demanded <= bounds[0]
//	setP_bounds[1] = 5; //maximum angle setpoint that will be demanded >= bounds[1]
//	uwb_M = (setP_bounds[1] - setP_bounds[0]) / (bounds[1] - bounds[0]);
//
//	uwb_C = -bounds[0] * uwb_M;
//
//	//AngleMode_bl = false;

	initTime = HAL_GetTick();
	/*-------ELRS--------*/
#if defined(useELRS)

	HAL_UART_Receive_DMA(&huart6, (uint8_t*) RxBuffer, LENGTH);

#endif
	/*------ ORIENTATION ---------*/
#if defined(useOrientation)

	imu.accel_factor = 0;
	imu.gyro_factor = 0;
	while ((imu.accel_factor == 0) || (imu.gyro_factor == 0)) { //ensure the imu is properly initialised (returning inf values if not)
		icm42688_init(&imu);

	}

	float AccelF0 = 70;
	float GyroF0 = 60;

	AccelB = lowpass_to_beta(AccelF0, 1000);
	GyroB = lowpass_to_beta(GyroF0, 1000);

	// 	   AccelB = 1;
	// 	   GyroB = 1;
	RateB = lowpass_to_beta(50, 1000);

	ahrs.yaw_gyro = 0;
	float gyro_d_hz = 60;
	gyro_d_beta = lowpass_to_beta(50, 1000);

	ICM42688_CollectCalibration(&imu);

	/*
	 * Offsets collected 13/11/24, when axes where changed to correct
	 * yaw angle.
	 */
	float acc_off[3] = { 0.0145301512, 0.0229074918, -0.0135035524 };
	float gyr_off[3] = { -0.870154381, -0.0751261264, -0.369328797 };

	imu.offset_a[0] = acc_off[0]; //0.00114199216		0.0279516112 	-0.0242290534
	imu.offset_a[1] = acc_off[1]; //0.00214750972, 0.0224864744 	-0.0205112789
	imu.offset_a[2] = acc_off[2]; //-0.00103964843, 0.0228862297 	-0.0201592781

	imu.offset_g[0] = gyr_off[0]; //-0.743428469, 	 -0.333792567,	 -0.422920495
	imu.offset_g[1] = gyr_off[1]; //-0.751915932,	 -0.272067368	 -0.401799768
	imu.offset_g[2] = gyr_off[2]; //-0.753778577,	 -0.318884224	 -0.409185112

	ICM42688_WriteCalibration(&imu);
	// 	  ICM42688_CollectCalibration(&imu);

	imu.init_completed = true;

	LIS3MDL_Result_t res = LIS3MDL_ERROR;
	while (res != LIS3MDL_OK) {
		res = LIS3MDL_Init(&mag, &hi2c2, LIS3MDL_Device_0, LIS3MDL_Scale_12G,
				LIS3MDL_MODE_ULTRAHIGH, LIS3MDL_ODR_FAST);

	}

	ahrs.fs = 200.0f;
	sample_time = 1000 / ahrs.fs;
	float rollF0 = 25;
	float pitchF0 = 25;

	ahrs.yaw_alpha = 0.9;

	offset_B = lowpass_to_beta(60, ahrs.fs);

#define FilterEKF
#if defined(FilterEKF)
	RollB = lowpass_to_beta(rollF0, ahrs.fs);
	PitchB = lowpass_to_beta(pitchF0, ahrs.fs);

#else
				 		RollB =1;
				 		PitchB =1;
#endif
	Icm42688_INT1_START();

	//LIS3MDL_ReadMag(&mag, &hi2c2); //this is done to clear the status reg so can use interrupt
	Fusion_Init();

	InitMagCal();

	while (Na < sample_tot + 1) {
	}

	Icm42688_INT1_STOP();

	while (Nm < sample_tot + 1) {

		if (HAL_GPIO_ReadPin(I2C2_DRDY_GPIO_Port, I2C2_DRDY_Pin)
				== GPIO_PIN_SET) {
			if (Nm < sample_tot) {

				LIS3MDL_ReadMag(&mag, &hi2c2);
				fInvertMagCal(&thisMag, &thisMagCal);

//				m0[0] += mag.mag[0];
//				m0[1] += mag.mag[1];
//				m0[2] += mag.mag[2];
				m0[0] += thisMag.fBcFast[0];
				m0[1] += thisMag.fBcFast[1];
				m0[2] += thisMag.fBcFast[2];
				Nm++;

			} else if (Nm == sample_tot) {
				m0[0] /= sample_tot;
				m0[1] /= sample_tot;
				m0[2] /= sample_tot;
				Nm++;
			}

		}
	}

	INIT_3D(&ahrs, &imu, &mag, m0, a0);
	dt = getCurrentMicros();

	current_tick = getCurrentMicros();
	Icm42688_INT1_START();

	uint32_t ahrs_timer = 0;
	// DwtInit();
	uint32_t time_us = 0;
	disconnect_time = uwTick;

#endif

	psi_init = M_PI / 2 - 30 * M_PI / 180;

#if defined(UWB_UART)

	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) &UART_Buffer,
			sizeof(UART_Buffer));
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

#endif

#if defined(UWB_UART)

		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) &UART_Buffer,
				sizeof(UART_Buffer));
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

#endif
	HAL_Delay(1);

	/* USER CODE END 2 */

	/* Call init function for freertos objects (in freertos.c) */
	MX_FREERTOS_Init();

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 2;
	RCC_OscInitStruct.PLL.PLLN = 80;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInitStruct.PLL2.PLL2M = 2;
	PeriphClkInitStruct.PLL2.PLL2N = 12;
	PeriphClkInitStruct.PLL2.PLL2P = 2;
	PeriphClkInitStruct.PLL2.PLL2Q = 2;
	PeriphClkInitStruct.PLL2.PLL2R = 2;
	PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
	PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
	PeriphClkInitStruct.PLL2.PLL2FRACN = 4096;
	PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

float lowpass_to_beta(float f0, float fs) {
	return 1 - exp(-2 * PI * f0 / fs);
	//float RC = 1/(2*PI*f0);
//return (1/fs)/(RC + 1/fs);
}

uint32_t map_angle(uint32_t x, uint32_t in_min, uint32_t in_max,
		uint32_t out_min, uint32_t out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float linear_interp(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min,
		float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void OnTxDone(void) {
	TickCounter = 0;
	AppState = APP_TX;
}

void OnRxDone(void) {
	TickCounter = 0;
	AppState = APP_RX;
}

void OnTxTimeout(void) {
	TickCounter = 0;
	AppState = APP_TX_TIMEOUT;

}

void OnRxTimeout(void) {
	TickCounter = 0;
	AppState = APP_RX_TIMEOUT;
}

void OnRxError(IrqErrorCode_t errorCode) {
	TickCounter = 0;
	AppState = APP_RX_ERROR;
}

void radio_functions(void) {

	if (isMaster == true)		//send  mode
	{
		//pkt.seq=0;

		memset(pkt.seq, 0x00, sizeof(pkt.seq));

		Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE,
				IRQ_RADIO_NONE);
		Radio.SendPayload(Buffer, packet, ( TickTime_t ) { RX_TIMEOUT_TICK_SIZE,
				TX_TIMEOUT_VALUE });
		AppState = APP_LOWPOWER;
		// REMOVING THE WHILE LOOP WHILE WE DONT HAVE RTOS CONFIGURED -- readded now  for RTOS
		while (1) {
			osDelay(10); //Need to wait some time between TxDone's
			//  HAL_Delay(50);

			SX1280ProcessIrqs();

			switch (AppState) {
			case APP_TX:

				AppState = APP_LOWPOWER;
				// pkt.addr=0x01;
//pkt.seq++;

				// sprintf(txBuf_FLASH, "%d", pkt.seq);

				char loraMSG[150];
				sprintf(loraMSG, "%0.3f,%0.3f,"
						"%0.3f,%0.3f,"
						"%0.3f,%0.3f,"
						"%d,%d,%d,%d,"
						" %0.3f,%0.3f\r\n", LM.Angle[0], LM.Angle[1], LM.E[0],
						LM.E[1], LM.R[0], LM.R[1], LM.Motor[0], LM.Motor[1],
						LM.Motor[2], LM.Motor[3], LM.PID[0], LM.PID[1]);

				packet = strlen(loraMSG);
				memset(Buffer, 0x00, sizeof(Buffer));
				memcpy(Buffer, loraMSG, packet);

				Radio.SetDioIrqParams(TxIrqMask, TxIrqMask, IRQ_RADIO_NONE,
						IRQ_RADIO_NONE);
				Radio.SendPayload(Buffer, packet, ( TickTime_t ) {
						RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE });
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
				osDelay(20);
				// HAL_Delay(20);
				break;
			default:
				break;
			}

		}

	} else if (isMaster == false)	    	//receive  mode
	{

		AppState = APP_LOWPOWER;
		Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE,
				IRQ_RADIO_NONE);
		Radio.SetRx(( TickTime_t ) { RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE });
		printf("start\n");
		while (1) {
			osDelay(5); //Need to wait some time between RxDone's
//HAL_Delay(10);

			SX1280ProcessIrqs();
			switch (AppState) {
			case APP_RX:

				memset(Buffer, 0x00, sizeof(Buffer));

				AppState = APP_LOWPOWER;
				Radio.GetPayload(Buffer, &BufferSize, BUFFER_SIZE); //接收的数�?�装载到buffer
				SX1280GetPacketStatus(&packetStatus);
				rssi = SX1280GetRssiInst();
				printf("%d--Rssi:%d\n", Buffer[1], rssi);

				packet = strlen(Buffer);
				char txBuf_Lora[200];
				memset(txBuf_Lora, 0x00, sizeof(txBuf_Lora));
				memcpy(txBuf_Lora, Buffer, packet);

				Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE,
						IRQ_RADIO_NONE);
				Radio.SetRx(( TickTime_t ) { RX_TIMEOUT_TICK_SIZE,
						RX_TIMEOUT_VALUE });
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
				// HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
				//HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);

				int end = 0;

				for (int i = 0; i < strlen(txBuf_Lora); i++) {
					if ((txBuf_Lora[i] == '\r')
							&& (txBuf_Lora[i + 1] == '\n')) {
						end = i;
						break;
					}
				}
				end += 2;

//				  char *txPC = (char *)malloc(end * sizeof(char));
//				    if (txPC == NULL) {
//				        // Handle memory allocation failure
//				        fprintf(stderr, "Memory allocation failed\n");
//				        return 1;
//				    }
//
//				    // Copy the required portion of the string to txPC
//				    strncpy(txPC, txBuf_FLASH, end);
				//  txPC[end] = '\0'; // Null-terminate the string
				//test_run_info((unsigned char *)txPC);

				char txRSSI[50];
				int receivelen = strlen(txBuf_Lora);
				sprintf(txRSSI, "RSSI: %d, Received Data Length %d \r\n",
						(int) rssi, receivelen);

				//	char newline[] = "\r\n";

				//	test_run_info_rxBuf((unsigned char *)txBuf_FLASH, (uint16_t)end);
				//		test_run_info_rxBuf((unsigned char *)txRSSI, strlen(txRSSI));
				//		test_run_info_rxBuf((unsigned char *)newline, strlen(newline));

				//    free(txPC);

				break;
			case APP_RX_TIMEOUT:
			case APP_RX_ERROR:
				//	HAL_Delay(5000);
				AppState = APP_LOWPOWER;

				Radio.SetDioIrqParams(RxIrqMask, RxIrqMask, IRQ_RADIO_NONE,
						IRQ_RADIO_NONE);
				Radio.SetRx(( TickTime_t ) { RX_TIMEOUT_TICK_SIZE,
						RX_TIMEOUT_VALUE });
				break;
			default:
				break;
			}
		}

	}

}

bool stopmotor_check = false;
float velEsum[2];

float velE[2];
bool collect_setpoint = false;
uint32_t uwbtime;

float lpf_uwb[3];
float alpha_uwb;

float Z_sum = 0;

float uwb_Z;

float c;

bool initYaw = false;

bool YawOn = false;
uint32_t YawWaitTime = 0;

float uwb_pid[2];
float uwbKP;
float uwbKD;

float KpUWB_Z = 1.5;
float KiUWB_Z = 0.75;
float KdUWB_Z = 0.3;

float delta_pos;
float delta_vel;

float VelErrorPrev[2];
float XYsum[2];

float trim[2];

void EKF(void) {

	fInvertMagCal(&thisMag, &thisMagCal);

	// update magnetic buffer checking for i) absence of first all-zero magnetometer output and ii) no calibration in progress
	// an all zero magnetometer reading can occur after power-on at rare intervals but it simply won't be used in the buffer
	if (!((globals.loopcounter < 100) && (thisMag.iBpFast[X] == 0)
			&& (thisMag.iBpFast[Y] == 0) && (thisMag.iBpFast[Z] == 0))
			&& !thisMagCal.iCalInProgress) {

		//	thisAccel.iGp = (int16)imu.acc_g*imu.accel_factor;
		//	thisMag.iBpFast = (int16)mag.mag_raw

		// update the magnetometer measurement buffer integer magnetometer data (typically at 25Hz)
		iUpdateMagnetometerBuffer(&thisMagBuffer, &thisAccel, &thisMag,
				globals.loopcounter);
	}

	ahrs.calculating = true;
	ahrs.factor = ahrs.Num1;

	memset(gyro_data, 0, sizeof(gyro_data));//clear gyro_data buffer that is used in EKF calcs
	memcpy(gyro_data, ahrs.gyro_data, sizeof(ahrs.gyro_data)); //copy fast orientatations to new gyro data buffer

	ahrs.Num1 = 0;
	memset(ahrs.gyro_data, 0, sizeof(ahrs.gyro_data)); //clear old buffer to fill during EKF calcs

	PREDICT_3D(&ahrs, &imu);
	UPDATE_3D(&ahrs, &imu, &mag);

	current_tick = getCurrentMicros();

	ahrs.calculating = false;

	int8 initiatemagcal;		// flag to initiate a new magnetic calibration

	float Deg_2Rad = 0.017453;
	float cosphi = cos(-state.euler[0] * Deg_2Rad);
	float sinphi = sin(-state.euler[0] * Deg_2Rad);

	float costheta = cos(state.euler[1] * Deg_2Rad);
	float sintheta = sin(state.euler[1] * Deg_2Rad);

	float yawRad = ahrs.psi_rad - psi_init;
	float cospsi = cos(yawRad);
	float sinpsi = sin(yawRad);

	float rotM[3][3] = { { costheta, sinphi * sintheta, cosphi * sintheta }, {
			0, cosphi, -sinphi }, { -sintheta, sinphi * costheta, cosphi
			* costheta } };

	float yawRot[3][3] = { { cospsi, -sinpsi, 0 }, { sinpsi, cospsi, 0 }, { 0,
			0, 1 } };

	float32_t MAT1[3][3], MAT2[3][3], MAT3[3];

	arm_matrix_instance_f32 RollPitchMAT = { 3, 3, MAT1 };
	arm_matrix_instance_f32 YawMAT = { 3, 3, MAT2 };
	arm_matrix_instance_f32 ResultMAT = { 3, 3, MAT3 };

	arm_mat_init_f32(&RollPitchMAT, 3, 3, (float32_t*) &rotM); //Initialise B matrix
	arm_mat_init_f32(&YawMAT, 3, 3, (float32_t*) &yawRot); //Initialise B matrix
	arm_mat_init_f32(&ResultMAT, 3, 3, (float32_t*) &state.uwbRotM); //Initialise B matrix

	arm_mat_mult_f32(&YawMAT, &RollPitchMAT, &ResultMAT);

	// check no magnetic calibration is in progress
	if (!thisMagCal.iCalInProgress) {
		// do the first 4 element calibration immediately there are a minimum of MINMEASUREMENTS4CAL
		initiatemagcal = (!thisMagCal.iMagCalHasRun
				&& (thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL));

		// otherwise initiate a calibration at intervals depending on the number of measurements available

		initiatemagcal |=
				((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS4CAL)
						&& (thisMagBuffer.iMagBufferCount < MINMEASUREMENTS7CAL)
						&& !(globals.loopcounter % INTERVAL4CAL)
						&& (thisMagCal.iValidMagCal <= 4));

		initiatemagcal |=
				((thisMagBuffer.iMagBufferCount >= MINMEASUREMENTS7CAL)
						&& (thisMagBuffer.iMagBufferCount < MINMEASUREMENTS10CAL)
						&& !(globals.loopcounter % INTERVAL7CAL)
						&& (thisMagCal.iValidMagCal <= 7));

		initiatemagcal |= ((thisMagBuffer.iMagBufferCount
				>= MINMEASUREMENTS10CAL)
				&& !(globals.loopcounter % INTERVAL10CAL)
				&& (thisMagCal.iValidMagCal <= 10));

		// initiate the magnetic calibration if any of the conditions are met
		if (initiatemagcal) {
			// set the flags denoting that a calibration is in progress
			thisMagCal.iCalInProgress = 1;
			thisMagCal.iMagCalHasRun = 1;

			// enable the magnetic calibration task to run
			//mqxglobals.MagCal_Event_Flag = 1;
		} // end of test whether to call calibration functions
	} // end of test that no calibration is already in progress

	globals.loopcounter++;

	if (collectpsi == true) {
		//	psi_init = ahrs.psi_rad; //initialise the orientation (must be initialised in correct position)
		collectpsi = false;
	}

	if (Uwbtoggleon == true) {
		collectpsi = false;
		uwbtime = HAL_GetTick();
		collect_setpoint = true;
		Uwbtoggleon = false;
	}

	state.euler[0] = ahrs.phi_deg;
	state.euler[1] = ahrs.theta_deg;
	state.euler[2] = ahrs.psi_deg;

//Tune kp
//#define TUNE_KP
#if defined(TUNE_KP)
			if (CrsfChannels[9] == 1792) {
				if (CrsfChannels[5] > 1200){
					if (kp <= 0.6){
					kp += 0.00005;
					}
				} else if (CrsfChannels[5] < 800) {
					if (kp >= 0.2){
					kp -= 0.00005;
					}
				}
			}

			LM.kp = kp;
			GyroKP = LM.kp;


#else

	kp = 1; //Tuned value
#endif

//Tune ki
//#define TUNE_KI
#if defined(TUNE_KI)
			if (CrsfChannels[9] == 1792) {
							if (CrsfChannels[5] > 1200){
								if (ki <= 0.5){
								ki += 0.000005;
								}
							} else if (CrsfChannels[5] < 800) {
								if (ki >= 0.00005){
								ki -= 0.000005;
								}
							}
						}
#endif

//Tune kd
//#define TUNE_KD
#if defined(TUNE_KD)
			if (CrsfChannels[9] == 1792) {
							if (CrsfChannels[5] > 1200){
								if (kd <= 3){
								kd += 0.005;
								}
							} else if (CrsfChannels[5] < 800) {
								if (kd > 0.005){
								kd -= 0.005;
								}
							}
						}
#else
	kd = 0; //tuned value
#endif

	if (UWB_data == false) {
		float scale;

		if (AngleMode_bl == true) {

#ifdef velControl

/*
 * Create a P controller from the error of desired velocity and UWB velocity
 *
 * Pass the output as the angle demand for Angle controller
 *
 * Should enforce the drone to remain in the same spot for no input demand
 */




velE[0] = r[0] - state.UWB_VXY[0];
velE[1] = r[1] - state.UWB_VXY[1];

if ((armed_drone == true)&&(throttle > 300)) {
	velEsum[0] = velEsum[0] + velE[0]*((float)sample_time/1000);
	velEsum[1] = velEsum[1] + velE[1]*((float)sample_time/1000);

	} else {
		velEsum[0] = 0;
		velEsum[1] = 0;

	}

velEsum[0] = constrain(velEsum[0], -i_limit, i_limit);
velEsum[1] = constrain(velEsum[1], -i_limit, i_limit);



rE[0] = velE[0] * velKP + velEsum[0] * velKI;
rE[1] = velE[1] * velKP + velEsum[1] * velKI;

state.velR[0] = rE[0];
state.velR[1] = rE[1];

#else

			rE[0] = r[0];
			rE[1] = r[1];

#endif

#ifdef debugvel
int rollpitch_max = 20;
													r[0] = linear_interp(CrsfChannels[0], xmin + 50, xmax - 50, -rollpitch_max, rollpitch_max);
													r[1] = linear_interp(CrsfChannels[1], xmin + 50, xmax - 50, -rollpitch_max, rollpitch_max);

rE[0] = r[0];
rE[1] = r[1];
#endif
//#define noAngle
#ifndef noAngle
			e[0] = rE[0] - ahrs.phi_deg;
			e[1] = rE[1] - ahrs.theta_deg;

			state.x_pid = 0;
			state.y_pid = 0;

			state.ZPiD[0] = 0;
			state.ZPiD[1] = 0;
			state.ZPiD[2] = 0;

			uwb_en = 0;
#else

	e[0] = rE[0] - 0;
	e[1] = rE[1] + 0;
#endif
//	e[2] = r[2] - ahrs.psi_deg;
		}

	} else if ((UWB_data == true) && (setpoint_collected == true)) {

		if ((state.UWB_deltaXY[0] != 0) || (state.UWB_deltaXY[1] != 0)) {

			/*
			 *
			 * vel_cmd = KpPosXY*(setpoint - position)
			 * if (vel_cmd.magnitude > maxSpeedXY){
			 * vel_cmd = vel_cmd.norm * maxSpeedXY
			 * }
			 *
			 * acc_cmd += KpVelXY*(vel_cmd - vel)
			 *
			 */

			/*
			 *
			 * err = setpoint - measurement
			 * psum = Kp * err
			 * isum += 0.5*Ki*dt*(err - last_err)
			 * isum = constrain(isum)
			 *
			 * dsum = -(2*kd*(measurement - last_measuremnt) +
			 * 				(2*tau*dt)*dsum)/(2*tau + dt)
			 *
			 * pidsum = psum + isum + dsum
			 * constrain(pidsum)
			 *
			 * last_err = err
			 * last_measurement = measurement
			 *
			 */

//#define TUNE_KP_X
#if defined(TUNE_KP_X)

			uwbKP = linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 5);
			uwbKd = 0;
			state.uwbKP = uwbKP;


#else

#endif

			float P = 1.5;

			float XYerror[2];

			XYerror[0] = state.UWB_deltaXY[0];
			XYerror[1] = state.UWB_deltaXY[1];

			float XYvelCorrection[2];

			//	P = 0;
#define secnorder_XY 0.1
			float linear_dist = secnorder_XY / (P * P);

			/*
			 */
			for (int i = 0; i <= 1; i++) {

				if (XYerror[i] > linear_dist) {
					XYvelCorrection[i] = safe_sqrt(
							2 * secnorder_XY
									* (XYerror[i] - (linear_dist / 2)));

				} else if (XYerror[i] < -linear_dist) {
					XYvelCorrection[i] = -safe_sqrt(
							2 * secnorder_XY
									* (-XYerror[i] - (linear_dist / 2)));

				} else {
					XYvelCorrection[i] = XYerror[i] * P;
				}

			}

			float VelP = 5;
			float VelI = 0.03;
			float VelD = 4;

//		VelI = linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 0.1);
			state.KiControl = VelI;
//		VelI = 0;
//		VelD = 0;

			float VelError[2];

			state.uwbKpV[0] = XYvelCorrection[0];
			state.uwbKpV[1] = XYvelCorrection[1];

			VelError[0] = (XYvelCorrection[0] - state.UWB_VXY[0]); //- dv[0];
			VelError[1] = (XYvelCorrection[1] - state.UWB_VXY[1]); //- dv[1];

			if ((armed_drone == true) && (throttle >= 250)) {

				XYsum[0] += VelError[0] * ((float) sample_time / 1000);
				XYsum[1] += VelError[1] * ((float) sample_time / 1000);

			} else {
				XYsum[0] = 0;
				XYsum[1] = 0;
			}

#define MaxKiAccelXY 5
			XYsum[0] = constrain(XYsum[0], -MaxKiAccelXY, MaxKiAccelXY);
			XYsum[1] = constrain(XYsum[1], -MaxKiAccelXY, MaxKiAccelXY);

			float XY_alpha = 0.01;

			state.uwbKp[0] = VelError[0] * VelP;
			state.uwbKi[0] = XYsum[0] * VelI;
			state.uwbKd[0] = (1 - XY_alpha) * state.uwbKd[0]
					+ XY_alpha * (VelError[0] - VelErrorPrev[0])
							* (1000 / (float) sample_time) * VelD;
			//	state.uwbKd[0] = state.roomInertial[1]*VelD;

			uwb_pid[0] = state.uwbKp[0] + state.uwbKi[0] + state.uwbKd[0];

			state.uwbKp[1] = VelError[1] * VelP;
			state.uwbKi[1] = XYsum[1] * VelI;
			state.uwbKd[1] = (1 - XY_alpha) * state.uwbKd[1]
					+ XY_alpha * (VelError[1] - VelErrorPrev[1])
							* (1000 / (float) sample_time) * VelD;
//		state.uwbKd[1] = state.roomInertial[0]*VelD;

			uwb_pid[1] = state.uwbKp[1] + state.uwbKi[1] + state.uwbKd[1];

			VelErrorPrev[0] = VelError[0];
			VelErrorPrev[1] = VelError[1];

#define MaxAccel 3

			float AccelMag = sqrt(
					uwb_pid[0] * uwb_pid[0] + uwb_pid[1] * uwb_pid[1]);

			if (AccelMag > MaxAccel) {
				uwb_pid[0] *= (MaxAccel / AccelMag);
				uwb_pid[1] *= (MaxAccel / AccelMag);

			}

			state.x_pid = uwb_pid[0];
			state.y_pid = uwb_pid[1];

			delta_pos = 0;
			delta_vel = 0;

			/*
			 *  throttle += uwb_Z;
			 */

			float temp1[2];
			float Rad2Deg = 180 / M_PI;

#define MaxTiltAngle 15

			/*
			 * PITCH ANGLE = atan(xdotdot/thrust)
			 */
			temp1[1] = constrain(Rad2Deg * atan((uwb_pid[1] / c)),
					-MaxTiltAngle, MaxTiltAngle);

			/*
			 * ROLL ANGLE = atan(xdotdot/thrust)
			 */
			temp1[0] = constrain(Rad2Deg * atan((uwb_pid[0] / c)),
					-MaxTiltAngle, MaxTiltAngle);

#ifndef Zctrl_1ms

			AltitudeHold();

#endif

			float psi = ahrs.psi_rad - psi_init;
			float cpsi = cos(psi);
			float spsi = sin(psi);

			cpsi = 1;
			spsi = 0;

			float M[2][2] = { { cpsi, -spsi }, { spsi, cpsi } };

			float demand[2];
			demand[0] = temp1[0] * M[0][0] + temp1[1] * M[0][1];
			demand[1] = temp1[0] * M[1][0] + temp1[1] * M[1][1];

#if defined(TUNE_KP_Z)
			uwb_pid[0] = r[0];
			uwb_pid[1] = r[1];
#endif

			//rE[0] = trim[0] + demand[0];
			//rE[1] = trim[1] + demand[1];
			rE[0] = demand[0];
			rE[1] = demand[1];

			e[0] = rE[0] - ahrs.phi_deg;
			e[1] = rE[1] - ahrs.theta_deg;

			//	e[0] = r[0] - ahrs.phi_deg;
			//	e[1] = r[1] - ahrs.theta_deg;

			//	 e[2] = 0;

			uwb_en = 1;

			state.uwbAngle[0] = uwb_pid[0];
			state.uwbAngle[1] = uwb_pid[1];

		}

	}

	if ((armed_drone == true) && (throttle > 250)) {
		esum[0] = esum[0] + e[0] * ((float) sample_time / 1000);
		esum[1] = esum[1] + e[1] * ((float) sample_time / 1000);

		angleki_store = esum[1];
	} else {
		esum[0] = 0;
		esum[1] = 0;
		angleki_store = esum[1];
	}

	esum[0] = constrain(esum[0], -i_limit, i_limit);
	esum[1] = constrain(esum[1], -i_limit, i_limit);

	/*
	 int intstat = 50;
	 if ((esum[0]*ki > intstat) || (esum[0]*ki < -intstat)) {
	 kiPID[0] = intstat;
	 kiPID[1] = intstat;
	 } else {
	 kiPID[0] = esum[0]*ki;
	 kiPID[1] = esum[1]*ki;
	 }*/

//rollPID = e[0] * kp + kiPID[0] - kdPID[0];
//pitchPID = e[1] * kp + kiPID[1] - kdPID[1];

//	AngleKP =  linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 0.5);
//	AngleKI = 0;
	AngleKP = 0.2;
	AngleKI = 0.183;

	state.kp_angle[0] = e[0] * AngleKP;
	state.kp_angle[1] = e[1] * AngleKP;

	state.ki_angle[0] = esum[0] * AngleKI;
	state.ki_angle[1] = esum[1] * AngleKI;

	rollPID = state.kp_angle[0] + state.ki_angle[0];	//- kdPID[0];
	pitchPID = state.kp_angle[1] + state.ki_angle[1];	//- kdPID[1];

	float yawError;
	yawError = (psi_init - ahrs.psi_rad);

	/*
	 * Need an integral controller for Yaw. Integral only
	 * winds up after 3s condition from taking init value
	 *
	 */
	if (yawError > M_PI) {
		yawError -= 2 * M_PI;
	} else if (yawError < -M_PI) {
		yawError += 2 * M_PI;
	}

	yawError *= (180 / M_PI);

	if ((armed_drone == true) && (throttle > 350) && (YawOn)) {
		esum[2] = esum[2] + yawError * ((float) sample_time / 1000);

	} else {
		esum[2] = 0;
	}

	esum[2] = constrain(esum[2], -(i_limit + 25), (i_limit + 25));

	float KpYaw = 1;
	float KiYaw = 0.2;
	state.YawKp = KpYaw * yawError;
	state.YawKi = esum[2] * KiYaw;
	yawPID = state.YawKp + state.YawKi;

	float YawRate = 30;
	yawPID *= YawRate;

	int maxPID = 200;

	float newangleR[3];

	newangleR[0] = Kl * rollPID;
	newangleR[1] = Kl * pitchPID;
	newangleR[2] = yawPID;

	newangleR[0] = constrain(newangleR[0], -maxPID, maxPID);
	newangleR[1] = constrain(newangleR[1], -maxPID, maxPID);
	newangleR[2] = constrain(newangleR[2], -maxPID, maxPID);

	float lpf_angle = 1;

	angleR[0] = (1 - lpf_angle) * angleR[0] + lpf_angle * newangleR[0];
	angleR[1] = (1 - lpf_angle) * angleR[1] + lpf_angle * newangleR[1];
	angleR[2] = (1 - lpf_angle) * angleR[2] + lpf_angle * newangleR[2];

	state.angleR_angle[0] = angleR[0];
	state.angleR_angle[1] = angleR[1];
	state.YawNewAngle = angleR[2];

}

float e_acro_prev[3];

float d_term[3];

int throttle_pid;
float bodyGyr[3];
float filtOffset[3];

float esum_Gyro[3];

void PID_Execute(void) {
	RateB = 0.95;

	float e_acro[3];

	filtOffset[0] = (1 - offset_B) * filtOffset[0]
			+ offset_B * ahrs.gyro_offset[0];
	filtOffset[1] = (1 - offset_B) * filtOffset[1]
			+ offset_B * ahrs.gyro_offset[1];
	filtOffset[2] = (1 - offset_B) * filtOffset[2]
			+ offset_B * ahrs.gyro_offset[2];

	filtOffset[0] = 0;
	filtOffset[1] = 0;
	filtOffset[2] = 0;

	bodyGyr[0] = (imu.gyr_dps[0] - filtOffset[0]);
	bodyGyr[1] = (imu.gyr_dps[1] - filtOffset[1]);
	bodyGyr[2] = (imu.gyr_dps[2] - filtOffset[2]);

	state.filtYawRate = filtOffset[2];

#define useBodyGyro

#ifdef useBodyGyro

	e_acro[0] = angleR[0] + (bodyGyr[0]);
	e_acro[1] = angleR[1] - (bodyGyr[1]);

	if (rtos_on == 1) {
		if (((osKernelSysTick() - YawWaitTime) > 1500) || (YawOn)) {

			if (!YawOn) {
				angleR[2] = 0;
				YawOn = true;
				e_acro[2] = r[2] - (bodyGyr[2]);
			} else {

				e_acro[2] = angleR[2] - (bodyGyr[2]);
			}

		} else {

			e_acro[2] = r[2] - (bodyGyr[2]);

		}
	} else {
		e_acro[2] = r[2] - (bodyGyr[2]);
	}

#else


#endif

	if ((armed_drone == true) && (throttle > 350)) {
		esum_Gyro[0] = esum_Gyro[0] + e_acro[0] * (0.001);
		esum_Gyro[1] = esum_Gyro[1] + e_acro[1] * (0.001);
		esum_Gyro[2] = esum_Gyro[2] + e_acro[2] * (0.001);

	} else {
		esum_Gyro[0] = 0;
		esum_Gyro[1] = 0;
		esum_Gyro[2] = 0;

	}

#define GyroIMAX 100
	esum_Gyro[0] = constrain(esum_Gyro[0], -GyroIMAX, GyroIMAX);
	esum_Gyro[1] = constrain(esum_Gyro[1], -GyroIMAX, GyroIMAX);
	esum_Gyro[2] = constrain(esum_Gyro[2], -GyroIMAX, GyroIMAX);

	float pid_acro[3];
	float dt_1 = 1000; //  = 1/dt = 1/1ms = 1000

//	GyroKP =  linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 5);

	float Ku = 1;
	float Tu = 0.15; //angle = 1.95s, gyro = 0.1s
//	GyroKP = 0.6*Ku;
//	GyroKI = 1.2*Ku/Tu;
//	GyroKD = 3*Ku*Tu/40;

	GyroKP = 0.5;
	GyroKI = 0.35;
//	GyroKD = 6.66e-3;
//	GyroKD = 0;
//	GyroKI = 0;

	GyroKP_Yaw = 0.4;
	GyroKD = 0.006;
	state.kp_gyro[0] = GyroKP * e_acro[0];
	state.kp_gyro[1] = GyroKP * e_acro[1];
	state.kp_gyro[2] = GyroKP_Yaw * e_acro[2];

	float GyroKD_yaw = 0.006;

	state.kd_gyro[0] = (1 - gyro_d_beta) * state.kd_gyro[0]
			+ gyro_d_beta * GyroKD * (e_acro[0] - e_acro_prev[0]) * dt_1;
	state.kd_gyro[1] = (1 - gyro_d_beta) * state.kd_gyro[1]
			+ gyro_d_beta * GyroKD * (e_acro[1] - e_acro_prev[1]) * dt_1;
	state.kd_gyro[2] = (1 - gyro_d_beta) * state.kd_gyro[2]
			+ gyro_d_beta * GyroKD_yaw * (e_acro[2] - e_acro_prev[2]) * dt_1;

	GyroKI_Yaw = 0.2;

	state.ki_gyro[0] = GyroKI * esum_Gyro[0];
	state.ki_gyro[1] = GyroKI * esum_Gyro[1];
	state.ki_gyro[2] = GyroKI_Yaw * esum_Gyro[2];

	pid_acro[0] = state.kp_gyro[0] + state.ki_gyro[0] + state.kd_gyro[0];
	pid_acro[1] = state.kp_gyro[1] + state.ki_gyro[1] + state.kd_gyro[1];
	pid_acro[2] = state.kp_gyro[2] + state.ki_gyro[2] + state.kd_gyro[2];

	e_acro_prev[0] = e_acro[0];
	e_acro_prev[1] = e_acro[1];
	e_acro_prev[2] = e_acro[2];

	GyroPID[0] = (1 - RateB) * GyroPID[0] + RateB * pid_acro[0];
	GyroPID[1] = (1 - RateB) * GyroPID[1] + RateB * pid_acro[1];
	GyroPID[2] = (1 - RateB) * GyroPID[2] + RateB * pid_acro[2];

	for (int i = 0; i < 3; i++) {
		GyroPID[i] = constrain(GyroPID[i], -300, 300);
	}

	state.gyroPID[0] = GyroPID[0];
	state.gyroPID[1] = GyroPID[1];
	state.gyroPID[2] = GyroPID[2];

	LM.E[0] = (angleR[0] - imu.gyr_dps[1]);
	LM.E[1] = (angleR[1] + imu.gyr_dps[0]);
	LM.PID[0] = GyroPID[0];
	LM.PID[1] = GyroPID[1];

//float psi = ahrs.psi_rad - psi_init;
//float cpsi = cos(psi);
//float spsi = sin(psi);

//int T_KP = 500;

// throttle_pid = (throttle - state.altitude)*T_KP;

	if ((UWB_data == true) && (setpoint_collected == true)) {
//	throttle = throttleUWB_setP;
	}

	if ((UWB_data == true) && (setpoint_collected == true)) {

		//throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, 0.8 * 2047);

		if (throttle > throttleHover) {
			throttle = throttleHover;
		}

//throttleUWB_setP = throttle;
#ifdef Zctrl_1ms

	AltitudeHold();

#endif

		M1_c = throttleUWB_setP + GyroPID[0] - GyroPID[1] + GyroPID[2]
				+ state.throttleCorrection; //m1, m4 = CW, m2, m3 = CCW
		M2_c = throttleUWB_setP + GyroPID[0] + GyroPID[1] - GyroPID[2]
				+ state.throttleCorrection;
		M3_c = throttleUWB_setP - GyroPID[0] - GyroPID[1] - GyroPID[2]
				+ state.throttleCorrection;
		M4_c = throttleUWB_setP - GyroPID[0] + GyroPID[1] + GyroPID[2]
				+ state.throttleCorrection;

	} else {

		if (throttle > throttleHover) {
			throttle = throttleHover;
		}

		M1_c = throttle + GyroPID[0] - GyroPID[1] + GyroPID[2]; //m1, m4 = CW, m2, m3 = CCW
		M2_c = throttle + GyroPID[0] + GyroPID[1] - GyroPID[2];
		M3_c = throttle - GyroPID[0] - GyroPID[1] - GyroPID[2];
		M4_c = throttle - GyroPID[0] + GyroPID[1] + GyroPID[2];

	}

//	M1_c = 300;
//	M2_c = 400;
//	M3_c = 0;
//	M4_c = 500;

	int zeroval = 48;
	if (throttle < (zeroval + 100)) {
		M1_c = 0;
		M2_c = 0;
		M3_c = 0;
		M4_c = 0;
	} else {

		if (M1_c >= (100 + zeroval)) {
			M1_c -= 100;
		} else {
			M1_c = 0;
		}

		if (M2_c >= (100 + zeroval)) {
			M2_c -= 100;
		} else {
			M2_c = 0;
		}

		if (M3_c >= (100 + zeroval)) {
			M3_c -= 100;
		} else {
			M3_c = 0;
		}

		if (M4_c >= (100 + zeroval)) {
			M4_c -= 100;
		} else {
			M4_c = 0;
		}

	}

	uint16_t motor_command[4] = { M1_c, M2_c, M4_c, M3_c };
	//uint16_t motor_command[4] = {0, 0, M3_c, 0};

	if (rtos_on == 1) {
		if ((osKernelSysTick() - CRSFtime) > 1000) {
			armed_drone = false; //disarms the drone if no new radio data for 1s
			armed_status = QUAD_NO_SIGNAL;
		}
	}

	if (armed_drone == true) {
//					if (stopmotor_check == false){

		dshot_write(motor_command);

//					} else {
//						uint16_t stopmot[4] = { 0, 0, 0, 0 };
//											dshot_write(stopmot);
//					}

		/*		 servo_angle(&M1,	M1_c);
		 servo_angle(&M2,	M2_c);
		 servo_angle(&M3, M3_c);
		 servo_angle(&M4,	M4_c);
		 */

	} else {
		uint16_t stopmot[4] = { 0, 0, 0, 0 };
		dshot_write(stopmot);

		/*	stop_pwm(&M1);
		 stop_pwm(&M2);
		 stop_pwm(&M3);
		 stop_pwm(&M4);
		 */

	}
}

void PID_Execute_Acro(void) {

	RateB = 0.95;

	float e_acro[3];
	e_acro[0] = 5 * r[0] - (imu.gyr_dps[1] - ahrs.gyro_offset[1]);
	e_acro[1] = 5 * r[1] + (imu.gyr_dps[0] - ahrs.gyro_offset[0]);
	e_acro[2] = 5 * r[2] + (imu.gyr_dps[2] - ahrs.gyro_offset[2]);

	float pid_acro[3];
	float dt_1 = 1000; //  = 1/dt = 1/1ms = 1000
	pid_acro[0] = GyroKP * e_acro[0]
			+ GyroKD * (e_acro[0] - e_acro_prev[0]) * dt_1;
	pid_acro[1] = GyroKP * e_acro[1]
			+ GyroKD * (e_acro[1] - e_acro_prev[1]) * dt_1;
	pid_acro[2] = GyroKP * e_acro[2]
			+ GyroKD * (e_acro[2] - e_acro_prev[2]) * dt_1;

	e_acro_prev[0] = pid_acro[0];
	e_acro_prev[1] = pid_acro[1];
	e_acro_prev[2] = pid_acro[2];

	GyroPID[0] = (1 - RateB) * GyroPID[0] + RateB * pid_acro[0];
	GyroPID[1] = (1 - RateB) * GyroPID[1] + RateB * pid_acro[1];
	GyroPID[2] = (1 - RateB) * GyroPID[2] + RateB * pid_acro[2];

	LM.E[0] = (angleR[0] - imu.gyr_dps[1]);
	LM.E[1] = (angleR[1] + imu.gyr_dps[0]);
	LM.PID[0] = GyroPID[0];
	LM.PID[1] = GyroPID[1];
	M1_c = throttle + GyroPID[0] - GyroPID[1] + GyroPID[2]; //m1, m4 = CW, m2, m3 = CCW
	M2_c = throttle + GyroPID[0] + GyroPID[1] - GyroPID[2];
	M3_c = throttle - GyroPID[0] - GyroPID[1] - GyroPID[2];
	M4_c = throttle - GyroPID[0] + GyroPID[1] + GyroPID[2];

	if (throttle < 100) {
		M1_c = 0;
		M2_c = 0;
		M3_c = 0;
		M4_c = 0;
	} else {

		if (M1_c > 100) {
			M1_c -= 100;
		} else {
			M1_c = 0;
		}

		if (M2_c > 100) {
			M2_c -= 100;
		} else {
			M2_c = 0;
		}

		if (M3_c > 100) {
			M3_c -= 100;
		} else {
			M3_c = 0;
		}

		if (M4_c > 100) {
			M4_c -= 100;
		} else {
			M4_c = 0;
		}

	}

	uint16_t motor_command[4] = { M1_c, M2_c, M4_c, M3_c };
	//uint16_t motor_command[4] = {0, 0, M3_c, 0};

	if (armed_drone == true) {
		dshot_write(motor_command);

		/*		 servo_angle(&M1,	M1_c);
		 servo_angle(&M2,	M2_c);
		 servo_angle(&M3, M3_c);
		 servo_angle(&M4,	M4_c);
		 */

	} else {
		uint16_t stopmot[4] = { 0, 0, 0, 0 };
		dshot_write(stopmot);

		/*	stop_pwm(&M1);
		 stop_pwm(&M2);
		 stop_pwm(&M3);
		 stop_pwm(&M4);
		 */

	}
}

float ZaccelError;
float ZaccelPrev;
float ZAccelSum;

float uwb_z_prev;
float AccelInput;

float PosZSum = 0;

void AltitudeHold(void) {

	float errorZ = 0;

#ifdef AltitudeHoldChanges
	if (state.ZSP_changing == false) { //
		errorZ = state.UWB_deltaXY[2]*100; // Convert to cm for a translation from meters to throttle input

	}
#else
	float dt_uwb = (float) (sample_time / 1000.0f);
	delta_pos = state.UWB_VXY[2] * dt_uwb
			+ 0.5 * EKF_Z.measured_acceleration * dt_uwb * dt_uwb;
	delta_vel = EKF_Z.measured_acceleration * dt_uwb;

	errorZ = state.UWB_deltaXY[2]; // Convert to cm for a translation from meters to throttle input

#endif

//
//			float KpUWB_Z = 1.5;
//			float KiUWB_Z = 0.75;
//			float KdUWB_Z = 0.3;

//#define TUNE_KP_Z
#if defined(TUNE_KP_Z)

	KpUWB_Z = linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 3);
	KiUWB_Z = 0;
	KdUWB_Z = 0;

	state.KpUWB_Z = KpUWB_Z;

	#else

	KpUWB_Z = 1.5;
	KiUWB_Z = 0.8;
	KdUWB_Z = 8;
#endif

	float KpXY = 0.5; //Kp XY to map Position error to desired Velocity
	float KpVXY = 1.5; //Kp XY to map Velocity error to desired Acceleration

#ifdef Zctrl_1ms
float Zctrl_SampleTime  = 1e-3;
#else
	float Zctrl_SampleTime = (float) sample_time / 1000.0f;
#endif

	float tilt_angle = ahrs.R_Matrix_Pos[2][2];
//			c = (9.81 - Zacc)/tilt_angle;

	float Kt;

	if (throttleUWB_setP > 100) {
		Kt = state.initZ / throttleUWB_setP;
	}

	errorZ /= 100;
	float Zerror = (errorZ);
//	KpUWB_Z = linear_interp(CrsfChannels[11], 191-20, 1792+20, 0, 5);
	state.Kpcontrol = KpUWB_Z;

	float P = 1.8;

	float linear_dist = secnorderlim / (P * P);

	/*
	 */
	if (Zerror > linear_dist) {
		state.ZPiD[0] = safe_sqrt(
				2 * secnorderlim * (Zerror - (linear_dist / 2)));

	} else if (Zerror < -linear_dist) {
		state.ZPiD[0] = -safe_sqrt(
				2 * secnorderlim * (-Zerror - (linear_dist / 2)));

	} else {
		state.ZPiD[0] = Zerror * P;
	}

	state.ZVelError = (state.ZPiD[0] - (state.UWB_VXY[2]));

	if ((armed_drone == true) && (throttle >= 400)) {
		PosZSum += state.ZVelError * Zctrl_SampleTime;
	} else {
		PosZSum = 0;
	}

	PosZSum = constrain(PosZSum, -100, 100);

	float ZVKi = linear_interp(CrsfChannels[11], 191 - 20, 1792 + 20, 0, 2.5);

	state.ZVelP = state.ZVelError * KdUWB_Z;
	state.ZVelI = PosZSum * ZVKi;

	uwb_Z = state.ZVelP + state.ZVelI;

	uwb_Z = constrain(uwb_Z, -0.5 * 9.81, 0.5 * 9.81);

	state.ZPiD[1] = uwb_Z;

	float alp = 0.03;

	state.Aff = (1 - alp) * state.Aff
			+ alp * (uwb_Z - uwb_z_prev) * (1000 / (float) 1);

	uwb_z_prev = uwb_Z;

	float alp_z = 1;
	AccelInput = (1 - alp_z) * AccelInput
			+ alp_z * (state.roomInertial[2] - 9.71); //-9.71 to accommodate for small offset on IMU w/out calibration
	state.ZAccSP = AccelInput;
	ZaccelError = uwb_Z - AccelInput;

//	ZaccelError = 0 - AccelInput;

	float KpAccel = 0.75;
	float KiAccel = 2 * KpAccel;
	float KdAccel = 0.01;
//
	KpAccel = 1;
	KiAccel = 1.9;
	KdAccel = 0.02;

	if ((armed_drone == true) && (throttle >= 400)) {

		Z_sum += (errorZ) * Zctrl_SampleTime;
		ZAccelSum += (ZaccelError * (Zctrl_SampleTime));

	} else {

		Z_sum = 0;
		ZAccelSum = 0;

	}

#ifdef AltitudeHoldChanges
	if (state.ZSP_changing == true) {
		Z_sum = 0;
	}
#endif

#define ZAccLimit 5

	float KfA = linear_interp(CrsfChannels[11], 191 - 20, 1792 + 20, -1, 1);

	KfA = 0;

	Z_sum = constrain(Z_sum, -Zlimit, Zlimit+25);
	ZAccelSum = constrain(ZAccelSum, -ZAccLimit, ZAccLimit);

	float alpha_Z = 0.05911;
	state.ZAccelPID[0] = ZaccelError * KpAccel;
	state.ZAccelPID[1] = ZAccelSum * KiAccel;
	state.ZAccelPID[2] = (1 - alpha_Z) * state.ZAccelPID[2]
			+ alpha_Z * (AccelInput - ZaccelPrev) * (1000 / (float) 1)
					* KdAccel;

	e[2] = state.ZAccelPID[0] + state.ZAccelPID[1] + state.ZAccelPID[2]
			+ state.Aff * KfA;

#define ThrottleLim 200

	e[2] /= Kt;
	e[2] = constrain(e[2], -ThrottleLim, ThrottleLim);

	state.Kt = Kt;
	state.E2_NoFiltNoAngle = e[2];

	e[2] /= ahrs.R_Matrix_Pos[2][2];

	float ThrottleA = 0.3;
	state.throttleCorrection = (1 - ThrottleA) * state.throttleCorrection
			+ ThrottleA * e[2];

	state.ZPiD[2] = state.throttleCorrection;

	ZaccelPrev = AccelInput;

//	uwb_Z = state.ZPiD[0] + state.ZPiD[1] + state.ZPiD[2];
//			uwb_Z = state.ZPiD[1] + state.ZPiD[2];

	float Zdotdot = state.throttleCorrection * Kt * ahrs.R_Matrix_Pos[2][2];

	state.Zpid = uwb_Z;

	c = (9.81 - Zdotdot) / ahrs.R_Matrix_Pos[2][2];
//	c = (9.81-Zdotdot)/ahrs.R_Matrix_Pos[2][2];

	state.Zdotdot = Zdotdot;
	state.C = c;
//	uwb_Z /= tilt_angle;

	state.UWB_deltaXY_Prev[0] = state.UWB_deltaXY[0];
	state.UWB_deltaXY_Prev[1] = state.UWB_deltaXY[1];
	state.UWB_deltaXY_Prev[2] = errorZ;
//	 e[2] = uwb_Z;

}

/*
 * Pulled from ardupilot AP_Math/AP_Math.cpp
 */
float safe_sqrt(float v) {
	float ret = sqrtf((float) v);
	if (isnan(ret)) {
		return 0;
	}
	return ret;
}

void ELRS_RUN(void) {
	//HAL_UART_Receive_IT(&huart6, (uint8_t *)RxBuffer,LENGTH);
	HAL_UART_Receive_DMA(&huart6, (uint8_t*) RxBuffer, LENGTH);

}

void InitMagCal(void) {
	float HI[3] = { 29.1230, -1.6240, -4.4980 };

	thisMagCal.fV[0] = HI[0];
	thisMagCal.fV[1] = HI[1];
	thisMagCal.fV[2] = HI[2];

	float SI[9] = { 0.9990, 0.0220, 0.0210, 0.0220, 1.0010, 0.0150, 0.0210,
			0.0150, 1.0010 };

	thisMagCal.finvW[0][0] = SI[0];
	thisMagCal.finvW[0][1] = SI[1];
	thisMagCal.finvW[0][2] = SI[2];

	thisMagCal.finvW[1][0] = SI[3];
	thisMagCal.finvW[1][1] = SI[4];
	thisMagCal.finvW[1][2] = SI[5];

	thisMagCal.finvW[2][0] = SI[6];
	thisMagCal.finvW[2][1] = SI[7];
	thisMagCal.finvW[2][2] = SI[8];

	thisMagCal.fB = 50.8570;
	thisMagCal.fFourBsq = 10345.616;
	thisMagCal.fFitErrorpc = 2.3220;

	thisMagCal.iValidMagCal = 10;

	thisMagCal.PreloadedCal = 1;

	//Prevents an immediate calibration on startup that wipes this cal
	thisMagCal.iMagCalHasRun = 1;

	//Having trouble loading a calibration so will just disable it running while code is running
	//Have the code to calibrate the compass with the embedded device/no need for laptop to calibrate

	thisMagCal.enable = 0; //1 = on,  0 = off

	/*
	 * Sets the pre-found Magnetic Calibration for a
	 * faster startup and convergence
	 *
	 * Subsequent mag cals in code attempt to improve
	 * on this calibration.
	 */

}

void PID_run(void) {

	//	sprintf(chPC, "ch0: %d\r\nch1: %d\r\nch2: %d\r\nch3: %d\r\nch4: %d\r\nch5: %d\r\nch6: %d\r\nch7: %d\r\nch8: %d\r\nch9: %d\r\nch10: %d\r\nch11: %d\r\n", CrsfChannels[0], CrsfChannels[1], CrsfChannels[2], CrsfChannels[3], CrsfChannels[4], CrsfChannels[5], CrsfChannels[6], CrsfChannels[7], CrsfChannels[8], CrsfChannels[9], CrsfChannels[10], CrsfChannels[11]);
	//ch5 = switchF ,ch11 = s2
//	throttle = CrsfChannels[2];

	//ch9 = button D

	//locks the channel to be +50 the minimum value, -50 the maximum value, handles errors so r[] doesnt blow up to infinity
//	if (CrsfChannels[0] < xmin + 50) {
//		CrsfChannels[0] = xmin + 50;
//
//	} else if (CrsfChannels[0] > xmax - 50) {
//		CrsfChannels[0] = xmax - 50;
//	}
//	if (CrsfChannels[1] < xmin + 50) {
//		CrsfChannels[1] = xmin + 50;
//
//	} else if (CrsfChannels[1] > xmax - 50) {
//		CrsfChannels[1] = xmax - 50;
//	}
//	if (CrsfChannels[3] < xmin + 50) {
//		CrsfChannels[3] = xmin + 50;
//
//	} else if (CrsfChannels[3] > xmax - 50) {
//		CrsfChannels[3] = xmax - 50;
//	}
//
//
//	//r = commanded value
//	r[0] = linear_interp(CrsfChannels[0], xmin + 50, xmax - 50, -30, 30);
//	r[1] = linear_interp(CrsfChannels[1], xmin + 50, xmax - 50, -30, 30);
//	throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, dshotMax * 0.6);
//	r[2] = linear_interp(CrsfChannels[3], xmin + 50, xmax - 50, -75, 75);
//
//	if ((r[0] < 0.5) && (r[0] > -0.5)) {
//		r[0] = 0;
//	}
//	if ((r[1] < 0.5) && (r[1] > -0.5)) {
//		r[1] = 0;
//	}
//	if ((r[2] < 0.5) && (r[2] > -0.5)) {
//		r[2] = 0;
//	}
//
////Tune kp
////#define TUNE_KP
//#if defined(TUNE_KP)
//			if (CrsfChannels[9] == 1792) {
//				if (CrsfChannels[5] > 1200){
//					if (kp <= 6){
//					kp += 0.005;
//					}
//				} else if (CrsfChannels[5] < 800) {
//					if (kp >= 1){
//					kp -= 0.005;
//					}
//				}
//			}
//
//			LM.kp = kp;
//
//
//#else
//
//	kp = 1; //Tuned value
//#endif
//
////Tune ki
////#define TUNE_KI
//#if defined(TUNE_KI)
//			if (CrsfChannels[9] == 1792) {
//							if (CrsfChannels[5] > 1200){
//								if (ki <= 0.5){
//								ki += 0.000005;
//								}
//							} else if (CrsfChannels[5] < 800) {
//								if (ki >= 0.00005){
//								ki -= 0.000005;
//								}
//							}
//						}
//#endif
//
////Tune kd
////#define TUNE_KD
//#if defined(TUNE_KD)
//			if (CrsfChannels[9] == 1792) {
//							if (CrsfChannels[5] > 1200){
//								if (kd <= 3){
//								kd += 0.005;
//								}
//							} else if (CrsfChannels[5] < 800) {
//								if (kd > 0.005){
//								kd -= 0.005;
//								}
//							}
//						}
//#else
//	kd = 0; //tuned value
//#endif
//
////swap input[0] to theta, input[1] = -phi
//
//	e[0] = r[0] - ahrs.theta_deg;
//	e[1] = r[1] + ahrs.phi_deg;
//	e[2] = r[2] - ahrs.psi_deg;
//
//	if (armed_drone == true) {
//		esum[0] = 0.98 * esum[0] + e[0];
//		esum[1] = 0.98 * esum[1] + e[1];
//	} else {
//		esum[0] = 0;
//		esum[1] = 0;
//	}
//
//	float a = 0.8;
//	kdPID[0] = (1 - a) * (kdPID[0]) + (a) * ahrs.angular_velocity[0] * kd;
//	kdPID[1] = (1 - a) * (kdPID[1]) + (a) * ahrs.angular_velocity[1] * kd;
//
//	/*
//	 int intstat = 50;
//	 if ((esum[0]*ki > intstat) || (esum[0]*ki < -intstat)) {
//
//	 kiPID[0] = intstat;
//	 kiPID[1] = intstat;
//	 } else {
//	 kiPID[0] = esum[0]*ki;
//	 kiPID[1] = esum[1]*ki;
//	 }*/
//
////rollPID = e[0] * kp + kiPID[0] - kdPID[0];
////pitchPID = e[1] * kp + kiPID[1] - kdPID[1];
//
//
//	rollPID = e[0] * AngleKP ;//- kdPID[0];
//	pitchPID = e[1] * AngleKP ;//- kdPID[1];
//
//	yawPID = 0;
//	int maxPID = 200;
//
//
//
//	angleR[0] = Kl*rollPID;
//	angleR[1] = Kl*pitchPID;
//
//	angleR[0] = constrain(angleR[0], -maxPID, maxPID);
//	angleR[1] = constrain(angleR[1], -maxPID, maxPID);

//sprintf(chPC, "kp: %0.3f, rPID: %0.3f, e0: %0.3f, r0: %0.3f, phi: %0.3f \r\n",kp, rollPID, e[0], r[0], ahrs.phi_deg );
//sprintf(chPC, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d \r\n",kp, rollPID, e[0], r[0], ahrs.phi_deg, CrsfChannels[0] );

#if defined(anglemode)
	M1_c = throttle + rollPID - pitchPID + yawPID;  //m1, m4 = CW, m2, m3 = CCW
	M2_c = throttle + rollPID + pitchPID - yawPID;
	M3_c = throttle - rollPID - pitchPID - yawPID;
	M4_c = throttle - rollPID + pitchPID + yawPID;

	memset(chPC, 0, sizeof(chPC));

	//posChar;
	char signoff[] = "\r\n";

	//sprintf(chPC, "%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%d,%0.3f,%0.3f,%0.3f,%d\r\n", throttle, rollPID, pitchPID, yawPID, ahrs.theta_deg, ahrs.phi_deg, ahrs.psi_deg, kp, throttle, kdPID[0], ahrs.angular_velocity[0], kd,elrs_error );

//sprintf(chPC, "%d,%d,%d,%d,%d,%d,%d,", M1_c, M2_c, M3_c, M4_c);
	//memcpy(chPC + strlen(chPC), txStats, strlen(txStats)+1);
//	test_run_info((unsigned char *)chPC);

	memset(Buffer, 0x00, sizeof(Buffer));
	memcpy(Buffer, chPC, strlen(chPC));
//memset(chPC, 0, strlen(chPC));

	size_t buf = strlen(chPC);
//memset(chPC, 0 , buf);

	if (throttle < 100) {
		M1_c = 0;
		M2_c = 0;
		M3_c = 0;
		M4_c = 0;
	} else {


		if (M1_c > 100){
			M1_c -= 100;
		} else {
			M1_c = 0;
		}

		if (M2_c > 100){
			M2_c -= 100;
		} else {
			M2_c = 0;
		}

		if (M3_c > 100){
			M3_c -= 100;
		} else {
			M3_c = 0;
		}

		if (M4_c > 100){
			M4_c -= 100;
		} else {
			M4_c = 0;
		}
//		M1_c -= 100;
//		M2_c -= 100;
//		M3_c -= 100;
//		M4_c -= 100;

	}




	uint16_t motor_command[4] = { M1_c, M2_c, M4_c, M3_c };
//uint16_t motor_command[4] = {0, 0, M3_c, 0};

	if (armed_drone == true) {
		dshot_write(motor_command);

		/*		 servo_angle(&M1,	M1_c);
		 servo_angle(&M2,	M2_c);
		 servo_angle(&M3, M3_c);
		 servo_angle(&M4,	M4_c);
		 */

	} else {
		uint16_t stopmot[4] = { 0, 0, 0, 0 };
		dshot_write(stopmot);

		/*	stop_pwm(&M1);
		 stop_pwm(&M2);
		 stop_pwm(&M3);
		 stop_pwm(&M4);
		 */

	}

	LM.Angle[0] = ahrs.theta_deg;
	LM.Angle[1] = ahrs.phi_deg;
	LM.E[0] = e[0];
	LM.E[1] = e[1];
	LM.R[0] = r[0];
	LM.R[1] = r[1];
	LM.Motor[0] = M1_c;
	LM.Motor[1] = M2_c;
	LM.Motor[2] = M3_c;
	LM.Motor[3] = M4_c;
	LM.PID[0] = rollPID;
	LM.PID[1] = pitchPID;
#endif
}

lfs_soff_t file_size;
int readtrue = 0;
lfs_ssize_t actualread;
int read_buf_len;
#define STRING_BUF_SZ 256
#define READ_BUF_SZ 256

void init_littlefs_reader() {
	char *string_buffer1 = (char*) malloc(STRING_BUF_SZ * sizeof(char));
	char *string_buffer2 = (char*) malloc(STRING_BUF_SZ * sizeof(char));
	uint8_t *read_buf = (uint8_t*) malloc(READ_BUF_SZ * sizeof(uint8_t));

	char headers[] =
			"Time (ms), Voltage (V), FlightPhase, Acceleration magnitude (g), Roll Rate (dps), Altitude (m), Latitude (DD), Longitude (DD), qs, qx, qy, qz, Velocity (m/s), "
					"Latitude (DD), Longitude (DD), Altitude (m), Velocity North (mm/s), Velocity East (mm/s), Velocity Down (mm/s), Acceleration EX (g), Acceleration EY (g), Acceleration EZ (g), Roll Rate BX (dps), Roll Rate BY (dps), Roll Rate BZ (dps),"
					"Altitude (m)\r\n";
	uint16_t headerlen = sizeof(headers);

	HAL_Delay(1000);//1s delay and a break point set here to give time to prep serial monitor to read data, addition 10s delay set prior to this
	//	 	test_run_info_rxBuf((unsigned char *)headers, headerlen);  //Print headers first to appear at top of CSV file

	int ret_init = w25qxx_littlefs_init();

	lfs_file_open(&littlefs, &file, filename, LFS_O_RDONLY); //Open instance of littleFs

	file_size = lfs_file_size(&littlefs, &file); //Return the size of the file to be read

	uint8_t txbuf_reader[256]; //Initialise a buffer to transmit over USB

	memset(txbuf_reader, 0, 256);
	sprintf(txbuf_reader, "%d\n", file_size);
	//    test_run_info_rxBuf((unsigned char *)txbuf_reader, (uint16_t)sizeof(txbuf_reader));//Call a USB transmit function to send data on USB

	HAL_Delay(100);
	if (readtrue == 0) { //Only call this function once
		for (lfs_size_t i = 0; i < (lfs_size_t) file_size; i += READ_BUF_SZ) { //Loop the size of the file, incrementing the USB buffer length = 256 bytes
			memset(read_buf, 0, READ_BUF_SZ); //Clear the buffer to 0s (NULL characters)
			memset(txbuf_reader, 0, 256); //Clear the buffer to 0s (NULL characters)
			const lfs_size_t bytes_read = lfs_min(READ_BUF_SZ, file_size - i); //Determine if there are less than 256 bytes left to read (unless for the last read of the file

			actualread = lfs_file_read(&littlefs, &file, read_buf, bytes_read); //Read either 256 bytes or the remaining bytes in file ( < 256 bytes)
			memcpy(txbuf_reader, read_buf, 256); //Copy the buffer contents to our USB buffer (good for debugginig the contents of the buffer)

			read_buf_len = sizeof(txbuf_reader); //get the size of the buffer
			test_run_info_rxBuf((unsigned char*) txbuf_reader,
					(uint16_t) read_buf_len); //Call a USB transmit function to send data on USB
			HAL_Delay(1); //Delay by 1ms to allow the serial monitor enough time to read the COM port
		} //end of for loop

		readtrue++; //add 1 to readtrue so only 1 copy of the file is sent during the code execution
	}
}

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM3 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM3) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_Delay(100);
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
