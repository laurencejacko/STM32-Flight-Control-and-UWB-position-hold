/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mode.h"
#include "delay.h"
#include "sx1280.h"
#include "sx1280_hal.h"
#include "sx1280_radio.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/*----- Enable/Disable components --------- */

#define OVERSAMPLE_RATIO 8

#define Zlimit 100

#define throttleHover 900
#define throttleIbegin (throttleHover - 200)
#define throttleMin 650


#define Z_totalLowerLimit -100
#define Z_totalHigherLimit 100

#define maxDshot 2047
#define MAX_THROTTLE (maxDshot * 0.7)
#define MAX_V 25


#define MAX_VZ 0.75//Max ascent/decent velocity
//#define MAG_VXY 0.75 //Max XY velocity
#define MAG_VXY 1 //Max XY velocity

					#define MAG_AXY 4 //Max XY acceleration
					#define Z_acc_Max 4 //Max ascent/decent acceleration


//#define dataPrint

#ifndef dataPrint

//#define HILuseUWB


#ifndef HILuseUWB
#define transmitter

#if defined(transmitter)
//#define useHX711
//#define useLora
#define useOrientation
#define useBaro
#define useGPS

//#define LogFlash //call the function to write to memory

#define usePID
#define DShot
#define useELRS
#define useUWB
#define Angle_Mode

#define LittleFS_Flash

//#define Zctrl_1ms //if enabled, Z controls every 1ms with IMU interrupt else runs at EKf frequency


#ifdef Angle_Mode
#ifdef useUWB
#define UWBtoggleSwitch
//#define velControl //enable velocity control
//#define UWB_MODE //if angle mode and use uwb are true, SELECT if we control using UWB, UNSELECT if control with IMU
#endif
#endif

#ifndef LittleFS_Flash
//#define printQuat
//#define printEuler
//#define printLinAccel
//#define printMagVal
//#define printGyro
//#define printUWB
//#define printMagCal
//#define printMagCalibrated
//#define printRoomInertial
//#define printImuAccel
//#define CRSFvals
//#define printXYuwbPID
//#define PrintAccControl
//#define PrintEKFaccel
//#define PrintMeasuredAGL
//#define PrintVelocity
//#define PrintUWB_XYZ
#endif


//#define FDCANtoFDCAN
#if defined(FDCANtoFDCAN)
#define FDtestB1
#define FDtestB2
#endif

#define UWB_RX //for UWB receiving on CAN
//#define UWB_UART //for UWB receiving on UART

#else

#define useLora

#endif


#endif
#endif
//#define clearMem  //Will clear the flash memory upon startup if defined

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
typedef struct {
	uint8_t addr;
//  uint8_t seq[100];
	uint8_t seq;
//char txBuf[100];
} pkt_t;


typedef struct {

	float quat[4]; //Quaternion q = q0 + q1.i + q2.j + q3.k
	float euler[3]; //Roll Pitch Yaw
	float pressure; //Pressure (Pa)
	float latP; //latitude position
	float lonP; //longitude position
	float latVel; //latitude velocity
	float lonVel;  //longitude velocity
	float angular_velocity[3]; //Angular velocity
	float linAccel[3]; //Earth axis linear acceleration

	float roomInertial[3];

	float positionUWB[6];
	float RSSI_UWB[6];

	float linAcc_sum[3];
	float pressure_sum;
	int acc_count, press_count;

	float baro_temp;

	float altitude;

	float UWB_XY[3];
	float UWB_deltaXY[3];
	float UWB_setpoint[3];
	float UWB_VXY[3];

	float UWB_deltaXY_Prev[3];

	float hVel;
	int GNSSfixType;


	float TargetApogee;

	float HIL_ts;
	float HIL_acc[3];
	float HIL_gyr[3];
	float HIL_pressure;
	int HIL_counter;
	int HIL_counter_prev;

	float HIL_UWBP[2];
	float HIL_UWBA[2];
	float HIL_YAW;
	float dummyval[5];


	float uwb_ekf_x[3];
	float uwb_ekf_y[3];
	float uwb_ekf_z[3];

	float height;
	float vel_Z;
	float omega[3];

	float battV;
	float time;
	int flightPhase;
	int calAxis;

	float UWB_dtUpdate;

	float acceleration;

	float kp_angle[2];
	float ki_angle[2];

	float angleR_angle[2];

	float kp_gyro[3];
	float kd_gyro[3];
	float ki_gyro[3];

	float gyroPID[3];

	float filtYawRate;

	float uwbAngle[2];

	float velR[2];

	float uwbinitXY[3];
	bool uwbinit;
	bool uwbfirstval;
	int uwbinitcount;

	float ZPiD[3];
	float ZAccelPID[3];
	float x_pid;
	float y_pid;

	float ZvelSetPoint;

	bool ZSP_changing;

	float uwbRotM[3][3];

	float Zpid;

	float uwbKp[2];
	float uwbKi[2];
	float uwbKd[2];

	float KpUWB_Z;
	float uwbKP;

	float predictionCount;

	float throttleCorrection;
	float roomInertialAvg[3];

	float Kpcontrol;

	float uwbKpV[2];
	float ZAccSP;

	float E2_NoFiltNoAngle;
	float Kt;
	float C;
	float Zdotdot;

	float YawKp;
	float YawKi;
	float YawNewAngle;

	float POS[3];
	float Aff;
	float KiControl;

	float Pos_error;
	float VKI;

	float initZ;

	float ZVelError;
	float ZVelP;
	float ZVelI;

} State_Vector;

typedef struct {

	int Motor[4];
	float PID[3];
	float R[3];
	float Angle[3];
	float E[3];
	float kp;

} LoraMotor;

typedef struct  {

	float r[3];
	float y[3];
	float e[3];
	float Isum[3];
	float dPrev[3];

	float kp;
	float ki;
	float kd;




}pid_;

//pid_ angle;
//pid_ gyro;
//pid_ uwb;
//pid_ vel;


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define xmin 172
#define xmax 1811
#define motmin 1000
#define motmax 2000

uint32_t map_angle(uint32_t x, uint32_t in_min, uint32_t in_max,
		uint32_t out_min, uint32_t out_max);

float safe_sqrt(float v);

#define secnorderlim 0.25


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef enum {
	APP_LOWPOWER,
	APP_RUNNING,
	APP_RX,
	APP_RX_TIMEOUT,
	APP_RX_ERROR,
	APP_TX,
	APP_TX_TIMEOUT,
} AppStates_t;
extern AppStates_t AppState;
extern uint8_t mode;
extern uint16_t RxIrqMask;
extern uint16_t TxIrqMask;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_2
#define LED3_GPIO_Port GPIOC
#define SPI1_INT_Pin GPIO_PIN_4
#define SPI1_INT_GPIO_Port GPIOC
#define SPI1_INT_EXTI_IRQn EXTI4_IRQn
#define SPI1_CS_Pin GPIO_PIN_5
#define SPI1_CS_GPIO_Port GPIOC
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define I2C2_DRDY_Pin GPIO_PIN_10
#define I2C2_DRDY_GPIO_Port GPIOE
#define I2C2_DRDY_EXTI_IRQn EXTI15_10_IRQn
#define PWM2_Pin GPIO_PIN_11
#define PWM2_GPIO_Port GPIOE
#define GPS_RESET_Pin GPIO_PIN_12
#define GPS_RESET_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define I2C2_INT_Pin GPIO_PIN_15
#define I2C2_INT_GPIO_Port GPIOE
#define I2C2_INT_EXTI_IRQn EXTI15_10_IRQn
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define RADIO_BUSY_Pin GPIO_PIN_8
#define RADIO_BUSY_GPIO_Port GPIOD
#define RADIO_BUSY_EXTI_IRQn EXTI9_5_IRQn
#define DIO1_Pin GPIO_PIN_9
#define DIO1_GPIO_Port GPIOD
#define DIO1_EXTI_IRQn EXTI9_5_IRQn
#define RADIO_RESET_Pin GPIO_PIN_10
#define RADIO_RESET_GPIO_Port GPIOD
#define SPI3_CS_Pin GPIO_PIN_0
#define SPI3_CS_GPIO_Port GPIOD
#define CAN_STBY_Pin GPIO_PIN_0
#define CAN_STBY_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
#define RX_TIMEOUT_VALUE                            1000 //  ms
#define RX_TIMEOUT_TICK_SIZE                        RADIO_TICK_SIZE_4000_US
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
