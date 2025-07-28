/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern State_Vector state;

#include "Stateest.h"

#include "tasks_uwb.h"
#include "usart.h"
extern kalman_filter_t EKF_X;
extern kalman_filter_t EKF_Y;
extern kalman_filter_t EKF_Z;

#include "fdcan.h"
#include "tim.h"

extern int printData;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

int PRES_INIT;


extern struct MagCalibration thisMagCal;			// hard and soft iron magnetic calibration
extern struct MagneticBuffer thisMagBuffer;		// magnetometer measurement buffer
#include "include_all.h"

uint32_t tick1, tick2, rtos_on;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float calculate_height(float pressure) {
		  return (-(powf(pressure / PRES_INIT , (1 / 5.257F)) - 1) * (44330));
		}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId Flash_WriteHandle;
osThreadId MS5607_ReadHandle;
osThreadId RadioFunctionHandle;
osThreadId GNSS_pollHandle;
osThreadId AHRSHandle;
osThreadId Read_UARRHandle;
osThreadId UWB_PositionHandle;
osThreadId MagCalHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern float sample_time;
extern bool isMaster;
extern struct MS5607Readings readings;
extern FDCAN_TxHeaderTypeDef   TxHeader;
extern char txBuf_FLASH[200];

extern int CANcounter;
/* USER CODE END FunctionPrototypes */

void LogData(void const * argument);
void BaroRead(void const * argument);
void RadioRun(void const * argument);
void GNSS_read(void const * argument);
void EKF_Run(void const * argument);
void UART_READ(void const * argument);
void UWB_Solve(void const * argument);
void RunMagCal(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Flash_Write */
  osThreadDef(Flash_Write, LogData, osPriorityAboveNormal, 0, 2048);
  Flash_WriteHandle = osThreadCreate(osThread(Flash_Write), NULL);

  /* definition and creation of MS5607_Read */
  osThreadDef(MS5607_Read, BaroRead, osPriorityNormal, 0, 128);
  MS5607_ReadHandle = osThreadCreate(osThread(MS5607_Read), NULL);

  /* definition and creation of RadioFunction */
  osThreadDef(RadioFunction, RadioRun, osPriorityNormal, 0, 512);
  RadioFunctionHandle = osThreadCreate(osThread(RadioFunction), NULL);

  /* definition and creation of GNSS_poll */
  osThreadDef(GNSS_poll, GNSS_read, osPriorityNormal, 0, 612);
  GNSS_pollHandle = osThreadCreate(osThread(GNSS_poll), NULL);

  /* definition and creation of AHRS */
  osThreadDef(AHRS, EKF_Run, osPriorityAboveNormal, 0, 1024);
  AHRSHandle = osThreadCreate(osThread(AHRS), NULL);

  /* definition and creation of Read_UARR */
  osThreadDef(Read_UARR, UART_READ, osPriorityNormal, 0, 1024);
  Read_UARRHandle = osThreadCreate(osThread(Read_UARR), NULL);

  /* definition and creation of UWB_Position */
  osThreadDef(UWB_Position, UWB_Solve, osPriorityAboveNormal, 0, 1024);
  UWB_PositionHandle = osThreadCreate(osThread(UWB_Position), NULL);

  /* definition and creation of MagCal */
  osThreadDef(MagCal, RunMagCal, osPriorityIdle, 0, 512);
  MagCalHandle = osThreadCreate(osThread(MagCal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_LogData */
/**
 * @brief  Function implementing the Flash_Write thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_LogData */
void LogData(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN LogData */

  int init1 = 0;
#if defined(dataPrint)
  HAL_Delay(10000);
#endif
	/* Infinite loop */
	for (;;) {

if (printData == 1) {

	  if (init1 == 0) {
		  init_littlefs_reader();

		init1++;
	  }



} else if (printData == 2) {
	char eraseBuf[] = "Beginning Erase";
	char eraseBuf1[] = "Erase Complete";

	    test_run_info_rxBuf((unsigned char *)eraseBuf, (uint16_t)sizeof(eraseBuf));//Call a USB transmit function to send data on USB
	CSP_QSPI_Erase_Chip();
    test_run_info_rxBuf((unsigned char *)eraseBuf1, (uint16_t)sizeof(eraseBuf1));//Call a USB transmit function to send data on USB
    printData = 0;
} else {

		FLASH_LOG();
	  }


		osDelay(20);
	}
  /* USER CODE END LogData */
}

/* USER CODE BEGIN Header_BaroRead */
/**
 * @brief Function implementing the MS5607_Read thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_BaroRead */
void BaroRead(void const * argument)
{
  /* USER CODE BEGIN BaroRead */

#if defined(useBaro)
	int counter = 0;

	int32_t pressure_read;
					int32_t pressure_sum = 0;
				 	for (int i = 0; i < 10; i++) {
				 		pressure_read = MS5607Update();
				 		pressure_sum += pressure_read;

				 	}
				 	PRES_INIT = pressure_sum / 10;
#endif

	/* Infinite loop */
	for (;;) {

#if defined(useBaro)
	if (counter = 0) {

counter++;
	}

		state.pressure = MS5607Update();
		//state.pressure_sum += state.pressure;
		//state.press_count++;

		state.altitude = calculate_height(state.pressure);


#endif

		osDelay(5);
	}
  /* USER CODE END BaroRead */
}

/* USER CODE BEGIN Header_RadioRun */
/**
 * @brief Function implementing the RadioFunction thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RadioRun */
void RadioRun(void const * argument)
{
  /* USER CODE BEGIN RadioRun */
	uint32_t delay = 0;

	/* Infinite loop */
	for (;;) {
#if defined(useLora)
		radio_functions();
			if (isMaster == true) {
				delay = 20;
			} else if (isMaster == false) {
				delay = 50;
			}
#endif
		osDelay(delay);
	}
  /* USER CODE END RadioRun */
}

/* USER CODE BEGIN Header_GNSS_read */
/**
 * @brief Function implementing the GNSS_poll thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GNSS_read */
void GNSS_read(void const * argument)
{
  /* USER CODE BEGIN GNSS_read */
	/* Infinite loop */
	for (;;) {
#if defined(useGPS)
		  GNSS_READ();
#endif
		osDelay(50);
	}
  /* USER CODE END GNSS_read */
}

/* USER CODE BEGIN Header_EKF_Run */
/**
 * @brief Function implementing the AHRS thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_EKF_Run */
void EKF_Run(void const * argument)
{
  /* USER CODE BEGIN EKF_Run */




	/* Infinite loop */
	for (;;) {

		uint32_t beginT = HAL_GetTick();
#if defined(useOrientation)
		EKF();
#endif
uint32_t endT = HAL_GetTick();
uint32_t dt_EKF = endT - beginT;

#if defined(useELRS)

	HAL_UART_Receive_DMA(&huart6, (uint8_t*) RxBuffer, LENGTH);

#endif
		osDelay(sample_time);
	}
  /* USER CODE END EKF_Run */
}

/* USER CODE BEGIN Header_UART_READ */
/**
 * @brief Function implementing the Read_UARR thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UART_READ */
void UART_READ(void const * argument)
{
  /* USER CODE BEGIN UART_READ */
	/* Infinite loop */
	for (;;) {
#if defined(UWB_UART)
		UART_FREERTOS();
#endif
		osDelay(5);
	}
  /* USER CODE END UART_READ */
}

/* USER CODE BEGIN Header_UWB_Solve */
/**
* @brief Function implementing the UWB_Position thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UWB_Solve */
void UWB_Solve(void const * argument)
{
  /* USER CODE BEGIN UWB_Solve */


	float32_t initX = (float32_t)state.uwbinitXY[0]/(state.uwbinitcount+1);
	float32_t initY = (float32_t)state.uwbinitXY[1]/(state.uwbinitcount+1);
	float32_t initZ = (float32_t)state.uwbinitXY[2]/(state.uwbinitcount+1);


	init_filter_struct(&EKF_X);
	initialize_matrices(&EKF_X, initX);

	init_filter_struct(&EKF_Y);
	initialize_matrices(&EKF_Y, initY);

	init_filter_struct(&EKF_Z);
	initialize_matrices(&EKF_Z, initZ);

	state.uwbfirstval = true;
	state.uwbinit = true;

	extern int newData;

  /* Infinite loop */
  for(;;)
  {
#if defined(useUWB)

	  if (newData == 1) {


//	  PositionCompUWB();
//	  newData = 0;
//	  CANcounter=0;
//
//	  tick2 = osKernelSysTick() - tick1;
//	  tick1 = osKernelSysTick();

	  }
	  rtos_on = 1;

		EKF_X.measured_acceleration = state.roomInertial[1];
									EKF_Y.measured_acceleration = state.roomInertial[0];
											EKF_Z.measured_acceleration = (state.roomInertial[2]-9.675);

											 kalman_prediction(&EKF_X);
												 kalman_prediction(&EKF_Y);
												 kalman_prediction(&EKF_Z);


//	  EKFUpdate();

	  EKFcompUWB();
#endif

/*
 *
 * solve matrix position
 *
 * compute EKF
 *
 *
 */


#if defined(HILuseUWB) //Hardware in Loop, data is received at USB from PC, the state is updated and returned to the PC

	  if (state.HIL_counter > state.HIL_counter_prev){


	//  	Kalman_X((float32_t)state.HIL_UWBP[0],(float32_t)state.HIL_UWBA[0]);
	//  	Kalman_Y((float32_t)state.HIL_UWBP[1],(float32_t)state.HIL_UWBA[1]);
		  Kalman_X();
		  Kalman_Y();

	  	 char tx_USB_buffer[100];
	  		 		  sprintf(tx_USB_buffer, "%0.3f, %0.3f, %0.3f,%0.3f, %0.3f, %0.3f\n" , state.uwb_ekf_x[0], state.uwb_ekf_x[1], state.uwb_ekf_x[2],
	  		 				state.uwb_ekf_y[0], state.uwb_ekf_y[1], state.uwb_ekf_y[2]);

	  		 		//  sprintf(tx_USB_buffer, "%d\n", state.HIL_counter_prev);


	  		 		  test_run_info_rxBuf(tx_USB_buffer,strlen(tx_USB_buffer));
	  		 		  memset(tx_USB_buffer, 0, strlen(tx_USB_buffer));
state.HIL_counter_prev = state.HIL_counter;



	  }
	    osDelay(1);
#else
	    osDelay(10);

#endif
  }
  /* USER CODE END UWB_Solve */
}

/* USER CODE BEGIN Header_RunMagCal */
/**
* @brief Function implementing the MagCal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RunMagCal */
void RunMagCal(void const * argument)
{
  /* USER CODE BEGIN RunMagCal */


  /* Infinite loop */
  for(;;)
  {
	  if ((thisMagCal.iCalInProgress == 1)&&(thisMagCal.enable == 1)) {
			MagCal_Run(&thisMagCal, &thisMagBuffer);
	  } else {
		  thisMagCal.iCalInProgress = 0;
	  }

    osDelay(10);
  }
  /* USER CODE END RunMagCal */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
