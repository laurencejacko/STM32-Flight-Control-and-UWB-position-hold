/*
 * tasks.c
 *
 *  Created on: Oct 29, 2024
 *      Author: laure
 */
#include <tasks_uwb.h>
#include "arm_math.h"
#include "GNSS.h"
#include "LIS3MDL.h"
#include "ICM42688.h"
#include "EKF_3D.h"
#include "Stateest.h"
#include "CRSF.h"
#include "cmsis_os.h"
#include "main.h"

extern GNSS_StateHandle GNSS_Handle;

extern LIS3MDL_t mag;
extern ICM42688 imu;
extern AHRS0 ahrs;

extern State_Vector state;

extern kalman_filter_t EKF_X;
extern kalman_filter_t EKF_Y;
extern kalman_filter_t EKF_Z;


#define BUFFER_LEN_POS 10
			float POS_BUFF[BUFFER_LEN_POS][3];
float POS[3];
			int POS_IND=0;



/*
 *
 * UWB
 *
 *
 */

/*
 * A B C D E
 */
#define pseudoRows 4
#define pseudoCols 5

#define UWBdims 3

//float pseudodataA[pseudoRows][pseudoCols] = {
//		{      0.2233  ,  1.8596 ,   0.1720 ,  -1.6445  ,  0.3896},
//		{    0.0236  ,  0.2347  ,  0.0189  , -0.0119  , -0.2654},
//		{     0.1519 ,  -0.2213 ,  -0.1907 ,   0.1631 ,   0.0971},
//		{    0.0082  ,  0.5581  ,  0.0015 ,  -0.5597 ,  -0.0081}  }; //Psuedoinv of A matrix

float pseudodataA[pseudoRows][pseudoCols] = {
		{       -0.5242 ,   1.2363 ,  -0.4241 ,  -1.0502 ,   1.7623},
		{      0.0766 ,   0.1488 ,   0.0620 ,  -0.1561 ,  -0.1313},
		{      0.1501 ,  -0.0516 ,  -0.1971 ,   0.0714  ,  0.0271},
		{     -0.1999  ,  0.2807 ,  -0.1617 ,  -0.3390  ,  0.4199}  }; //Psuedoinv of A matrix



#define anchorlen 5

arm_matrix_instance_f32 pinvA = { pseudoRows, pseudoCols, (float32_t*) pseudodataA };

float b[anchorlen];
arm_matrix_instance_f32 matb = { anchorlen, 1, (float32_t*) b };

//float anchorsquare[anchorlen] = {       4.3459   , 2.7595   , 6.8970 ,   6.0054  ,  2.7737}; // B = S(i)^2 - sum(anchor.^2) ----> anchorsquare = sum(anchor.^2)
float anchorsquare[anchorlen] = {     6.7251 ,    3.3936 ,   7.0018 ,   5.8347 ,   2.7462}; // B = S(i)^2 - sum(anchor.^2) ----> anchorsquare = sum(anchor.^2)

float dataUWB[anchorlen];



float32_t Xpdw[UWBdims+1];
arm_matrix_instance_f32 positionresult = { UWBdims+1, 1, Xpdw };

void PositionCompUWB(void){

	arm_status status;

	for (int i = 0; i < anchorlen; i++) {
		dataUWB[i] = (state.positionUWB[i] * state.positionUWB[i])
				- anchorsquare[i];
	}

	arm_mat_init_f32(&matb, anchorlen, 1, (float32_t*) &dataUWB); //Initialise B matrix

#if defined(UseMatrixW)
// Xpdw = inv(Transpose(A) * C * A) * Transpose(A) * C * b
arm_mat_mult_f32(&matATCA_inv, &matb, &positionresult);
#else
	status = arm_mat_mult_f32(&pinvA, &matb, &positionresult);

	EKF_X.inputPos = Xpdw[1];
	EKF_Y.inputPos = Xpdw[2];
	EKF_Z.inputPos = Xpdw[3];


}

void EKFUpdate(void){


	float errorDiff = sqrt(fabs(Xpdw[0] - ((Xpdw[1]*Xpdw[1] + Xpdw[2]*Xpdw[2] + Xpdw[3]*Xpdw[3]))));
	state.Pos_error = 0.8*state.Pos_error + 0.2*errorDiff;

	if ((state.Pos_error  > 0.65) || (errorDiff > 0.65)){
		EKF_X.measured_AGL = (float32_t) Xpdw[1];
		EKF_Y.measured_AGL = (float32_t) Xpdw[2];

	} else {
		EKF_X.measured_AGL = (float32_t) Xpdw[1];
		EKF_Y.measured_AGL = (float32_t) Xpdw[2];
		EKF_Z.measured_AGL = (float32_t) Xpdw[3];
	}




	if (state.uwbinit == true) {
		Kalman_X();
		Kalman_Y();
		Kalman_Z();
	}




	if (state.uwbfirstval == false) {
		state.uwbinitXY[0] += Xpdw[1];
		state.uwbinitXY[1] += Xpdw[2];
		state.uwbinitXY[2] += Xpdw[3];

		state.uwbinitcount++;
	}

		state.UWB_XY[0] = state.uwb_ekf_x[0];
			state.UWB_XY[1] = state.uwb_ekf_y[0];

		//	state.UWB_XY[0] = EKF_X.measured_AGL;
		//	state.UWB_XY[1] = EKF_Y.measured_AGL;


			state.UWB_XY[2] = state.uwb_ekf_z[0];







			/*
			 * Perform a moving average on less accurate Z measurement
			 */
			POS[2] -= POS_BUFF[POS_IND][2];
			POS_BUFF[POS_IND][2] = state.UWB_XY[2];
			POS[2] += POS_BUFF[POS_IND][2];

			POS_IND++;

			state.POS[2] = POS[2]/BUFFER_LEN_POS;

			if (POS_IND == BUFFER_LEN_POS) POS_IND = 0;

			state.UWB_deltaXY[0] = state.UWB_setpoint[0] - state.UWB_XY[0];
					state.UWB_deltaXY[1] = state.UWB_setpoint[1] - state.UWB_XY[1];
					state.UWB_deltaXY[2] = state.UWB_setpoint[2] - state.POS[2];


				//	state.UWB_deltaXY[0] *= 100;
				//	state.UWB_deltaXY[1] *= 100;
					state.UWB_deltaXY[2] *= 100;


					state.UWB_VXY[0] = (float) EKF_X.x_bar_data[1];
														state.UWB_VXY[1] = (float) EKF_Y.x_bar_data[1];
														state.UWB_VXY[2] = (float) EKF_Z.x_bar_data[1];



}

void StateUpdatePrediction(void){


//	state.UWB_XY[0] = state.uwb_ekf_x[0];
//		state.UWB_XY[1] = state.uwb_ekf_y[0];
//
//	//	state.UWB_XY[0] = EKF_X.measured_AGL;
//	//	state.UWB_XY[1] = EKF_Y.measured_AGL;
//
//
//		state.UWB_XY[2] = state.uwb_ekf_z[0];
//
//		state.UWB_VXY[0] = state.uwb_ekf_x[1];
//		state.UWB_VXY[1] = state.uwb_ekf_y[1];
//		state.UWB_VXY[2] = state.uwb_ekf_z[1];

//		state.UWB_XY[0] = (float) EKF_X.x_hat_data[0];
//				state.UWB_XY[1] = (float) EKF_Y.x_hat_data[0];
//
//			//	state.UWB_XY[0] = EKF_X.measured_AGL;
//			//	state.UWB_XY[1] = EKF_Y.measured_AGL;
//
//
//				state.UWB_XY[2] = (float) EKF_Z.x_hat_data[0];



//			state.UWB_VXY[0] = (float) EKF_X.x_hat_data[1];
//					state.UWB_VXY[1] = (float) EKF_Y.x_hat_data[1];
//					state.UWB_VXY[2] = (float) EKF_Z.x_hat_data[1];

}


extern uint32_t uwbtime;
extern bool collect_setpoint;
extern bool setpoint_collected;
extern int CrsfChannels[CRSF_NUM_CHANNELS];

extern int throttleUWB_setP;
extern float angleR[3];

extern int throttle;

extern float psi_init ;

extern float esum[3];

extern bool initYaw;
extern bool armed_drone;

extern uint32_t YawWaitTime;
extern uint32_t rtos_on;

extern bool YawOn;

extern float Z_sum;
extern float ZAccelSum;
extern float XYsum[2];
extern float VelErrorPrev[2];
extern float ZaccelPrev;

extern float trim[2];
extern float r[3];

extern float PosZSum;

void EKFcompUWB(void) {

//	if ((HAL_GetTick() - dt_UWB) > 1000) {
////		UWB_data = false;  //if no new data for 1000ms disable UWB positioning
//	} else if (UWB_data == true) {
//		//	UWB_data = true;
//	}

//#ifndef UWB_MODE
//
////	UWB_data = false;
//
//#endif

//	if ((fabs(Xpdw[1]) < 3 )&&(fabs(Xpdw[2]) < 3 )){
//	state.UWB_XY[0] = ((1-uwbalpha)*state.UWB_XY[0]) + uwbalpha*(float)Xpdw[1];
//	state.UWB_XY[1] = ((1-uwbalpha)*state.UWB_XY[1]) + uwbalpha*(float)Xpdw[2];
//	}



//					float NED[3];
//		NED[0] = (float32_t) ((9.81)*(ahrs.R_Matrix_Pos[0][0]* imu.acc_g[0] +
//					ahrs.R_Matrix_Pos[1][0] * imu.acc_g[1] +
//			 ahrs.R_Matrix_Pos[2][0] * imu.acc_g[2]));
//

//
//
//
//		NED[1]=	(float32_t) (9.81*(ahrs.R_Matrix_Pos[0][1]* imu.acc_g[0] +
//					ahrs.R_Matrix_Pos[1][1] * imu.acc_g[1] +
//			 ahrs.R_Matrix_Pos[2][1] * imu.acc_g[2]));
//

//
//
//
//		NED[2]=	(float32_t) (9.81*( (ahrs.R_Matrix_Pos[0][2]* imu.acc_g[0] +
//					ahrs.R_Matrix_Pos[1][2] * imu.acc_g[1]+
//			 ahrs.R_Matrix_Pos[2][2] * imu.acc_g[2]) -1 ));
//
//
////		NED[0] = (float32_t) (9.81)*(imu.acc_g[0]);
////		NED[1] = (float32_t) (9.81)*(imu.acc_g[1]);
////		NED[2]=	(float32_t) (9.81)*(imu.acc_g[2] -1);
//
//

//
//	float psi_rad = M_PI*state.euler[2]/180;
//
//	float cp = cos(psi_rad);
//	float sp = sin(psi_rad);
//
//	float INERTIAL[3];
//
//	INERTIAL[0] = cp*NED[0] - sp*NED[1];
//	INERTIAL[1] = sp*NED[0] + cp*NED[1];
//	INERTIAL[2] = NED[2];

//	state.roomInertial[0] = -INERTIAL[0];
//	state.roomInertial[1] = -INERTIAL[1];
//	state.roomInertial[2] = INERTIAL[2];


//	EKF_X.measured_acceleration = state.roomInertial[0];
//			EKF_Y.measured_acceleration = state.roomInertial[1];
//					EKF_Z.measured_acceleration = state.roomInertial[2];
//

	float accel_EKFinput[3];




//	state.UWB_XY[0] = Xpdw[1];
//	state.UWB_XY[1] = Xpdw[2];


	if ((state.UWB_XY[2] > 1.1) && (throttle > 600) && (armed_drone) && (!initYaw) ){
		initYaw = true;
	}
		if ((rtos_on==1)&&(!initYaw)){
		YawWaitTime = osKernelSysTick();
		}
	//	psi_init = ahrs.psi_rad;



	if (!armed_drone) {
		initYaw = false;
		YawOn = false;
	}

	if ((collect_setpoint == true)) {


		state.UWB_setpoint[0] = 0; //set point for drone to go to
	//	state.UWB_setpoint[0] = state.UWB_XY[0];

		state.UWB_setpoint[1] = 0.4;
	//	state.UWB_setpoint[1] = state.UWB_XY[1];


		/*
		 * Prevents a height SP too close to floor/obstacles
		 * Too close to ceiling
		 */
		if (state.UWB_XY[2] < 1.3) {
			state.UWB_setpoint[2] = 1.3;
		} else if ( state.UWB_XY[2] > 2) {
			state.UWB_setpoint[2] = 2;
		} else {
			state.UWB_setpoint[2] = state.UWB_XY[2];
		}
	//	state.UWB_setpoint[2] = 1.6;





		state.ZvelSetPoint = 0;


		uwbtime = HAL_GetTick();
		collect_setpoint = false;
		setpoint_collected = true;

		/*
		 * Prevents a hover throttle low enough to end flight by falling to floor
		 * Prevents rising to close to the ceiling
		 */
		if (throttle > throttleHover) {
			throttle = throttleHover;
		} else if (throttle < throttleMin) {
			throttle = throttleMin;
		}

		Z_sum = 0;
		ZAccelSum = 0;

		XYsum[0] = 0;
		XYsum[1] = 0;

		throttleUWB_setP = throttle;
		state.initZ = state.roomInertial[2];
		angleR[0] = 0;
		angleR[1] = 0;
		angleR[2] = 0;
//		psi_init = ahrs.psi_rad;
		psi_init = M_PI/2 - 35*M_PI/180;


		esum[0] = 0;
		esum[1] = 0;
		esum[2] = 0;

		PosZSum = 0;

		VelErrorPrev[0] = state.UWB_VXY[0];
		VelErrorPrev[1] = state.UWB_VXY[1];
		ZaccelPrev = state.roomInertialAvg[2]-1;


		trim[0] = r[0];
		trim[1] = r[1];

//		soft_reset_kalman(&EKF_X);
//		soft_reset_kalman(&EKF_Y);
//		soft_reset_kalman(&EKF_Z);

		//initYaw = true;



	}






#endif

}

void Kalman_X(void) {

#if defined(HILuseUWB)

	EKF_X.measured_acceleration = (float32_t)state.HIL_UWBA[0];
	EKF_X.measured_AGL = (float32_t)state.HIL_UWBP[0];
#endif

	kalman_step(&EKF_X);

	state.uwb_ekf_x[0] = (float) EKF_X.x_bar_data[0];
	state.uwb_ekf_x[1] = (float) EKF_X.x_bar_data[1];
	state.uwb_ekf_x[2] = (float) (EKF_X.measured_acceleration
			+ EKF_X.x_bar_data[2]);

}

void Kalman_Y(void) {

#if defined(HILuseUWB)

	EKF_Y.measured_acceleration = (float32_t)state.HIL_UWBA[1];
	EKF_Y.measured_AGL = (float32_t)state.HIL_UWBP[1];
#endif

	kalman_step(&EKF_Y);

	state.uwb_ekf_y[0] = (float) EKF_Y.x_bar_data[0];
	state.uwb_ekf_y[1] = (float) EKF_Y.x_bar_data[1];
	state.uwb_ekf_y[2] = (float) (EKF_Y.measured_acceleration
			+ EKF_Y.x_bar_data[2]);

}

void Kalman_Z(void) {

#if defined(HILuseUWB)

	EKF_Z.measured_acceleration = (float32_t)state.HIL_UWBA[1];
	EKF_Z.measured_AGL = (float32_t)state.HIL_UWBP[1];
#endif

	kalman_step(&EKF_Z);

	state.uwb_ekf_z[0] = (float) EKF_Z.x_bar_data[0];
	state.uwb_ekf_z[1] = (float) EKF_Z.x_bar_data[1];
	state.uwb_ekf_z[2] = (float) (EKF_Z.measured_acceleration
			+ EKF_Z.x_bar_data[2]);

}


/*
 * GPS
 *
 */

int countspsec = 0;
int countsGPS = 0;

extern uint32_t Timer;
void GNSS_READ(void) {

//	if ((HAL_GetTick() - Timer) > 1000) {

//		GNSS_GetUniqID(&GNSS_Handle);
//		GNSS_ParseBuffer(&GNSS_Handle);
//		osDelay(250);
		GNSS_GetPVTData(&GNSS_Handle);
		GNSS_ParseBuffer(&GNSS_Handle);
		osDelay(250);
		GNSS_SetMode(&GNSS_Handle, Stationary);
	//	osDelay(250);

		countsGPS++;

		float course;
		float gSpeed;
		float fLon;
		float fLat;

		state.latP = GNSS_Handle.fLat;
		state.lonP = GNSS_Handle.fLon;
		state.latVel = GNSS_Handle.velN;
		state.lonVel = GNSS_Handle.velE;
		state.hVel = GNSS_Handle.velD;

		state.GNSSfixType = GNSS_Handle.fixType;

		Timer = HAL_GetTick();

//	}

}


/*
 *
 * MAGNETOMETER
 *
 */

extern I2C_HandleTypeDef hi2c2;

void MAG_Poll(void) {

	if (HAL_GPIO_ReadPin(I2C2_DRDY_GPIO_Port, I2C2_DRDY_Pin) == GPIO_PIN_SET) {
		LIS3MDL_ReadMag(&mag, &hi2c2);

		//	state.mag_counter++;
	}

}

//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//
//	if (huart->Instance == USART1) {
//
//		//if (HAL_UART_Receive_IT(&huart1, (uint8_t *) RxBuf_uart, RxBuf_SIZE) ==  HAL_OK)
//		//   {
//		//handle success case here
////	memset(UART_Buffer, 0, sizeof(UART_Buffer));
//		// sprintf(UART_Buffer, "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f",0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f);
//
//		// HAL_UART_Receive_IT(&huart1 , rxBuffer, sizeof(rxBuffer));
//		// HAL_UART_Receive_IT(&huart6 , (uint8_t*) &UART_Buffer, sizeof(UART_Buffer));
//
////	HAL_UARTEx_ReceiveToIdle_DMA(&huart6 , (uint8_t*) &UART_Buffer, sizeof(UART_Buffer));
//
//		oldPos = newPos;  // Update the last position before copying new data
//
//		/* If the data in large and it is about to exceed the buffer size, we have to route it to the start of the buffer
//		 * This is to maintain the circular buffer
//		 * The old data in the main buffer will be overlapped
//		 */
//		if (oldPos + Size > MainBuf_SIZE) // If the current position + new data size is greater than the main buffer
//		{
//			uint16_t datatocopy = MainBuf_SIZE - oldPos; // find out how much space is left in the main buffer
//			memcpy((uint8_t*) MainBuf + oldPos, RxBuf_uart, datatocopy); // copy data in that remaining space
//
//			oldPos = 0;  // point to the start of the buffer
//			memcpy((uint8_t*) MainBuf, (uint8_t*) RxBuf_uart + datatocopy,
//					(Size - datatocopy));  // copy the remaining data
//			newPos = (Size - datatocopy);  // update the position
//		}
//
//		/* if the current position + new data size is less than the main buffer
//		 * we will simply copy the data into the buffer and update the position
//		 */
//		else {
//			memcpy((uint8_t*) MainBuf + oldPos, RxBuf_uart, Size);
//			newPos = Size + oldPos;
//		}
//
//		/* start the DMA again */
//		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) RxBuf_uart,
//		RxBuf_SIZE);
//		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//
//		for (var = 0; var <= MainBuf_SIZE; ++var) {
//			if ((MainBuf[var] == '$') && (MainBuf[var + 1] == 'G')) { //use the payload length byte to speed the for loop search
//				for (end = var; end < var + 60; end++) {
//					if ((MainBuf[end] == '\r') && (MainBuf[end + 1] == '\n')) {
//						end++;
//						break;
//					}
//				}
//
//// need to perform a check sum to ensure 'end' corresponds to \r for the '$' that began the message
//				memset(UART_Buffer, 0, sizeof(UART_Buffer));
//				memcpy(UART_Buffer, &MainBuf[var + 3], end - var - 2);
//				// Null-terminate the uart_buffer to make it a valid string
//				UART_Buffer[end - var - 2] = '\0';
//
//				sscanf((char*) UART_Buffer, "%f,%f,%f,%f,%f,%f",
//						&state.positionUWB[0], &state.positionUWB[1],
//						&state.positionUWB[2], &state.positionUWB[3],
//						&state.positionUWB[4], &state.positionUWB[5]);
//
//				sscanf((char*) UART_Buffer, "%f,%f,%f,%f,%f,%f", &b[0], &b[1],
//						&b[2], &b[3], &b[4], &b[5]);
//
//				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//				HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
//				char posChar[150];
//				memset(posChar, 0, sizeof(posChar));
//				sprintf(posChar, "%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\r\n",
//						state.positionUWB[0], state.positionUWB[1],
//						state.positionUWB[2], state.positionUWB[3],
//						state.positionUWB[4], state.positionUWB[5]);
//
//				dt_UWB = HAL_GetTick() - uwbT0;
//
//				uwbT0 = HAL_GetTick();
//				if (state.uwbinit == false) {
//
//					PositionComp();
//				}
//
//				arm_status status;
//
//				for (int i = 0; i < anchorlen; i++) {
//					dataUWB[i] = (state.positionUWB[i] * state.positionUWB[i])
//							- anchorsquare[i];
//				}
//
//				arm_mat_init_f32(&matb, anchorlen, 1, (float32_t*) &dataUWB); //Initialise B matrix
//
//#if defined(UseMatrixW)
//	// Xpdw = inv(Transpose(A) * C * A) * Transpose(A) * C * b
//	arm_mat_mult_f32(&matATCA_inv, &matb, &positionresult);
//#else
//				status = arm_mat_mult_f32(&pinvA, &matb, &positionresult);
//
//				break;
//
//			}
//		}
//
//		/*} else
//		 {
//		 int fgfff = 0;
//
//		 HAL_UART_AbortReceive(&huart1);
//		 HAL_UART_Receive_IT(&huart1, (uint8_t *) RxBuf_uart, RxBuf_SIZE);
//		 }*/
//
//	}
//
//}

