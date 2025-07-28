/*
 * EKF_3D.h
 *
 *  Created on: Sep 3, 2023
 *      Author: laure
 */

#ifndef INC_EKF_3D_H_
#define INC_EKF_3D_H_


#include "ICM42688.h"
#include "LIS3MDL.h"

#include "arm_math.h"


#define DECIMATION_factor 1

typedef struct {

	float phi_rad;
	float theta_rad;
	float psi_rad;

	float phi_deg;
	float theta_deg;
	float psi_deg;
	float rho;
	float chi;

	float yaw_comp;
	float yaw_gyro;
	float yaw_mag;
	float yaw_alpha;


float theta_degnon;
float phi_degnon;
	float xk[12];
	float xk1[12];
	float fs;
	float fs2;
	float delta_d[3];
	float gyro_offset[3]; //rad/s
	float del_q[4]; //delta_rad in quat form
	float dq; //product (PI) of del_q
	float q1[4]; //Current Quat
	float q0[4]; //Previous Quat

	float g_GYRO[3]; //gravity estimate from gyro
	float m_GYRO[3]; //mag estimate from gyro
	float linAccelPrior[3];
	float gAccel[3];
	float R_Matrix[3][3];
	float R_Matrix_Pos[3][3];
	float m[3]; //magnetic vector estimate
	float z[6];
	float H[6][12];
	float P0[12][12];
	float S[6][6];
	float S_36[36]; //36*1 matrix of S[6][6]
	float R[6][6];
	float Q[12][12];

	float thk[3]; //orientation error vector
	float bk[3]; //gyroscope zero angle rate bias vector
	float ak[3]; //acceleration error vector
	float dk[3]; //magnetic disturbance error vector

	float linAccelPrior_K0[3];
	float gyro_offset_K0[3]; // rad/s

	float angular_velocity[3];
	float vector[3];
	float inclination;
	float m_ErrorNED[3];

	float gyro_data[3][100];
	bool calculating; //flag to stop DMA overwriting gyro buffer while calculating

	int factor;
	float MAG_strength;
	bool tf; //Compares mag error and strength for jamming detection

	float kappa;
	float kappa2;

	int Num1;
	float linAccel[3];


}AHRS0;

void K_MATRIX(AHRS0 *kal); //Finds the Kalman Gain
void P1_MATRIX(AHRS0 *kal); //Finds the P1 Matrix
void MAGNETOMETER_CORRECT(AHRS0 *kal); //Checks the mag field
void UPDATE_mag_VECTOR(AHRS0 *kal, LIS3MDL_t *mag); //Updates the mag vector
void EULER(AHRS0 *kal, LIS3MDL_t *mag); //Converts quaternion to euler angle in radians
void MATRIX_Q(AHRS0 *kal); //Finds the error covariance Q that becomes P0 for next iteration
void POSTERIORI_ERROR(AHRS0 *kal); //Calculates the new states
void ORIENTATION_Est(AHRS0 *kal, ICM42688 *imu); //Calculates the orientation from new state estiamtes
void ERROR_MODEL(AHRS0 *kal, LIS3MDL_t *mag); //Z model based on gravity and mag vector from gyroscope

void INIT_3D(AHRS0 *kal, ICM42688 *imu,LIS3MDL_t *mag,float m0[3], float a0[3] );
void PREDICT_3D(AHRS0 *kal, ICM42688 *imu);
void UPDATE_3D(AHRS0 *kal, ICM42688 *imu, LIS3MDL_t *mag);

void Quaternion_q1(AHRS0 *kal); //finds q1 from state estimate and q0
void euler_quat(AHRS0 *kal); //find q0 from q1 and
void MATRIX_q0(AHRS0 *kal); //finds the rotation matrix from q0
//void QUAT_q0(AHRS0 *kal);



#endif /* INC_EKF_3D_H_ */
