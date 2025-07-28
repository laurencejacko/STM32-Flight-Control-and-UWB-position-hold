/*
 * EKF_3D.c
 *
 *  Created on: Sep 3, 2023
 *      Author: laure
 */


#include "EKF_3D.h"
#include "ICM42688.h"
#include "LIS3MDL.h"

#include "arm_math.h"
#include "math_helper.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;

extern float PitchB, RollB;

#define GYRO_DRIFT_Beta 1e-12f//Gyro offset drift^2
#define LINEAR_NOISE_xi 4e-5f //Linear acceleration drift (20e-3)^2 changed from 4e-8
#define MAG_NOISE_gamma 0.5f //Magnetic disturbance drift (TAKEN FROM NXP)


#define MAG_FACTOR_sigma 0.5f //Magnetic disturb factor
#define LINEAR_FACTOR_nu 0.5f //Linear acceleration decay factor


#define GYRO_NOISE_eta  0.00784f //Gyro Noise (0.028)^2 == (rms)^2 -. changed frm 0.0078
#define ACCEL_SENSOR_NOISE 5e-6f //Sensor Noise of Accelerometer (rms^2) (m/s)^2 (TAKEN FROM DATA PROCESSED IN MATALB) change from 5e-6
#define MAGNET_SENSOR_NOISE_XY 0.5f //Sensor noise of Magnetometer (rms^2)  (uT)^2 ( "" )
#define MAGNET_SENSOR_NOISE_Z 0.5f //Sensor noise of Magnetometer (rms^2)  (uT)^2 ( "" )

#define corrupt_quat 0.001f
#define smallq0 0.01f

#define cosdeltaMAX 0.4226183F
#define sindeltaMAX 0.9063078F
//int N = DECIMATION_factor;

float q0[4];
float q1[4];

float angles[3] = {0.87266f, 0.34907f, 1.2217f};

float accel_noise;
float mag_noise;

float Deg2Rad = M_PI/180;
float Rad2Deg = 180/M_PI;

bool yawcomp_init = false;
float32_t R[6][6]; //R
float32_t SI[6][6]; //S^-1
float32_t S[6][6]; //S
float32_t H[6][12]; //H
float32_t Ht[12][6];  //H (transposed)
float32_t P0[12][12]; //Previous Predicted error covariance
float32_t P1[12][12]; //Updated error covariance
float32_t K[12][6]; //Kalman Gain

float32_t K_MAG[3][6]; // == K(10:12,:)
float32_t M_ERROR[3];

float32_t SI_f32[6][6];

float32_t zg[3];
float32_t zm[3];

arm_matrix_instance_f32 R_M;
arm_matrix_instance_f32 SI_M;
arm_matrix_instance_f32 S_M;
arm_matrix_instance_f32 H_M;
arm_matrix_instance_f32 Ht_M;
arm_matrix_instance_f32 P0_M;
arm_matrix_instance_f32 P1_M;
arm_matrix_instance_f32 K_M;

arm_matrix_instance_f32 K_MAG_M;
arm_matrix_instance_f32 z_M;
arm_matrix_instance_f32 zg_M;
arm_matrix_instance_f32 mERROR_M;



arm_matrix_instance_f32 x_12M;
arm_matrix_instance_f32 x_9M;

float32_t x_12[12];
float32_t x_9[9];

arm_matrix_instance_f32 temp;
arm_matrix_instance_f32 temp1;
arm_matrix_instance_f32 temp2;
arm_matrix_instance_f32 temp3;
arm_matrix_instance_f32 temp4;
float DeltaP1;
/*
float32_t tempM[6][12];
float32_t temp1M[6][6];
float32_t temp2M[12][6];
float32_t temp3M[6][6];
float32_t temp4M[12][6];
*/
float32_t tempM[12][6];


	float32_t temp1M[6][6];
	float32_t temp2M[12][6];
	float32_t temp3M[12][12];
	float32_t temp4M[6][12];


	arm_matrix_instance_f32 SI_M1;
	arm_matrix_instance_f32 S_M1;

	arm_matrix_instance_f32 K_M1;
	extern float gyro_data[3][100];

	float sum_gyr[3] = {0,0,0};

#include "include_all.h"
extern struct MagSensor thisMag;					// this magnetometer


float MaxGyroOffset = 7;
float MinGyroOffset = -7;
float MaxGyroOffsetChange;



void K_MATRIX(AHRS0 *kal){
/*
 * Calculates the P1 matrix by calculating,
 *
 * Ht = Observation model (transposed)
 * S = (Innovation Covariance)
 * S^-1
 * K = (Kalman Gain)
 */


	arm_status status;


	/*float S[6][6];
	float SI[6][6];

	float HP[6][12]; //6 x 12 = H*P0
	float PHt[12][6]; //12 x 6 = P0*Ht

	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 12; j++){

			HP[i][j] = 0;

			for (int k = 0; k < 12; k++) {
				//H = 6x12, P0 = 12x12
				HP[i][j] += kal->H[i][k] * kal->Q[k][j];
			}
		}
	}
	for (int i = 0; i < 12; i++){
		for (int j = 0; j < 6; j++){

			PHt[i][j] = 0;

			for (int k = 0; k < 12; k++) {
				//P0 = 12x12, Ht = 12x6
				PHt[i][j] += kal->Q[i][k] * kal->H[j][k];
			}
		}
	}

	float HPHt[6][6]; //H*P0*Ht == HP*Ht

	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 6; j++){
			HPHt[i][j] = 0;

			S[i][j] = 0;
			SI[i][j] = 0;

			for (int k = 0; k < 12; k++){

				HPHt[i][j] += HP[i][k] * kal->H[j][k];

			}
		}
	}

	for (int i = 0; i < 6; i++){
		for (int j = 0; j < 6; j++){
			S[i][j] = kal->R[i][j] +  HPHt[i][j];
		}
	}

	arm_mat_init_f32(&S_M1, 6, 6, (float32_t *)&S[0][0]); //Initialise S Matrix
	arm_mat_init_f32(&SI_M1, 6, 6, (float32_t *)&SI[0][0]); //Initialise S^-1 Matrix

	status = arm_mat_inverse_f32(&S_M1, &SI_M1); //Calculate S^-1 Matrix


	float K[12][6];

	for (int i = 0; i < 12; i++){
			for (int j = 0; j < 6; j++){

				K[i][j] = 0;

				for (int k = 0; k < 6; k++) {
					//PHt = 12x6, SI = 6x6
					K[i][j] += PHt[i][k] * SI[k][j];
				}
			}
		}

	float KHP[12][12];

	for (int i = 0; i < 12; i++){
			for (int j = 0; j < 12; j++){

				KHP[i][j] = 0;

				for (int k = 0; k < 6; k++) {
					//K = 12x6, HP = 6x12
					KHP[i][j] += K[i][k] * HP[k][j];
				}

				P1[i][j] = kal->Q[i][j] - KHP[i][j];

			}
		}
}*/


/*
	status = arm_mat_mult_f32(&H_M, &P0_M, &temp); //Calc H*P0

	status = arm_mat_trans_f32(&H_M, &Ht_M); //Calc H (Transpose)

	status = arm_mat_mult_f32(&temp, &Ht_M, &temp1); //Calc H*P0*Ht

	status = arm_mat_add_f32(&R_M, &temp1, &S_M); // S = R + H*P0*Ht

	status = arm_mat_inverse_f32(&S_M, &SI_M); //Calculate S^-1 Matrix


	//Calculate Kalman Gain K = P0*Ht*S^-1
	status = arm_mat_mult_f32(&P0_M, &Ht_M, &temp2); //Calc P0*Ht
	status = arm_mat_mult_f32(&temp2, &SI_M, &K_M); //Calc K = P0*Ht*S^-1

	//Update Error Covariance Matrix P1 = P0 - K*H*P0

	status = arm_mat_mult_f32(&K_M, &H_M, &temp3); //Calc K*H
	status = arm_mat_mult_f32(&temp3, &P0_M, &temp4); //Calc K*H*P0


	status = arm_mat_sub_f32(&P0_M, &temp4, &P1_M); // Calculate P1 = P0 - K*H*P0
*/


	accel_noise = LINEAR_NOISE_xi + ACCEL_SENSOR_NOISE + Deg2Rad*Deg2Rad*(kal->kappa2)*(GYRO_DRIFT_Beta+ GYRO_NOISE_eta);
	mag_noise = MAG_NOISE_gamma + MAGNET_SENSOR_NOISE_XY + Deg2Rad*Deg2Rad*(kal->kappa2)*(GYRO_DRIFT_Beta+ GYRO_NOISE_eta);


	for (int i = 0; i < 3; i++){
		kal->R[i][i] = accel_noise;
		kal->R[i+3][i+3] = mag_noise;
	}

//	mag_noise = MAG_NOISE_gamma + MAGNET_SENSOR_NOISE_Z + Deg2Rad*Deg2Rad*(kal->kappa2)*(GYRO_DRIFT_Beta+ GYRO_NOISE_eta);

kal->R[5][5] = mag_noise;


	  //H*P0
	arm_mat_init_f32(&temp, 12, 6, &tempM[0][0]); //Q * Ht

	arm_mat_init_f32(&H_M, 6, 12, (float32_t *)&kal->H[0][0]);
	arm_mat_init_f32(&Ht_M, 12, 6, &Ht[0][0]); //Initialise H (Transpose)
	arm_mat_init_f32(&P0_M, 12, 12, (float32_t *)&kal->Q[0][0]);

	status = arm_mat_trans_f32(&H_M, &Ht_M); //Calc H (Transpose)
	status = arm_mat_mult_f32(&P0_M, &Ht_M, &temp); //Calc (P0*Ht)


	arm_mat_init_f32(&temp1, 6, 6, &temp1M[0][0]); //Initialise Temp1 variable = H*P0*Ht
	arm_mat_init_f32(&R_M, 6, 6, (float32_t *)&kal->R[0][0]); //Initialise R Matrix


	status = arm_mat_mult_f32(&H_M, &temp, &temp1); //Calc H*(P0*Ht)

	for (int i =0; i < 6; i++) {
		for(int j= 0; j < 6; j++){
			S[i][j] = temp1M[i][j] + kal->R[i][j];
		}
	}

//	for (int i =0; i < 6; i++) {
//		for(int j= 0; j < i; j++){
//			S[i][j] = S[j][i];
//		}
//	}


	arm_mat_init_f32(&S_M, 6, 6, &S[0][0]); //Initialise S Matrix


	//status = arm_mat_add_f32(&R_M, &temp1, &S_M); // S = R + (H*P0*Ht)

	arm_mat_init_f32(&SI_M, 6, 6, &SI[0][0]); //Initialise S^-1 Matrix
	status = arm_mat_inverse_f32(&S_M, &SI_M); //Calculate S^-1 Matrix


	//Calculate Kalman Gain K = P0*Ht*S^-1
	arm_mat_init_f32(&K_M, 12, 6, &K[0][0]); //Initialise K Matrix
	status = arm_mat_mult_f32(&temp, &SI_M, &K_M); //Calc (P0*Ht)*(S^-1)



}

void P1_MATRIX(AHRS0 *kal){


	//END HERE AND COMPUTE STATE BEFORE UPDATING COVARIANCE MATRIX Q

	arm_status status;

	//Update Error Covariance Matrix P1 = P0 - K*H*P0
	arm_mat_init_f32(&temp3, 12, 12, &temp3M[0][0]); //Initialise temp3 = K*H
	arm_mat_init_f32(&temp4, 6, 12, &temp4M[0][0]); //Initialise temp4 = K*H*P0

	status = arm_mat_mult_f32(&H_M, &P0_M, &temp4); //Calc H*P0
	status = arm_mat_mult_f32(&K_M, &temp4, &temp3); //Calc K*(H*P0)

	arm_mat_init_f32(&P1_M, 12, 12, &P1[0][0]); //Initialise P1
	status = arm_mat_sub_f32(&P0_M, &temp3, &P1_M);


	/*for (int i = 0; i < 12; i++){
		for(int j =0; j < 12; j++){
			P1[i][j] = P0
		}
	}
*/

//	status = arm_mat_sub_f32(&P0_M, &temp4, &P1_M); // Calculate P1 = P0 - (K*H*P0)




}









void MAGNETOMETER_CORRECT(AHRS0 *kal) {

	arm_status status;



	float m_TOT = 0;

	for (int i = 0; i < 3; i++){
		m_TOT += kal->dk[i]*kal->dk[i];
	}

	float expected = 4*kal->MAG_strength*kal->MAG_strength;

	if (m_TOT > expected)
		kal->tf = true; //True == ignore mag terms (too much error)
	 else
		kal->tf = false; //False == include mag terms (suitable amount of error)
}


void UPDATE_mag_VECTOR(AHRS0 *kal, LIS3MDL_t *mag) {
	//IF kal->tf == false;

	for (int i = 0; i < 3; i++){
		kal->m_ErrorNED[i] = 0;
		for (int j = 0; j < 3; j++){
			kal->m_ErrorNED[i] += kal->R_Matrix_Pos[j][i]*kal->dk[j];
		}
	}

//	kal->m_ErrorNED[0] = kal->R_Matrix_Pos[0][0]*kal->dk[0] +  kal->R_Matrix_Pos[1][0]*kal->dk[1] + kal->R_Matrix_Pos[2][0]*kal->dk[2];
//	kal->m_ErrorNED[2] = kal->R_Matrix_Pos[0][2]*kal->dk[0] +  kal->R_Matrix_Pos[1][2]*kal->dk[1] + kal->R_Matrix_Pos[2][2]*kal->dk[2];

	kal->m_ErrorNED[1] = 0;

	float MAG[3];


	float fopp = kal->m[2] - kal->m_ErrorNED[2];
	float fadj = kal->m[0] - kal->m_ErrorNED[0];

	if (fadj < 0){
		fadj = 0;
	}
	float fhyp = sqrtf(fopp*fopp + fadj*fadj);


	float sindelta, cosdelta;
	if (fhyp != 0){
		float ftmp = 1.0F / fhyp;
		sindelta = fopp * ftmp;
		cosdelta = fadj * ftmp;

	if (sindelta > sindeltaMAX){

		sindelta = sindeltaMAX;
		cosdelta = cosdeltaMAX;

	} else if (sindelta < -sindeltaMAX){

		sindelta = -sindeltaMAX;
		cosdelta = cosdeltaMAX;
	}

	 DeltaP1 = Rad2Deg*asin(sindelta);

	kal->m[0] = kal->MAG_strength*cosdelta;
	kal->m[1] = 0;
	kal->m[2] = kal->MAG_strength*sindelta;

	}
//	kal->inclination = atan2(MAG[2], MAG[1]);//MATLAB

	//kal->m[0] = kal->MAG_strength*cos(kal->inclination);
	//kal->m[1] = 0;
	//kal->m[2] = kal->MAG_strength*sin(kal->inclination);


}

void EULER(AHRS0 *kal, LIS3MDL_t *mag){

/*	float qw = kal->q1[0];
	float qx = kal->q1[1];
	float qy = kal->q1[2];
	float qz = kal->q1[3];


	float x = 2*(qw*qy - qx*qz);

	kal->phi_rad = atan2( (2*(qw*qx + qy*qz)) , (1-2*(qx*qx - qy*qy)) );
	kal->theta_rad = -M_PI*0.5 + 2*atan2( sqrt(1+x) , sqrt(1-x) );
	kal->psi_rad = atan2( (2*(qw*qz + qx*qy)) , (1-2*(qy*qy + qz*qz)));*/

/*	if (kal->R_Matrix_Pos[0][2] >= 1.0f) {
		kal->R_Matrix_Pos[0][2] = 1.0f;
	} else if (kal->R_Matrix_Pos[0][2] <= -1.0f) {
		kal->R_Matrix_Pos[0][2] = -1.0f;
	}

	kal->theta_deg = Rad2Deg * asin(-kal->R_Matrix_Pos[0][2]);
	kal->phi_deg = Rad2Deg * atan2(kal->R_Matrix_Pos[1][2], kal->R_Matrix_Pos[2][2]);





	if (kal->phi_deg == 180.0f){
		kal->phi_deg = -180.0f;
	}

	if (kal->theta_deg == 90.0f){
		kal->psi_deg = Rad2Deg * atan2(kal->R_Matrix_Pos[2][1] , kal->R_Matrix_Pos[1][1]) + kal->phi_deg;
	} else if (kal->theta_deg == -90.0f){
		kal->psi_deg = Rad2Deg * atan2(-kal->R_Matrix_Pos[2][1], kal->R_Matrix_Pos[1][1]) - kal->phi_deg;
	} else {
		kal->psi_deg = Rad2Deg * atan2(kal->R_Matrix_Pos[0][1], kal->R_Matrix_Pos[0][0]);
	}

	if (kal->psi_deg < 0.0f){
		kal->psi_deg += 360.0f;
	}

	if (kal->psi_deg >= 360.0f){
		kal->psi_deg = 0.0f;
	}
	*/


	if (kal->R_Matrix_Pos[2][0] >= 1.0f) {
		kal->R_Matrix_Pos[2][0] = 1.0f;
	} else if (kal->R_Matrix_Pos[2][0] <= -1.0f) {
		kal->R_Matrix_Pos[2][0] = -1.0f;
	}

//	RollB = 1;
//	PitchB = 1;
	kal->theta_deg = ((1- RollB) * kal->theta_deg) + (RollB * (Rad2Deg * asin(-kal->R_Matrix_Pos[0][2])));

	kal->phi_deg = ((1-PitchB) *kal->phi_deg) + (PitchB *(Rad2Deg * atan2(-kal->R_Matrix_Pos[1][2], kal->R_Matrix_Pos[2][2])));



	//float y = kal->R_Matrix_Pos[0][2];
//	kal->theta_deg = Rad2Deg * atan2(y, sqrt(1 - y*y));



	if (kal->phi_deg == 180.0f){
		kal->phi_deg = -180.0f;
	}

	if (kal->theta_deg == 90.0f){
		kal->psi_deg = Rad2Deg * atan2(kal->R_Matrix_Pos[2][1] , kal->R_Matrix_Pos[1][1]) + kal->phi_deg;
	} else if (kal->theta_deg == -90.0f){
		kal->psi_deg = Rad2Deg * atan2(-kal->R_Matrix_Pos[2][1], kal->R_Matrix_Pos[1][1]) - kal->phi_deg;
	} else {
		kal->psi_deg = Rad2Deg * atan2(kal->R_Matrix_Pos[0][1], kal->R_Matrix_Pos[0][0]);

	}

	if (kal->psi_deg < 0.0f){
			kal->psi_deg += 360.0f;
		}

		if (kal->psi_deg >= 360.0f){
			kal->psi_deg = 0.0f;
		}


	kal->psi_rad = kal->psi_deg * Deg2Rad;
	//kal->psi_rad = Deg2Rad * kal->psi_deg;


	float cr = cos(kal->theta_deg*Deg2Rad);
	float sr = sin(kal->theta_deg*Deg2Rad);

	float ct=  cos(kal->phi_deg*Deg2Rad);
	float st = sin(kal->phi_deg*Deg2Rad);

//	float b[3]= {0,0,0};
	//float tempMat[3][3] =

//	b[0] = cr,sr*st, sr*ct},
//
//			{0, ct, -st },
//
//			{-sr, cr*st, cr*ct}};
//
float m[3] = {0,0,0};
memcpy(m, thisMag.fBcFast, sizeof(thisMag.fBcFast));
	kal->yaw_mag= Rad2Deg*atan2(m[2]*st - m[1]*ct, m[0]*cr + sr*(m[1]*st +  m[2]*ct));

	if (yawcomp_init == false) {
		kal->yaw_gyro = kal->yaw_mag;
		yawcomp_init = true;
	}

	kal->yaw_comp = kal->yaw_alpha*(kal->yaw_gyro) + (1-kal->yaw_alpha)*kal->yaw_mag;

	if (kal->yaw_comp < 0.0f) {
		kal->yaw_comp += 360.0f;
	}

	if (kal->yaw_comp > 360.0) {
		kal->yaw_comp = 0.0f;
	}
/*
*/


	kal->rho = kal->psi_deg;

	/*if (kal->R_Matrix_Pos[2][2] >= 1.0f) {
			kal->R_Matrix_Pos[2][2] = 1.0f;
		} else if (kal->R_Matrix_Pos[0][2] <= -1.0f) {
			kal->R_Matrix_Pos[2][2] = -1.0f;
		}*/

	kal->chi = Rad2Deg * acos(kal->R_Matrix_Pos[2][2]);



}

void MATRIX_Q(AHRS0 *kal){

	//Predict Error Estimate Covariance = (Q)

	for (int i = 0; i < 12; i++){
		for (int j=0; j < 12; j++){
			kal->Q[i][j] = 0.0f;
		}
	}


	float kap = kal->kappa;
	float kap2 = kal->kappa2;

	for (int i = 0; i < 3; i++) {
		kal->Q[i][i] = P1[i][i] + kap2*(P1[i+3][i+3] + GYRO_DRIFT_Beta + GYRO_NOISE_eta);
		kal->Q[i+3][i+3] = P1[i+3][i+3] + GYRO_DRIFT_Beta;
		kal->Q[i+6][i+6] = LINEAR_FACTOR_nu*LINEAR_FACTOR_nu*(P1[i+6][i+6]) + LINEAR_NOISE_xi;
		kal->Q[i+9][i+9] = MAG_FACTOR_sigma*MAG_FACTOR_sigma*(P1[i+9][i+9]) + MAG_NOISE_gamma;

		kal->Q[i][i+3] = -kap*(P1[i+3][i+3] + GYRO_DRIFT_Beta);
		kal->Q[i+3][i] = -kap*(P1[i+3][i+3] + GYRO_DRIFT_Beta);
	}

	//Update P0 matrix using  Q


	for (int i = 0; i < 12; i++){
		for (int j=0; j < 12; j++){
			kal->P0[i][j] = kal->Q[i][j];
		}
	}

}

void POSTERIORI_ERROR(AHRS0 *kal){

	arm_status status;

		//arm_mat_init_f32(&x_12M, 12, 1, &x_12[0]); //Initialise R Matrix

		//status = arm_mat_mult_f32(&K_M, &z_M, &x_12M); //Calc P0*Ht


		for (int i = 0; i < 3; i++){
						kal->thk[i] = K[i][0]*zg[0] + K[i][1]*zg[1] + K[i][2]*zg[2];
						kal->bk[i] = K[i+3][0]*zg[0] + K[i+3][1]*zg[1] + K[i+3][2]*zg[2];
						kal->ak[i] = K[i+6][0]*zg[0] + K[i+6][1]*zg[1] + K[i+6][2]*zg[2];
						kal->dk[i] = K[i+9][0]*zg[0] + K[i+9][1]*zg[1] + K[i+9][2]*zg[2] + K[i+9][3]*zm[0] + K[i+9][4]*zm[1] + K[i+9][5]*zm[2];
					}


		MAGNETOMETER_CORRECT(kal);




		/*arm_mat_init_f32(&x_9M, 9, 1, &x_9[0]); //Initialise R Matrix
		arm_mat_init_f32(&zg_M, 3, 1, &zg[0]); //Initialise R Matrix

		float32_t K_1[9][3];

		for (int i = 0; i < 9; i++){
			for (int j = 0; j< 3; j++){
				K_1[i][j] = K[i][j];
			}
		}
		arm_mat_init_f32(&K_M1, 9, 3, &K_1[0][0]); //Initialise R Matrix


		status = arm_mat_mult_f32(&K_M1, &zg_M, &x_9M); //Calc P0*Ht*/


	if (kal->tf == false){
		for (int i = 0; i < 3; i++){
			kal->thk[i] += K[i][3]*zm[0] + K[i][4]*zm[1] + K[i][5]*zm[2];
			kal->bk[i] += K[i+3][3]*zm[0] + K[i+3][4]*zm[1] + K[i+3][5]*zm[2];
			kal->ak[i] += K[i+6][3]*zm[0] + K[i+6][4]*zm[1] + K[i+6][5]*zm[2];
		}
	}




}

void ORIENTATION_Est(AHRS0 *kal, ICM42688 *imu){





	for (int i = 0; i < 3;i++){

	kal->linAccelPrior[i] = kal->linAccelPrior_K0[i] - kal->ak[i];



	if (kal->bk[i] > MaxGyroOffsetChange) {
		kal->gyro_offset[i] -= MaxGyroOffsetChange;

	} else if (kal->bk[i] < -MaxGyroOffsetChange) {
		kal->gyro_offset[i] += MaxGyroOffsetChange;

	} else {

		kal->gyro_offset[i] -= kal->bk[i];

	}

		if (kal->gyro_offset[i] > MaxGyroOffset) kal->gyro_offset[i] = MaxGyroOffset;
		if (kal->gyro_offset[i] < MinGyroOffset) kal->gyro_offset[i] = MinGyroOffset;





//	kal->gyro_offset[i] = kal->gyro_offset[i] - (kal->bk[i]);

	if (kal->factor > 0) {
	sum_gyr[i] /= (kal->factor);
	}


	}


	kal->angular_velocity[0] = sum_gyr[0] - kal->gyro_offset[0];
	kal->angular_velocity[1] = sum_gyr[1] - kal->gyro_offset[1];
	kal->angular_velocity[2] = sum_gyr[2] - kal->gyro_offset[2];





//kal->angular_velocity[i] = imu->gyr_dps[i] - kal->gyro_offset[i];



	kal->linAccel[0] = kal->R_Matrix_Pos[0][0]*imu->acc_g[0]  + kal->R_Matrix_Pos[1][0]*imu->acc_g[1]  + kal->R_Matrix_Pos[2][0]*imu->acc_g[2];
	kal->linAccel[1] = kal->R_Matrix_Pos[0][1]*imu->acc_g[0]  + kal->R_Matrix_Pos[1][1]*imu->acc_g[1]  + kal->R_Matrix_Pos[2][1]*imu->acc_g[2];
	kal->linAccel[2] = kal->R_Matrix_Pos[0][2]*imu->acc_g[0]  + kal->R_Matrix_Pos[1][2]*imu->acc_g[1]  + kal->R_Matrix_Pos[2][2]*imu->acc_g[2];


}
void INIT_3D(AHRS0 *kal, ICM42688 *imu, LIS3MDL_t *mag, float m0[3], float a0[3] ) {


	kal->fs2 = kal->fs*kal->fs;
	kal->MAG_strength = thisMagCal.fB;

	  //H*P0
	/*
	arm_mat_init_f32(&temp, 6, 12, &tempM[0][0]);

	arm_mat_init_f32(&temp1, 6, 6, &temp1M[0][0]);


	arm_mat_init_f32(&H_M, 6, 12, (float32_t *)&kal->H[0][0]);
	arm_mat_init_f32(&P0_M, 12, 12, (float32_t *)&kal->Q[0][0]);
	arm_mat_init_f32(&Ht_M, 12, 6, &Ht[0][0]); //Initialise H (Transpose)
	arm_mat_init_f32(&R_M, 6, 6, (float32_t *)&kal->R[0][0]); //Initialise R Matrix
	arm_mat_init_f32(&S_M, 6, 6, &S[0][0]); //Initialise S Matrix
	arm_mat_init_f32(&SI_M, 6, 6, &SI[0][0]); //Initialise S^-1 Matrix
	arm_mat_init_f32(&K_M, 12, 6, &K[0][0]); //Initialise R Matrix
	arm_mat_init_f32(&temp2, 12, 6, &temp2M[0][0]); //Initialise temp2 = P0*Ht
	arm_mat_init_f32(&temp3, 12, 12, &temp3M[0][0]); //Initialise temp3 = K*H
	arm_mat_init_f32(&temp4, 12, 12, &temp4M[0][0]); //Initialise temp4 = K*H*P0
	arm_mat_init_f32(&P1_M, 12, 12, &P1[0][0]); //Initialise P1
*/


	float dt2 = 1/ kal->fs2;

float dfsdf = fabs(2E-1F );

	MaxGyroOffsetChange = sqrt(fabs(2E-3F )) / kal->fs;


	for (int i = 0; i < 3; i++){
		kal->m[i] = 0.0f;
		kal->gyro_offset[i] = 0.0f;
		//kal->R[i][i] = accel_noise;
		//kal->R[i+3][i+3] = mag_noise;
	}



	for (int i = 0; i < 12; i++){
			for (int j=0; j < 12; j++){
				kal->Q[i][j] = 0.0f;
			}
		}

	/*	for (int i = 0; i < 3; i++) {
			kal->Q[i][i] =  GYRO_DRIFT_Beta + GYRO_NOISE_eta;
			kal->Q[i+3][i+3] = GYRO_DRIFT_Beta;
			kal->Q[i+6][i+6] = LINEAR_FACTOR_nu*LINEAR_FACTOR_nu;
			kal->Q[i+9][i+9] = MAG_FACTOR_sigma*MAG_FACTOR_sigma + MAG_NOISE_gamma;
			kal->Q[i][i+3] = -1*(GYRO_DRIFT_Beta);
			kal->Q[i+3][i] = -1*(GYRO_DRIFT_Beta);
		}*/
	for (int i = 0; i < 3; i++) {  //NXP
			kal->Q[i][i] =  0.02f;
			kal->Q[i+3][i+3] = 0.25f;
			kal->Q[i+6][i+6] = 0.0001f;
			kal->Q[i+9][i+9] = 0.6f;
			kal->Q[i][i+3] = 0.0f;
			kal->Q[i+3][i] = 0.0f;
		}


		//Initialise orientation using MAG and ACCEL  (MATLAB ALGORITHM)




		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);


		float a[3]= {a0[0],a0[1],a0[2]}, m[3] = {m0[0], m0[1],m0[2]};

//mag->magBEGIN = false;
		HAL_Delay(50);
	/*	 for (int i = 0; i < sample_tot; i++) {
		 HAL_Delay(10);


	//	 while(mag->magBEGIN == false){
		  LIS3MDL_ReadMag(&mag, &hi2c2);
	//	/ }

	//	 mag->magBEGIN = false;

		 ICM42688_ReadData(imu);

		 for (int j = 0; j < 3; j++){

			 m[j] += mag->mag_raw[j];
		 }
		 a[0] += imu->acc_g[0];
		 a[1] += imu->acc_g[1];
		 a[2] = a[2] + (imu->acc_g[2] - 1);


		 }

		 for (int j = 0; j < 3; j++){
		 			 a[j] /= sample_tot;
		 			 m[j] /= sample_tot;
		 		 }

		 a[2] += 1;

*/
		kal->m[0] = m[0];
		kal->m[1] = m[1];
		kal->m[2] = m[2];


//m[0,1,2] = R[0,1,2][X]
//a[0,1,2] = R[0,1,2][Z]

		 float b[3];

		 b[0] = a[1]*m[2] - a[2]*m[1];
		 b[1] = a[2]*m[0] - a[0]*m[2];
		 b[2] = a[0]*m[1] - a[1]*m[0];

		 float c[3];

		 c[0] = a[2]*b[1] - a[1]*b[2];
		 c[1] = a[0]*b[2] - a[2]*b[0];
		 c[2] = a[1]*b[0] - a[0]*b[1];

		 float R[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

		 //Creates and normalises the rotation matrix
		 float normal[3]= {0, 0,0};

		 for (int i = 0; i<  3; i++){
			 R[i][0] = c[i];
			 R[i][1] = b[i];
			 R[i][2] = a[i];

			 normal[0] +=  c[i]*c[i];
			 normal[1] +=  b[i]*b[i];
			 normal[2] +=  a[i]*a[i];
		 }


		 for (int i = 0; i<  3; i++){

		 	 R[i][0] /= sqrt(normal[0]);
		 	 R[i][1] /= sqrt(normal[1]);
		 	 R[i][2] /= sqrt(normal[2]);


		 }


		float mag_mod = sqrt(kal->m[0]*kal->m[0] + kal->m[1]*kal->m[1] + kal->m[2]*kal->m[2]);
		float mod_a = a[0]*m[0] + a[1]*m[1] + a[2]*m[2];
		kal->inclination = Rad2Deg*asin(-mod_a / (sqrt(normal[2])*mag_mod));

		float q02;
		float recip;

		q02 = (fabs(0.25*(1+ R[0][0] + R[1][1] + R[2][2])));
		kal->q1[0] = sqrt(q02);

		if (kal->q1[0] > smallq0){
			recip = 0.25f / kal->q1[0];
			kal->q1[1] = recip*(R[1][2] - R[2][1]);
			kal->q1[2] = recip*(R[2][0] - R[0][2]);
			kal->q1[3] = recip*(R[0][1] - R[1][0]);
		} else {
			kal->q1[1] = sqrt(fabs(0.5f * (1.0f + R[0][0]) - q02));
			kal->q1[2] = sqrt(fabs(0.5f * (1.0f + R[1][1]) - q02));
			kal->q1[3] = sqrt(fabs(0.5f * (1.0f + R[2][2]) - q02));

		if ((R[1][2] - R[2][1]) < 0.0f)
			kal->q1[1] = -kal->q1[1];

		if ((R[2][0] - R[0][2]) < 0.0f)
			kal->q1[2] = -kal->q1[1];

		if ((R[0][1] - R[1][0]) < 0.0f)
			kal->q1[3] = -kal->q1[1];

		}


//DONT CHANGE THIS ORDER [1 0 2]
	//	 kal->delta_rad[1] = -asin( R[0][2] ); //theta
	//	 kal->delta_rad[0] = atan2( (R[1][2]/cos(kal->delta_rad[1])) , (R[2][2]/cos(kal->delta_rad[1])) ); //roll
	//	 kal->delta_rad[2] = atan2( (R[0][1]/cos(kal->delta_rad[1])) , (R[0][0]/cos(kal->delta_rad[1])) ); //heading


//		 kal->delta_rad[2] *= (M_PI/180);



	//	 QUAT_q0(kal); //Sets the initial quaternion using mag and accel data
	//	 kal->k0 = false;
}


void PREDICT_3D(AHRS0 *kal, ICM42688 *imu){

	/*	for (int i = 0; i < 3; i++) {
			kal->delta_d[i] = (imu->gyr_dps[i] - kal->gyro_offset[i])/(kal->fs);
		}*/


		//Testing Purposes only//////////////////////////
//	for (int i = 0; i < 3; i++) {
//			kal->delta_rad[i] = angles[i];
//	}
////////////////////////////////////////////////////////

	kal->kappa = kal->factor/kal->fs;
	kal->kappa2 = kal->kappa*kal->kappa;


		euler_quat(kal);


		MATRIX_q0(kal);



		//Estimate Gravity Vector from Orientation (kal->q0)
		for (int i =0; i <3; i++){
			kal->g_GYRO[i] = kal->R_Matrix[i][2];
		}

		//Estimate Gravity Vector from Acceleration

		for (int i = 0; i<3; i++){
			kal->linAccelPrior_K0[i] = LINEAR_FACTOR_nu*kal->linAccelPrior[i];
			kal->gAccel[i] = imu->acc_g[i] + kal->linAccelPrior_K0[i];// - kal->g_GYRO[i];
		}

		//Estimate Magnetic vector from Orientation and Previous mag estimate



		for (int i = 0; i <3; i++) {

				//kal->m[1] == 0
				kal->m_GYRO[i] = kal->R_Matrix[i][0]*kal->m[0] + kal->R_Matrix[i][2]*kal->m[2];

			//kal->m_GYRO[i] *= Deg2Rad;
		}

}


void ERROR_MODEL(AHRS0 *kal, LIS3MDL_t *mag) {

	for (int i = 0; i <3; i++) {

			zg[i] = kal->gAccel[i] - kal->g_GYRO[i];
			kal->z[i] = (float)zg[i];
		//	zm[i] = mag->mag[i] - kal->m_GYRO[i];


			zm[i] = thisMag.fBcFast[i] - kal->m_GYRO[i];
			kal->z[i+3] = (float)zm[i];
	}

}

void UPDATE_3D(AHRS0 *kal, ICM42688 *imu, LIS3MDL_t *mag){

	//Creating Observation Model (H) == function of gravity and magnetic vector computed from gyroscope orientation

	float kappa = kal->kappa;

	float gx = Deg2Rad*kal->g_GYRO[0], gy = Deg2Rad*kal->g_GYRO[1], gz = Deg2Rad*kal->g_GYRO[2];
	float mx = Deg2Rad*kal->m_GYRO[0], my = Deg2Rad*kal->m_GYRO[1], mz = Deg2Rad*kal->m_GYRO[2];

	float H0[6] = {0,   gz,   -gy, 0,      -kappa*gz, kappa*gy};
	float H1[6]= {-gz, 0,    gx,  kappa*gz,  0,      -kappa*gx};
	float H2[6] = {gy,  -gx,  0,   -kappa*gy, kappa*gx,  0};
	float H3[6] = {0,   mz,   -my, 0,      -kappa*mz, kappa*my};
	float H4[6] = {-mz, 0,    mx,  kappa*mz,  0,      -kappa*mx};
	float H5[6] = {my,  -mx,  0,   -kappa*my, kappa*mx,  0};


	for (int i=0; i< 3; i++) {
		for (int j=0; j<6;j++){
				//Sets terms H(0:2, 6:11) == 0, H(3:5, 6:11) == 0
				kal->H[i][j+6] = 0.0f;
				kal->H[i+3][j+6] = 0.0f;

				//Sets terms H(0:5, 0:5) == Correct H
				kal->H[0][j] = H0[j];
				kal->H[1][j] = H1[j];
				kal->H[2][j] = H2[j];
				kal->H[3][j] = H3[j];
				kal->H[4][j] = H4[j];
				kal->H[5][j] = H5[j];

		}
		//Sets identity on H(0:2, 6:8) ==1, H(3:5, 9:11) == -1
		kal->H[i][i+6] = 1.0f;
		kal->H[i+3][i+9] = -1.0f;

	}

	K_MATRIX(kal); //Compute KALMAN GAIN
	ERROR_MODEL(kal, mag); //Create Error Model Z = [Zg, Zm]^T

	POSTERIORI_ERROR(kal); //Computes x+ = K*z

	Quaternion_q1(kal); //Computes the q- quaternion
	ORIENTATION_Est(kal, imu); //Computes q+ = (q-)(theta+), Updates posteriero estimates

	if (kal->tf == false){
		UPDATE_mag_VECTOR(kal, mag); //Computes Mag vector
	}

	EULER(kal,mag); //Calculates Euler angles from rotation matrix

	P1_MATRIX(kal); // Computes P1 from K, Q, H
	MATRIX_Q(kal); //Updates the matrix P0 with estiamte covariance Q





}


void Quaternion_q1(AHRS0 *kal) {


	//Calculate rotation angle length in similar method to pythagoras (alpha)
	float alpha = -1 * sqrtf(kal->thk[0]*kal->thk[0] + kal->thk[1]*kal->thk[1]+kal->thk[2]*kal->thk[2]);

	float alpha_rad = alpha * Deg2Rad;
	float sin_alpha_2 = sin(alpha_rad*0.5);

	float var = -1 * (sin_alpha_2 / alpha);

	//Compute small change in quaternion, theta * temp == normalised vector(i,j,k) * sin(alpha/2)

	float dq0, dq1, dq2, dq3;
	if (alpha != 0.0F){
		dq1 = kal->thk[0] * var;
		dq2 = kal->thk[1] * var;
		dq3 = kal->thk[2] * var;
	} else {
	    dq1 = 0;
		dq2 = 0;
		dq3 = 0;
	}

	float vec_mag2 = 0;

	//Check if magnitude2 of quaternion vector <= 1 (NXP FUSION) -> Can apply trig identity cos2 + sin2 == 1
	//vec_mag2 == sin_alpha_2
	vec_mag2 = dq1*dq1 + dq2*dq2 + dq3*dq3;
	if (vec_mag2 <= 1) {
		dq0 = sqrt(1.0f - vec_mag2);
	} else {
		dq0 = 0;
	}

	//Multiply change in orientation quaternion by previous orientation quaternion
	//(r1, v1)(r2, v2) = (r1r2 - dot(v1,v2), r1v2 + r2v1 + cross(v1, v2)

//	float q0 = kal->q0[0], q1 = kal->q0[1], q2 = kal->q0[2], q3 = kal->q0[3];

	float q0 = kal->q0[0], q1 = kal->q0[1], q2 = kal->q0[2], q3 = kal->q0[3]; //INITIAL STATE AT [0 0 0]
	//float qa = kal->del_q[0], qb = kal->del_q[1], qc = kal->del_q[2], qd = kal->del_q[3];

	/*kal->q1[0] = q0*qa - q1*qb - q2*qc - q3*qd;
	kal->q1[1] = q1*qa + q0*qb - q3*qc + q2*qd;
	kal->q1[2] = q2*qa + q3*qb + q0*qc - q1*qd;
	kal->q1[3] = q3*qa - q2*qb + q1*qc + q0*qd;*/


	kal->q1[0] = q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3; //TAKEN FROM NXP
	kal->q1[1] = q0*dq1 + q1*dq0 + q2*dq3 - q3*dq2;
	kal->q1[2] = q0*dq2 - q1*dq3 + q2*dq0 + q3*dq1;
	kal->q1[3] = q0*dq3 + q1*dq2 - q2*dq1 + q3*dq0;



	//NORMALISE q1 --- TAKEN FROM NXP

	float norm;

	norm = sqrt(kal->q1[0]*kal->q1[0] + kal->q1[1]*kal->q1[1] + kal->q1[2]*kal->q1[2] + kal->q1[3]*kal->q1[3]);

	if (norm > corrupt_quat){
		norm = 1.0f / norm;
		kal->q1[0] *= norm;
		kal->q1[1] *= norm;
		kal->q1[2] *= norm;
		kal->q1[3] *= norm;
	} else {
		kal->q1[0] = 1.0f;
		kal->q1[1] = 0.0f;
		kal->q1[2] = 0.0f;
		kal->q1[3] = 0.0f;
	}

	if (kal->q1[0] < 0.0f) {
		kal->q1[0] = -kal->q1[0];
		kal->q1[1] = -kal->q1[1];
		kal->q1[2] = -kal->q1[2];
    	kal->q1[3] = -kal->q1[3];
	}


	kal->R_Matrix_Pos[0][0] = 2*(kal->q1[0]*kal->q1[0] + kal->q1[1]*kal->q1[1]) - 1;
	kal->R_Matrix_Pos[0][1] = 2*(kal->q1[1]*kal->q1[2] + kal->q1[0]*kal->q1[3]);
	kal->R_Matrix_Pos[0][2] = 2*(kal->q1[1]*kal->q1[3] - kal->q1[0]*kal->q1[2]);

	kal->R_Matrix_Pos[1][0] = 2*(kal->q1[1]*kal->q1[2] - kal->q1[0]*kal->q1[3]);
	kal->R_Matrix_Pos[1][1] = 2*(kal->q1[0]*kal->q1[0] + kal->q1[2]*kal->q1[2]) - 1;
	kal->R_Matrix_Pos[1][2] = 2*(kal->q1[2]*kal->q1[3] + kal->q1[0]*kal->q1[1]);

	kal->R_Matrix_Pos[2][0] = 2*(kal->q1[1]*kal->q1[3] + kal->q1[0]*kal->q1[2]);
	kal->R_Matrix_Pos[2][1] = 2*(kal->q1[2]*kal->q1[3] - kal->q1[0]*kal->q1[1]);
	kal->R_Matrix_Pos[2][2] = 2*(kal->q1[0]*kal->q1[0] + kal->q1[3]*kal->q1[3]) - 1;

	float fetarad, fetadeg;
	if ((kal->q1[0] >= 1.0f) || (kal->q1[0] <= -1.0f)){
		fetarad = 0.0f;
		fetadeg = 0.0f;
	} else {
		fetarad = 2.0f*acos(kal->q1[0]);
		fetadeg = fetarad*Rad2Deg;
	}

	if (fetadeg >= 180.0f){
		fetadeg -=360.0f;
		fetarad = fetadeg*Deg2Rad;
	}

	float eta2 = sin(0.5f*fetarad);

	if (eta2 == 0.0f){
		kal->vector[0] = 0.0f;
		kal->vector[1] = 0.0f;
		kal->vector[2] = 0.0f;
	} else {
		float variable= fetadeg/eta2;
		kal->vector[0] = kal->q1[1] * variable;
		kal->vector[1] = kal->q1[2] * variable;
		kal->vector[2] = kal->q1[3] * variable;
	}


}


/*
void QUAT_q0(AHRS0 *kal){
	//Calculate rotation angle length in similar method to pythagoras (alpha)
	float alpha = sqrt(kal->delta_rad[0]*kal->delta_rad[0] + kal->delta_rad[1]*kal->delta_rad[1]+kal->delta_rad[2]*kal->delta_rad[2]);

	float sin_alpha_2 = sin(alpha*0.5);

	float var = sin_alpha_2 / alpha;

	//Compute small change in quaternion, delta_rad * temp == normalised vector(i,j,k) * sin(alpha/2)
	kal->q0[1] = kal->delta_rad[0] * var;
	kal->q0[2] = kal->delta_rad[1] * var;
	kal->q0[3] = kal->delta_rad[2] * var;

	//Check if magnitude2 of quaternion vector <= 1 (NXP FUSION) -> Can apply trig identity cos2 + sin2 == 1
	//vec_mag2 == sin_alpha_2
	float vec_mag2 = kal->del_q[0]*kal->del_q[0] + kal->del_q[1]*kal->del_q[1] + kal->del_q[2]*kal->del_q[2];

	if (vec_mag2 <= 1) {
		kal->q0[0] = cos(alpha/2);
	} else {
		kal->q0[0] = 0;
	}
}*/


void euler_quat(AHRS0 *kal){


	//int N = kal->factor;
	//Calculate rotation angle length in similar method to pythagoras (alpha)

	float dp0 = 1, dp1 = 0, dp2 = 0, dp3 = 0;
	float dq0, dq1, dq2, dq3;
	//ERROR COULD COME FROM factor == 0
	for (int i = 0; i < kal->factor; i++){

		float alpha = sqrt(gyro_data[0][i]*gyro_data[0][i] + gyro_data[1][i]*gyro_data[1][i]+gyro_data[2][i]*gyro_data[2][i]);
		float alpha_rad = alpha * Deg2Rad;
		float sin_alpha_2 = sin(alpha_rad*0.5f);

		float var = sin_alpha_2 / alpha;




		kal->yaw_gyro += gyro_data[2][i];


			if (alpha != 0.0F){
				dq1 = gyro_data[0][i] * var;
				dq2 = gyro_data[1][i] * var;
				dq3 = gyro_data[2][i] * var;
			} else {
			    dq1 = 0.0f;
				dq2 = 0.0f;
				dq3 = 0.0f;
			}

			float vec_mag2 = 0;

			//Check if magnitude2 of quaternion vector <= 1 (NXP FUSION) -> Can apply trig identity cos2 + sin2 == 1
			//vec_mag2 == sin_alpha_2
			vec_mag2 = dq1*dq1 + dq2*dq2 + dq3*dq3;
			if (vec_mag2 <= 1) {
				dq0 = sqrt(1.0f - vec_mag2);
			} else {
				dq0 = 0.0f;
			}

			//dp = dp * dq
			//Finding the product of the incremental quaternions taken from gyro readings
				dp0 = dp0*dq0 - dp1*dq1 - dp2*dq2 - dp3*dq3;
				dp1 = dp0*dq1 + dp1*dq0 + dp2*dq3 - dp3*dq2;
				dp2 = dp0*dq2 - dp1*dq3 + dp2*dq0 + dp3*dq1;
				dp3 = dp0*dq3 + dp1*dq2 - dp2*dq1 + dp3*dq0;

	}

	//kal->calculating = true;
//	kal->Num1 = 0;

	//Copy the result back into variable,
	dq0 = dp0;
	dq1 = dp1;
	dq2 = dp2;
	dq3 = dp3;

	memset(sum_gyr, 0, sizeof(sum_gyr));
	for (int i = 0; i < 3; i++){
		for (int j =0; j< kal->factor; j++) {

			sum_gyr[i] += (gyro_data[i][j]/0.001);

		}


	}

	memset(gyro_data, 0, sizeof(gyro_data));


	/*float alpha = sqrt(kal->delta_d[0]*kal->delta_d[0] + kal->delta_d[1]*kal->delta_d[1]+kal->delta_d[2]*kal->delta_d[2]);
	float alpha_rad = alpha * Deg2Rad;
	float sin_alpha_2 = sin(alpha_rad*0.5f);

	float var = sin_alpha_2 / alpha;


	float dq0, dq1, dq2, dq3;
		if (alpha != 0.0F){
			dq1 = kal->delta_d[0] * var;
			dq2 = kal->delta_d[1] * var;
			dq3 = kal->delta_d[2] * var;
		} else {
		    dq1 = 0.0f;
			dq2 = 0.0f;
			dq3 = 0.0f;
		}

		float vec_mag2 = 0;

		//Check if magnitude2 of quaternion vector <= 1 (NXP FUSION) -> Can apply trig identity cos2 + sin2 == 1
		//vec_mag2 == sin_alpha_2
		vec_mag2 = dq1*dq1 + dq2*dq2 + dq3*dq3;
		if (vec_mag2 <= 1) {
			dq0 = sqrt(1.0f - vec_mag2);
		} else {
			dq0 = 0.0f;
		}*/






	//Compute small change in quaternion, delta_d * temp == normalised vector(i,j,k) * sin(alpha/2)




	//Multiply change in orientation quaternion by previous orientation quaternion
	//(r1, v1)(r2, v2) = (r1r2 - dot(v1,v2), r1v2 + r2v1 + cross(v1, v2)

//	float q0 = kal->q0[0], q1 = kal->q0[1], q2 = kal->q0[2], q3 = kal->q0[3];

	float q0 =  kal->q1[0], q1 = kal->q1[1], q2 = kal->q1[2], q3 = kal->q1[3]; //INITIAL STATE AT [0 0 0]

	//float qa = kal->del_q[0], qb = kal->del_q[1], qc = kal->del_q[2], qd = kal->del_q[3];
	kal->q0[0] = q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3; //TAKEN FROM NXP
	kal->q0[1] = q0*dq1 + q1*dq0 + q2*dq3 - q3*dq2;
	kal->q0[2] = q0*dq2 - q1*dq3 + q2*dq0 + q3*dq1;
	kal->q0[3] = q0*dq3 + q1*dq2 - q2*dq1 + q3*dq0;



}

void MATRIX_q0(AHRS0 *kal){




		kal->R_Matrix[0][0] = 2.0F*(kal->q0[0]*kal->q0[0] + kal->q0[1]*kal->q0[1]) - 1.0F;
		kal->R_Matrix[0][1] = 2.0F*(kal->q0[1]*kal->q0[2] + kal->q0[0]*kal->q0[3]);
		kal->R_Matrix[0][2] = 2.0F*(kal->q0[1]*kal->q0[3] - kal->q0[0]*kal->q0[2]);

		kal->R_Matrix[1][0] = 2.0F*(kal->q0[1]*kal->q0[2] - kal->q0[0]*kal->q0[3]);
		kal->R_Matrix[1][1] = 2.0F*(kal->q0[0]*kal->q0[0] + kal->q0[2]*kal->q0[2]) - 1.0F;
		kal->R_Matrix[1][2] = 2.0F*(kal->q0[2]*kal->q0[3] + kal->q0[0]*kal->q0[1]);

		kal->R_Matrix[2][0] = 2.0F*(kal->q0[1]*kal->q0[3] + kal->q0[0]*kal->q0[2]);
		kal->R_Matrix[2][1] = 2.0F*(kal->q0[2]*kal->q0[3] - kal->q0[0]*kal->q0[1]);
		kal->R_Matrix[2][2] = 2.0F*(kal->q0[0]*kal->q0[0] + kal->q0[3]*kal->q0[3]) - 1.0F;

	/*

	float R[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

				 float normal[3]= {0, 0,0};

				 for (int i = 0; i<  3; i++){

					 normal[0] +=  kal->R_Matrix[i][0]*kal->R_Matrix[i][0];
					 normal[1] +=  kal->R_Matrix[i][1]*kal->R_Matrix[i][1];
					 normal[2] +=  kal->R_Matrix[i][2]*kal->R_Matrix[i][2];

				 }


				 for (int i = 0; i<  3; i++){

					 kal->R_Matrix[i][0] /= sqrt(normal[0]);
					 kal->R_Matrix[i][1] /= sqrt(normal[1]);
					 kal->R_Matrix[i][2] /= sqrt(normal[2]);


				 }

				 */


}
