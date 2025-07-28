/*
 * Stateest.h
 *
 *  Created on: Jun 15, 2024
 *      Author: laure
 */

#ifndef STATEEST_H_
#define STATEEST_H_

#include "arm_math.h"
//#include "HV_data.h"
//#include "globals.h"
//#include "types.h"

#define STD_NOISE_IMU      0.0004F //m/s^2
#define STD_NOISE_OFFSET   0.000001F
//#define STD_NOISE_IMU      5e-6
//#define STD_NOISE_OFFSET   1e-8F



#define STD_NOISE_BARO     900.0F
#define STD_NOISE_BARO_INITIAL  9.0F
#define CATS_ERR_FILTER_ACC 1



#define STD_NOISE_UWB 0.3 // RMS from MATLAB sims for 0.1m RMS point to point error
#define STD_NOISE_UWB_Z 0.1 // RMS from MATLAB sims for 0.1m RMS point to point error

typedef struct {
  float32_t Ad_data[9];
  float32_t Ad_T_data[9];
  float32_t Bd_data[3];
  float32_t GdQGd_T_data[9];
  float32_t H_data[3];
  float32_t H_T_data[3];
  float32_t K_data[3];
  float32_t x_bar_data[3];
  float32_t x_hat_data[3];
  float32_t P_bar_data[9];
  float32_t P_hat_data[9];
  arm_matrix_instance_f32 Ad;
  arm_matrix_instance_f32 Ad_T;
  arm_matrix_instance_f32 GdQGd_T;
  arm_matrix_instance_f32 Bd;
  arm_matrix_instance_f32 H;
  arm_matrix_instance_f32 H_T;
  arm_matrix_instance_f32 K;
  arm_matrix_instance_f32 x_bar;
  arm_matrix_instance_f32 x_hat;
  arm_matrix_instance_f32 P_bar;
  arm_matrix_instance_f32 P_hat;
  float32_t measured_acceleration;
  float32_t measured_AGL;
  float32_t R;
  float32_t t_sampl;
  float inputPos;
} kalman_filter_t;

void EKF_1D_Run(void);

void init_filter_struct(kalman_filter_t *filter);

void initialize_matrices(kalman_filter_t *filter, float32_t initP);

void kalman_prediction(kalman_filter_t *filter);

void reset_kalman(kalman_filter_t *filter);

void soft_reset_kalman(kalman_filter_t *filter);

void kalman_update(kalman_filter_t *filter);

void kalman_step(kalman_filter_t *filter);

void GetEstimationInputData(void);
#endif /* STATEEST_H_ */
