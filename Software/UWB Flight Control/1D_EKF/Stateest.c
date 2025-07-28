/*
 * Stateest.c
 *
 *  Created on: Jun 15, 2024
 *      Author: laure
 */


#include "Stateest.h"
#include "main.h"
//#include "types.h"
#include "stdbool.h"

//#include "cmsis_os2.h"
//#include "HV_data.h"
//#include "globals.h"
  kalman_filter_t m_filter;
//  orientation_filter_t m_orientation_filter;



//  bool GetNewFsmEnum() {
//      auto new_enum = (flight_fsm_e)osEventFlagsWait(fsm_flag_id, 0xFF, osFlagsNoClear, 0);
//
//      /* If this happens, there is an error on the Event Flag.*/
//      if (new_enum > TOUCHDOWN || new_enum < CALIBRATING) {
//        return false;
//      }
//
//      if (new_enum == m_fsm_enum) {
//        return false;
//      }
//
//      m_fsm_enum = new_enum;
//      return true;
//    }
void EKF_1D_Run(void) {


	/*
	 *  Init Kalman filter
	 */




	/*
	 * Init Orientation state est
	 */
//	  init_orientation_filter(&m_orientation_filter);
//	  reset_orientation_filter(&m_orientation_filter);


//	  uint32_t tick_count = osKernelGetTickCount();  //used as timestamp in recording data
//	  uint32_t tick_update = sysGetTickFreq() / CONTROL_SAMPLING_FREQ; //used as timestamp in recording data


	    /* update fsm enum */
//	    bool fsm_updated = GetNewFsmEnum();
//
//	/*
//	 * Reset IMU when go from calibrating to READY
//	 */
//	    if ((m_fsm_enum == READY) && fsm_updated) {
//	      reset_kalman(&m_filter);
//	    //  reset_orientation_filter(&m_orientation_filter);
//	    }
//
//	/*
//	 * Soft reset Kalman filter when go from READY to THRUSTING
//	 * Reset orientation when going to thrusting
//	 */
//
//	    if ((m_fsm_enum == THRUSTING) && fsm_updated) {
//	      soft_reset_kalman(&m_filter);
//	//      reset_orientation_filter(&m_orientation_filter);
//	    }
//
//
//
//
//	/*
//	 * Write measuremnet data into the filter struct
//	 */
//	    GetEstimationInputData();

	/*
	 * Do a Kalman step
	 */
//	    kalman_step(&m_filter, m_fsm_enum);


//	      orientation_info_t orientation_info{};
//
//	      for (uint8_t i = 0; i < 4; i++) {
//	        orientation_info.estimated_orientation[i] =
//	            static_cast<int16_t>(m_orientation_filter.estimate_data[i] * 10000.0F);
//	      }

	//      record(tick_count, ORIENTATION_INFO, &orientation_info);


	/*
	 * Record filtered data
	 */
//	      filtered_data_info_t filtered_data_info = {
//	          .filtered_altitude_AGL = m_filter.measured_AGL,
//	          .filtered_acceleration = m_filter.measured_acceleration,
//	      };
//
//	      record(tick_count, FILTERED_DATA_INFO, &filtered_data_info);



	/*
	 * Log EKF outputs
	 */


//	      flight_info_t flight_info = {.height = m_filter.x_bar_data[0],
//	                                   .velocity = m_filter.x_bar_data[1],
//	                                   .acceleration = m_filter.measured_acceleration + m_filter.x_bar_data[2]};
//	      if (m_fsm_enum >= DROGUE) {
//	        flight_info.acceleration = m_filter.x_bar_data[2];
//	      }
//	      record(tick_count, FLIGHT_INFO, &flight_info);
//
//	      // log_info("H: %ld; V: %ld; A: %ld; O: %ld", (int32_t)((float)filter.x_bar.pData[0] * 1000),
//	      //          (int32_t)((float)filter.x_bar.pData[1] * 1000), (int32_t)(filtered_data_info.filtered_acceleration *
//	      //          1000), (int32_t)((float)filter.x_bar.pData[2] * 1000));
//	      log_sim("[%lu]: height: %f, velocity: %f, offset: %f", tick_count, static_cast<double>(m_filter.x_bar.pData[0]),
//	              static_cast<double>(m_filter.x_bar.pData[1]), static_cast<double>(m_filter.x_bar_data[2]));
//
//	      tick_count += tick_update;
//	      osDelayUntil(tick_count);
//

	  }




void init_filter_struct(kalman_filter_t *const filter) {
  arm_mat_init_f32(&filter->Ad, 3, 3, filter->Ad_data);
  arm_mat_init_f32(&filter->Ad_T, 3, 3, filter->Ad_T_data);
  arm_mat_init_f32(&filter->Bd, 3, 1, filter->Bd_data);
  arm_mat_init_f32(&filter->GdQGd_T, 3, 3, filter->GdQGd_T_data);
  arm_mat_init_f32(&filter->H, 1, 3, filter->H_data);
  arm_mat_init_f32(&filter->H_T, 3, 1, filter->H_T_data);
  arm_mat_init_f32(&filter->K, 3, 1, filter->K_data);
  arm_mat_init_f32(&filter->x_hat, 3, 1, filter->x_hat_data);
  arm_mat_init_f32(&filter->x_bar, 3, 1, filter->x_bar_data);
  arm_mat_init_f32(&filter->P_hat, 3, 3, filter->P_hat_data);
  arm_mat_init_f32(&filter->P_bar, 3, 3, filter->P_bar_data);
}

void initialize_matrices(kalman_filter_t *const filter, float32_t initP) {
  /* Matrix -> mat[9] = [0, 1, 2; 3, 4 , 5; 6, 7, 8];*/


	float dt_uwb = 10.0F;
	filter->t_sampl = dt_uwb/1000.0F;
  /* Initialize static values */
  float32_t Ad[9] = {1, filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 0, 1, filter->t_sampl, 0, 0, 1};
  arm_matrix_instance_f32 Ad_mat;
  arm_mat_init_f32(&Ad_mat, 3, 3, Ad);

  float32_t Ad_T[9] = {1, filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 0, 1, filter->t_sampl, 0, 0, 1};
  arm_matrix_instance_f32 Ad_T_mat;
  arm_mat_init_f32(&Ad_T_mat, 3, 3, Ad_T);
  arm_mat_trans_f32(&Ad_mat, &Ad_T_mat);

  float32_t Gd[6] = {filter->t_sampl, filter->t_sampl * filter->t_sampl / 2, 1, filter->t_sampl, 0, 1};
  // float32_t Gd[6] = {filter->t_sampl, 0, 1, 0, 0, 0};
  arm_matrix_instance_f32 Gd_mat;
  arm_mat_init_f32(&Gd_mat, 3, 2, Gd);

  float32_t Gd_T[6];
  arm_matrix_instance_f32 Gd_T_mat;
  arm_mat_init_f32(&Gd_T_mat, 2, 3, Gd_T);
  arm_mat_trans_f32(&Gd_mat, &Gd_T_mat);

  float32_t Bd[3] = {filter->t_sampl * filter->t_sampl / 2, filter->t_sampl, 0};
  arm_matrix_instance_f32 Bd_mat;
  arm_mat_init_f32(&Bd_mat, 3, 1, Bd);

  float32_t H[3] = {1, 0, 0};
  arm_matrix_instance_f32 H_mat;
  arm_mat_init_f32(&H_mat, 1, 3, H);

  float32_t H_T[3] = {1, 0, 0};
  arm_matrix_instance_f32 H_T_mat;
  arm_mat_init_f32(&H_T_mat, 3, 1, H_T);

  float32_t Q[4] = {STD_NOISE_IMU, 0, 0, STD_NOISE_OFFSET};
  arm_matrix_instance_f32 Q_mat;
  arm_mat_init_f32(&Q_mat, 2, 2, Q);

  float32_t GdQGd_T[9];
  arm_matrix_instance_f32 GdQGd_T_mat;
  arm_mat_init_f32(&GdQGd_T_mat, 3, 3, GdQGd_T);

  float32_t holder[6];
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 3, 2, holder);

  arm_mat_mult_f32(&Gd_mat, &Q_mat, &holder_mat);
  arm_mat_mult_f32(&holder_mat, &Gd_T_mat, &GdQGd_T_mat);

  float32_t x_bar[3] = {initP, 0, 0};
  arm_matrix_instance_f32 x_bar_mat;
  arm_mat_init_f32(&x_bar_mat, 3, 1, x_bar);

  float32_t x_hat[3] = {initP, 0, 0};
  arm_matrix_instance_f32 x_hat_mat;
  arm_mat_init_f32(&x_hat_mat, 3, 1, x_hat);

  float32_t K[3] = {0, 0, 0};
  arm_matrix_instance_f32 K_mat;
  arm_mat_init_f32(&K_mat, 3, 1, K);

  float32_t P_hat[9] = {0.1F, 0, 0, 0, 0.1F, 0, 0, 0, 0.1F};
  arm_matrix_instance_f32 P_hat_mat;
  arm_mat_init_f32(&P_hat_mat, 3, 3, P_hat);

  float32_t P_bar[9] = {0.1F, 0, 0, 0, 0.1F, 0, 0, 0, 0.1F};
  arm_matrix_instance_f32 P_bar_mat;
  arm_mat_init_f32(&P_bar_mat, 3, 3, P_bar);

  filter->R = STD_NOISE_UWB;
  memcpy(filter->Ad_data, Ad, sizeof(Ad));
  memcpy(filter->Ad_T_data, Ad_T, sizeof(Ad_T));
  memcpy(filter->Bd_data, Bd, sizeof(Bd));
  memcpy(filter->GdQGd_T_data, GdQGd_T, sizeof(GdQGd_T));
  memcpy(filter->H_data, H, sizeof(H));
  memcpy(filter->H_T_data, H_T, sizeof(H_T));
  memcpy(filter->K_data, K, sizeof(K));
  memcpy(filter->x_bar_data, x_bar, sizeof(x_bar));
  memcpy(filter->x_hat_data, x_hat, sizeof(x_hat));
  memcpy(filter->P_bar_data, P_bar, sizeof(P_bar));
  memcpy(filter->P_hat_data, P_hat, sizeof(P_hat));
}

void reset_kalman(kalman_filter_t *filter) {
//  log_debug("Resetting Kalman Filter...");
  float32_t x_dash[3] = {0.0F, 10.0F, 0.0F};
  float32_t P_dash[9] = {0.1F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.1F};

  memcpy(filter->x_bar_data, x_dash, sizeof(x_dash));
  memcpy(filter->x_bar_data, x_dash, sizeof(x_dash));
  memcpy(filter->P_bar_data, P_dash, sizeof(P_dash));
  memcpy(filter->P_bar_data, P_dash, sizeof(P_dash));
}

void soft_reset_kalman(kalman_filter_t *filter) {
//  log_debug("Resetting Kalman Filter...");
  float32_t P_dash[9] = {0.1F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.1F};

//filter->x_bar_data[1] = 0;
//filter->x_hat_data[1] = 0;



  memcpy(filter->P_hat_data, P_dash, sizeof(P_dash));
  memcpy(filter->P_bar_data, P_dash, sizeof(P_dash));
}

/* This Function Implements the kalman Prediction as long as more than 0 IMU
 * work */
void kalman_prediction(kalman_filter_t *filter) {
  float32_t holder_data[3];
  arm_matrix_instance_f32 holder_vec;
  arm_mat_init_f32(&holder_vec, 3, 1, holder_data);

  float32_t holder2_data[3];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 3, 1, holder2_data);

  float32_t holder[9];
  arm_matrix_instance_f32 holder_mat;
  arm_mat_init_f32(&holder_mat, 3, 3, holder);

  float32_t holder2[9];
  arm_matrix_instance_f32 holder2_mat;
  arm_mat_init_f32(&holder2_mat, 3, 3, holder2);

  /* Prediction Step */
arm_status statusus;
  /* Calculate Prediction of the state: x_hat = A*x_bar + B*u */
statusus=  arm_mat_mult_f32(&filter->Ad, &filter->x_bar, &holder_vec);
statusus = arm_mat_scale_f32(&filter->Bd, filter->measured_acceleration, &holder2_vec);
statusus= arm_mat_add_f32(&holder_vec, &holder2_vec, &filter->x_hat);

  /* Update the Variance of the state P_hat = A*P_bar*A' + GQG' */
statusus=  arm_mat_mult_f32(&filter->Ad, &filter->P_bar, &holder_mat);
  statusus=  arm_mat_mult_f32(&holder_mat, &filter->Ad_T, &holder2_mat);
  statusus= arm_mat_add_f32(&holder2_mat, &filter->GdQGd_T, &filter->P_hat);

  memcpy(filter->P_bar_data, filter->P_hat_data, sizeof(filter->P_hat_data));
  memcpy(filter->x_bar_data, filter->x_hat_data, sizeof(filter->x_hat_data));

  /* Prediction Step finished */
}

/* This function implements the Kalman update when no Barometer is faulty */
void kalman_update(kalman_filter_t *filter) {
	arm_status statusus;

	  memcpy(filter->P_hat_data, filter->P_bar_data, sizeof(filter->P_bar_data));
	  memcpy(filter->x_hat_data, filter->x_bar_data, sizeof(filter->x_bar_data));


  float32_t holder_single[1];
  arm_matrix_instance_f32 holder_single_mat;
  arm_mat_init_f32(&holder_single_mat, 1, 1, holder_single);

  float32_t holder2_data[3];
  arm_matrix_instance_f32 holder2_vec;
  arm_mat_init_f32(&holder2_vec, 3, 1, holder2_data);

  float32_t holder_0_1x3[3];
  arm_matrix_instance_f32 holder_0_1x3_mat;
  arm_mat_init_f32(&holder_0_1x3_mat, 1, 3, holder_0_1x3);

  /* Update Step */

  /* Calculate K = P_hat*H_T*(H*P_Hat*H_T+R)^-1 */
  statusus =   arm_mat_mult_f32(&filter->H, &filter->P_hat, &holder_0_1x3_mat);
  statusus =  arm_mat_mult_f32(&holder_0_1x3_mat, &filter->H_T, &holder_single_mat);
  holder_single[0] += filter->R;

  statusus =arm_mat_mult_f32(&filter->P_hat, &filter->H_T, &holder2_vec);
  statusus= arm_mat_scale_f32(&holder2_vec, 1.0F / holder_single[0], &filter->K);

  /* Calculate x_bar = x_hat+K*(y-Hx_hat); */

  statusus =  arm_mat_mult_f32(&filter->H, &filter->x_hat, &holder_single_mat);
  statusus = arm_mat_scale_f32(&filter->K, filter->measured_AGL - holder_single[0], &holder2_vec);
  statusus = arm_mat_add_f32(&holder2_vec, &filter->x_hat, &filter->x_bar);

  /* Finished Calculating x_bar */

  /* Calculate P_bar = (eye-K*H)*P_hat */
  float32_t eye[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  arm_matrix_instance_f32 eye_mat;
  arm_mat_init_f32(&eye_mat, 3, 3, eye);
  float32_t holder_3x3[9];
  arm_matrix_instance_f32 holder_3x3_mat;
  arm_mat_init_f32(&holder_3x3_mat, 3, 3, holder_3x3);
  float32_t holder2_3x3[9];
  arm_matrix_instance_f32 holder2_3x3_mat;
  arm_mat_init_f32(&holder2_3x3_mat, 3, 3, holder2_3x3);

  statusus =  arm_mat_mult_f32(&filter->K, &filter->H, &holder_3x3_mat);
  statusus = arm_mat_sub_f32(&eye_mat, &holder_3x3_mat, &holder2_3x3_mat);
  statusus = arm_mat_mult_f32(&holder2_3x3_mat, &filter->P_hat, &filter->P_bar);
}

float32_t R_interpolation(float32_t velocity) {
  /* Todo: Can be optimized */
  const float32_t lower_bound = 20.0F;
  const float32_t upper_bound = 100.0F;
  const float32_t f_lower_bound = 0.3981F;
  const float32_t f_upper_bound = 1.0F;

  const float32_t m = (f_lower_bound - f_upper_bound) / (lower_bound - upper_bound);
  const float32_t b = f_upper_bound - m * upper_bound;
  if (velocity < lower_bound) {
    return powf(f_lower_bound, 5.0F);
  }
  if (velocity < upper_bound) {
    return powf(m * velocity + b, 5.0F);
  }
  return f_upper_bound;
}

void kalman_step(kalman_filter_t *filter) {
  /* Update IMU trust value based on flight phase */


 // filter->R = STD_NOISE_UWB;

  /* If all IMUs are disabled, trust the barometer */
//  if (get_error_by_tag(CATS_ERR_FILTER_ACC)) {
//    filter->R = STD_NOISE_BARO_INITIAL;
//  }

 // kalman_prediction(filter);

  kalman_update(filter);



  GetEstimationOutput();

}


void GetEstimationInputData(void){




//
//
//	  if (m_fsm_enum < DROGUE) {
//	    m_filter.measured_acceleration = input_data.accelZ;
//	  } else {
//	    m_filter.measured_acceleration = 0;
//	  }
//
//	  m_filter.measured_AGL = input_data.height_AGL;
//



}

void GetEstimationOutput(void) {

//	estimation_output_t output = {};
//
//	state.UWB_Px = m_filter.x_bar_data[0];
//	output.velocity = m_filter.x_bar_data[1];
//	output.acceleration = m_filter.measured_acceleration + m_filter.x_bar_data[2];
//
//	m_est_output = output;
//
////	state.height = (float)output.height;
////	state.vel_Z = (float)output.velocity;
//
//	return output;
}




