/*
 * tasks.h
 *
 *  Created on: Oct 29, 2024
 *      Author: laure
 */

#ifndef TASKS_UWB_H_
#define TASKS_UWB_H_




void PositionCompUWB(void);
void EKFcompUWB(void);
void EKFUpdate(void);

void Kalman_X(void);
void Kalman_Y(void);
void Kalman_Z(void);

void GNSS_READ(void);
void MAG_Poll(void);

#endif /* TASKS_UWB_H_ */
