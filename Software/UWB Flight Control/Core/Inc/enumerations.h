/*
 * enumerations.h
 *
 *  Created on: Aug 20, 2023
 *      Author: laure
 */

#ifndef INC_ENUMERATIONS_H_
#define INC_ENUMERATIONS_H_



typedef enum {

	b0 = 0,
	b1 = 1,
	b2 = 2,
	b3 = 3,
	b4 = 4,

	c1 = 0x00,
	c2 = 0x28,
	c3 = 0x50

}bank;

typedef enum {

	_15_625dps, //42688 (15.625dps - 2000dps)
	_31_25dps,
	_62_5dps,
	_125dps,
	_250dps, //42670 (250dps - 2000dps)
	_500dps,
	_1000dps,
	_2000dps

}gyro_scale;



typedef enum {
	_16g,
	_8g,
	_4g,
	_2g

}accel_scale;

typedef enum {
	_16guass,
	_12guass,
	_8guass,
	_4guass

}mag_scale;



typedef enum {

	_32khz,
	_16khz,
	_8khz,
	_4khz,
	_2khz,
	_1600hz, //ICM42670
	_1khz,
	_800hz, //ICM42670
	_500hz,
	_400hz, //ICM42670
	_200hz, //ICM42670
	_100hz, //ICM42670
	_50hz, //ICM42670
	_25hz, //ICM42670
	_12_5hz, //ICM42670
	_6_25hz,
	_3_125hz,
	_1_5625hz,
	_0_625hz, //LIS3MDLTR
	_1_25hz, //LIS3MDLTR
	_2_5hz, //LIS3MDLTR
	_5hz, //LIS3MDLTR
	_10hz, //LIS3MDLTR
	_20hz, //LIS3MDLTR
	_40hz, //LIS3MDLTR
	_80hz //LIS3MDLTR

}ODR;


typedef enum {

	_LP,
	_MP,
	_HP,
	_UHP,
	_FASTODR,
	_DEFAULTODR
}XY_Mode;

typedef enum { //lis3dltr

	_CONTINUOUS,
	_SINGLEconversion,
	_POWERDOWN
}OP_Mode;

typedef struct{

	uint8_t state;

}BLOCKING;


#define spi_ready 1
#define spi_busy 0

/*

typedef enum {

	_4000hz, //0.125ms latency
	_170hz, //1ms latency
	_82hz, //2ms
	_40hz, //4ms
	_20hz, //8ms
	_10hz, //16ms
	_5hz, //32ms


}TEMP_LPF;

*/

typedef enum {
	_odr2,
	_400_odr4, //max(400Hz, ODR)/4
	_400_odr5,
	_400_odr8,
	_400_odr10,
	_400_odr16,
	_400_odr20,
	_400_odr40,
	_400_odr_lowlatency,
	_200_8odr_lowlatency
}COMMON_LPF; //max(400Hz, ODR)/(odrX)




#endif /* INC_ENUMERATIONS_H_ */
