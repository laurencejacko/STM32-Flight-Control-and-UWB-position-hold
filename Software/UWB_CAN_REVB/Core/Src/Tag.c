/*
 * Tag.c
 *
 *  Created on: Mar 16, 2024
 *      Author: laure
 */
/*! ----------------------------------------------------------------------------
 *  @file    ds_twr_responder.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 - 2021 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "deca_probe_interface.h"
#include <config_options.h>
#include <deca_device_api.h>
#include <deca_spi.h>
#include <example_selection.h>
#include <port.h>
#include <shared_defines.h>
#include <shared_functions.h>
#include <math.h>
#include <string.h>

#include "stdlib.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "examples_defines.h"
#include <assert.h>
#include <example_selection.h>


extern example_ptr example_pointer;


#if (defined(TEST_DS_TWR_RESPONDER))

void add_distANDrssi_to_PCmsg(uint8_t *a, char *b, char *c, uint8_t dist_idx,
		uint8_t addressTag);
float getRSSI(uint32_t C, uint16_t N, uint8_t D, float A);
static dwt_rxdiag_t rx_diag;
uint32_t C; // Channel Impulse Response Power; ipatovPower
uint16_t N; // Preamble Accumulation Count; ipatovAccumCount
uint8_t D; // DGC_DECISION (0 to 7)
const float A = 121.7;
float rssi;
char rssi_str[16] = { '\0' };
char dist_str_to_PC[16] = { 0 };
static uint8_t tx_dist_to_PC[] =
		{ 0x41, 0x88, 0, 0xCA, 0xDE, 'P', 'C', 'A', 'N', 0xE2, 'C', 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; //30 length
#define DISTANCE_IDX 11

extern void test_run_info(unsigned char *data);

#include "stdio.h"

int _write(int file, char *ptr, int len) {
	int iii = 0;
	for (iii = 0; iii < len; iii++)
		ITM_SendChar((*ptr++));
	return len;
}
/* Example application name */
#define APP_NAME "DS TWR RESP v1.0"

/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = { 5, /* Channel number. */
DWT_PLEN_128, /* Preamble length. Used in TX only. */
DWT_PAC8, /* Preamble acquisition chunk size. Used in RX only. */
9, /* TX preamble code. Used in TX only. */
9, /* RX preamble code. Used in RX only. */
1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
DWT_BR_6M8, /* Data rate. */
DWT_PHRMODE_STD, /* PHY header mode. */
DWT_PHRRATE_STD, /* PHY header rate. */
(129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
DWT_STS_MODE_OFF, /* STS disabled */
DWT_STS_LEN_64, /* STS length see allowed values in Enum dwt_sts_lengths_e */
DWT_PDOA_M0 /* PDOA mode off */
};

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 15



/* Frames used in the ranging process. See NOTE 2 below. */
static uint8_t rx_poll_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
		0x21, 0, 0 };
static uint8_t tx_resp_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A',
		0x10, 0x02, 0, 0, 0, 0 };
static uint8_t rx_final_msg[] = { 0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E',
		0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX            2
#define FINAL_MSG_POLL_TX_TS_IDX  10
#define FINAL_MSG_RESP_RX_TS_IDX  14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
static uint8_t initiatorAdress[] = { 0, 0 };
/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8_t rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW IC's delayed TX function. This includes the
 * frame length of approximately 190 us with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 900
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW IC's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 220
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT 5

/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

static double position[6];
static float signal_strength[6];
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
 * temperature. These values can be calibrated prior to taking reference measurements. See NOTE 2 below. */
extern dwt_txconfig_t txconfig_options;

static uint8_t allMSGCOMMONLEN = 7;
#define ANCHOR_IDX 11

char msg_to_PC[100];

void get_msg_toPC(char *a, uint8_t *b, uint8_t anchor_idx);

char distA[5];
char distB[5];
char distC[5];

//extern UART_HandleTypeDef huart2;

#define SIG_LVL_FACTOR     0.4 // Factor between 0 and 1; default 0.4 from experiments and simulations.
#define SIG_LVL_THRESHOLD  12 // Threshold unit is dB; default 12dB from experiments and simulations.
#define ALPHA_PRF_16       113.8 // Constant A for PRF of 16 MHz. See User Manual for more information.
#define ALPHA_PRF_64       120.7 // Constant A for PRF of 64 MHz. See User Manual for more information.
#define RX_CODE_THRESHOLD  8 // For 64 MHz PRF the RX code is 9.
#define LOG_CONSTANT_C0    63.2 // 10log10(2^21) = 63.2    // See User Manual for more information.
#define LOG_CONSTANT_D0_E0 51.175 // 10log10(2^17) = 51.175  // See User Manual for more information.
#define IP_MIN_THRESHOLD   3.3 // Minimum Signal Level in dB. Please see App Notes "APS006 PART 3"
#define IP_MAX_THRESHOLD   6.0 // Minimum Signal Level in dB. Please see App Notes "APS006 PART 3"
#define CONSTANT_PR_IP_A   0.39178 // Constant from simulations on DW device accumulator, please see App Notes "APS006 PART 3"
#define CONSTANT_PR_IP_B   1.31719 // Constant from simulations on DW device accumulator, please see App Notes "APS006 PART 3"

extern UART_HandleTypeDef huart2;

int counter[6];

#define NUM_MES 300
double reldistance[6][NUM_MES];

float lowpass_to_beta(float f0, float fs);

float lowpass_to_beta(float f0, float fs) {
	//return 1 - exp(-2 * PI * f0 / fs);
	float RC = 1 / (2 * M_PI * f0);
	return (1 / fs) / (RC + 1 / fs);
}

double final_distance[6];

double avg_reldistance[6];
double median[6];

union DoublePair {

	struct {
			float float1;
			float float2;
		};

							uint8_t txBuffer[8]; // 8 bytes, exactly enough for two floats

						};

union DoublePair converter;

int compare(const void* a, const void* b) {
    double diff = *(double *)a - *(double *)b;
    return (diff > 0) - (diff < 0); // Returns 1 if a > b, -1 if a < b, 0 if equal
}

extern CAN_HandleTypeDef hcan1;
extern uint32_t              TxMailbox;
extern CAN_TxHeaderTypeDef   TxHeader[6];

extern int printData;




//#define AnchorIndex 0 //define the current PCB as anchor A

//#if AnchorIndex == 0
////A
//int AggregateDelay = 32994;
//
//#elif AnchorIndex == 1
////B
//	int AggregateDelay = 33035;
//
//#elif AnchorIndex == 2
////C
//	int AggregateDelay = 32722;
//
//#elif AnchorIndex == 3
////D
//	int AggregateDelay = 32593;
//
//#elif AnchorIndex == 4
////E
//			int AggregateDelay =32875;
//
//#elif AnchorIndex == 5
////F
//					int AggregateDelay = 2*16385;
//
//#elif AnchorIndex == TAG
////TAG
//							int AggregateDelay = 32757;
//
//#endif
#if AnchorIndex == 0
//A
int AggregateDelay = 32837;

#elif AnchorIndex == 1
//B
	int AggregateDelay = 32829;

#elif AnchorIndex == 2
//C
	int AggregateDelay = 32816;

#elif AnchorIndex == 3
//D
	int AggregateDelay = 32818;

#elif AnchorIndex == 4
//E
			int AggregateDelay =32841;

#elif AnchorIndex == 5
//F
					int AggregateDelay = 32770;

#elif AnchorIndex == TAG
//TAG
						//	int AggregateDelay = 32774;
							int AggregateDelay = 0;


#endif

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
//#define TX_ANT_DLY 16385
//#define RX_ANT_DLY 16385
#define TxPercent 0.44
#define RxPercent 0.56


#define TX_ANT_DLY TxPercent*AggregateDelay
#define RX_ANT_DLY RxPercent*AggregateDelay


//#define PC //Define PC to send data to gateway node, comment out to transmit data to PC via USB
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn ds_twr_responder()
 *
 * @brief Application entry point.
 *
 * @param  none
 *
 * @return none
 */
int ds_twr_responder(void) {

	uint32_t dev_id;
	uint8_t D;
	dwt_nlos_alldiag_t all_diag;
	dwt_nlos_ipdiag_t index;

	// All float variables used for recording different diagnostic results and probability.
	float ip_f1, ip_f2, ip_f3, sts1_f1, sts1_f2, sts1_f3, sts2_f1, sts2_f2,
			sts2_f3 = 0;
	float ip_n, sts1_n, sts2_n, ip_cp, sts1_cp, sts2_cp, ip_rsl, ip_fsl,
			sts1_rsl, sts1_fsl, sts2_rsl, sts2_fsl = 0;
	float pr_nlos, sl_diff_ip, sl_diff_sts1, sl_diff_sts2, sl_diff, index_diff =
			0;
	float alpha, ip_alpha, log_constant = 0;

	/* Display application name on LCD. */
	test_run_info((unsigned char*) APP_NAME);

	/* Configure SPI rate, DW3000 supports up to 36 MHz */
//	port_set_dw_ic_spi_fastrate();
	/* Reset DW IC */
	reset_DWIC(); /* Target specific drive of RSTn line into DW IC low for a period. */

	Sleep(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC

	/* Probe for the correct device driver. */
	int ret = dwt_probe((struct dwt_probe_s*) &dw3000_probe_interf);
	if (ret != 0) {
		while (1) {
			HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_Delay(100);

		}
	}

	dev_id = dwt_readdevid();

	while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */{
	};

	if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
		test_run_info((unsigned char*) "INIT FAILED     ");
		while (1) {
		};
	}

	/* Configure DW IC. See NOTE 15 below. */
	/* if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device */
	if (dwt_configure(&config)) {
		test_run_info((unsigned char*) "CONFIG FAILED [TAG]    ");
		while (1) {
			test_run_info((unsigned char*) "CONFIG FAILED [TAG]    ");
		};
	}

	/* Configure the TX spectrum parameters (power, PG delay and PG count) */
	dwt_configuretxrf(&txconfig_options);

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);

//	 uint16_t returneddelay = dwt_getrxantennadelay();
	dwt_settxantennadelay(TX_ANT_DLY);

	/* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
	 * Note, in real low power applications the LEDs should not be used. */
	dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

	memset(distA, 0, sizeof(distA));
	memset(distB, 0, sizeof(distB));
	memset(distC, 0, sizeof(distC));
	if ((dev_id == (uint32_t) DWT_DW3000_DEV_ID)
			|| (dev_id == (uint32_t) DWT_DW3000_PDOA_DEV_ID)) {
		log_constant = LOG_CONSTANT_C0;
	} else {
		log_constant = LOG_CONSTANT_D0_E0;
	}

	dwt_configciadiag(DW_CIA_DIAG_LOG_ALL);

	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);

	memset(counter, 0, sizeof(counter));
	memset(final_distance, 0, sizeof(final_distance));


//	dwt_readcarrierintegrator();



	/* Loop forever responding to ranging requests. */
	while (1) {

		if 	(example_pointer != ds_twr_responder) {
			break;
		}

		/*
		 *
		 * simple_tx.c sketch, begins the blink transaction to define a new node being polled from
		 *
		 */

		/*
		 *
		 * Begins the ds_twr_responder.c sketch, calculates the distance upon successful completion.
		 *
		 */
		dwt_setpreambledetecttimeout(0);
		/* Clear reception timeout to start next ranging process. */
		dwt_setrxtimeout(0);

		/* Activate reception immediately. */
		dwt_rxenable(DWT_START_RX_IMMEDIATE);

		/* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
		waitforsysstatus(&status_reg, NULL,
				(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
						| SYS_STATUS_ALL_RX_ERR), 0);

		if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
			uint16_t frame_len;

			/* Clear good RX frame event in the DW IC status register. */
			dwt_writesysstatuslo(DWT_INT_RXFCG_BIT_MASK);

			/* A frame has been received, read it into the local buffer. */
			frame_len = dwt_getframelength();
			if (frame_len <= RX_BUF_LEN) {
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}

			/* Check that the frame is a poll sent by "DS TWR initiator" example.
			 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
			rx_buffer[ALL_MSG_SN_IDX] = 0;
			if (memcmp(rx_buffer, rx_poll_msg, allMSGCOMMONLEN) == 0) {

				initiatorAdress[0] = rx_buffer[7]; //tutaj
				initiatorAdress[1] = rx_buffer[8];

				// set current initiator address
				tx_resp_msg[5] = initiatorAdress[0];
				tx_resp_msg[6] = initiatorAdress[1];
				//
				rx_poll_msg[7] = initiatorAdress[0];
				rx_poll_msg[8] = initiatorAdress[1];
				//
				rx_final_msg[7] = initiatorAdress[0];
				rx_final_msg[8] = initiatorAdress[1];
				allMSGCOMMONLEN = 10;

				uint32_t resp_tx_time;
				int ret;

				/* Retrieve poll reception timestamp. */
				poll_rx_ts = get_rx_timestamp_u64();

				/* Set send time for response. See NOTE 9 below. */
				resp_tx_time = (poll_rx_ts
						+ (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
				dwt_setdelayedtrxtime(resp_tx_time);

				/* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
				dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
				dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
				/* Set preamble timeout for expected frames. See NOTE 6 below. */
				dwt_setpreambledetecttimeout(PRE_TIMEOUT);

				/* Write and send the response message. See NOTE 10 below.*/
				tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
				dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
				dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
				ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

				/* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
				if (ret == DWT_ERROR) {
					continue;
				}

				/* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
				waitforsysstatus(&status_reg, NULL,
						(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
								| SYS_STATUS_ALL_RX_ERR), 0);

				/* Increment frame sequence number after transmission of the response message (modulo 256). */
				frame_seq_nb++;
				//}
				// else if (memcmp(rx_buffer, rx_final_msg, allMSGCOMMONLEN)) {
				if (status_reg & DWT_INT_RXFCG_BIT_MASK) {
					/* Clear good RX frame event and TX frame sent in the DW IC status register. */
					dwt_writesysstatuslo(
							DWT_INT_RXFCG_BIT_MASK | DWT_INT_TXFRS_BIT_MASK);

					/* A frame has been received, read it into the local buffer. */
					frame_len = dwt_getframelength();
					if (frame_len <= RX_BUF_LEN) {
						dwt_readrxdata(rx_buffer, frame_len, 0);
					}

					/* Check that the frame is a final message sent by "DS TWR initiator" example.
					 * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
					rx_buffer[ALL_MSG_SN_IDX] = 0;
					if (memcmp(rx_buffer, rx_final_msg, allMSGCOMMONLEN) == 0) {
						uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
						uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
						double Ra, Rb, Da, Db;
						int64_t tof_dtu;

						/* Retrieve response transmission and final reception timestamps. */
						resp_tx_ts = get_tx_timestamp_u64();
						final_rx_ts = get_rx_timestamp_u64();

						/* Get timestamps embedded in the final message. */
						final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX],
								&poll_tx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX],
								&resp_rx_ts);
						final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX],
								&final_tx_ts);

						/* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
						poll_rx_ts_32 = (uint32_t) poll_rx_ts;
						resp_tx_ts_32 = (uint32_t) resp_tx_ts;
						final_rx_ts_32 = (uint32_t) final_rx_ts;
						Ra = (double) (resp_rx_ts - poll_tx_ts);
						Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
						Da = (double) (final_tx_ts - resp_rx_ts);
						Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
						tof_dtu = (int64_t) ((Ra * Rb - Da * Db)
								/ (Ra + Rb + Da + Db));

						tof = tof_dtu * DWT_TIME_UNITS;
						distance = tof * SPEED_OF_LIGHT;
						/* Display computed distance on LCD. */
					//	sprintf(dist_str, "DIST: %3.2f m \r\n", distance);
						//    test_run_info((unsigned char *)dist_str);
						//printf("distance (m): %0.3f \r\n", distance);
						/* as DS-TWR initiator is waiting for RNG_DELAY_MS before next poll transmission
						 * we can add a delay here before RX is re-enabled again
						 */
						allMSGCOMMONLEN = 7;

						//CALCULATES RSSI with the github method (same values)
						dwt_readdiagnostics(&rx_diag);
						C = rx_diag.ipatovPower;
						N = rx_diag.ipatovAccumCount;
						/* ====> Read DGC here <==== */
						D = dwt_get_dgcdecision();
						/* ====> Calculate RSSI <==== */
						rssi = getRSSI(C, N, D, A);//41, 103, 4

						float alpha =1;
						uint8_t indexCurrent;
						if ((rx_buffer[7] == 'A') && (rx_buffer[8] == 1)) {

							indexCurrent = 0;

//							if (distance > 0){
//							position[0] = (1-alpha)*position[0] + (alpha)*distance;
//							signal_strength[0] = rssi;
//							}
//
//							if ((distance > 0) && (counter[0] < 100)) {
//								reldistance[0][counter[0]] += distance;
//								counter[0]++;
//							}

//							if (counter[0] == 100) {
//								reldistance[0] /= 100;
//								counter[0]++;
//							}

							//		SendMessageCAN(rx_buffer[7], distance);

							//	HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);


						} else if ((rx_buffer[7] == 'B')
								&& (rx_buffer[8] == 2)) {

							indexCurrent = 1;


						} else if ((rx_buffer[7] == 'C')
								&& (rx_buffer[8] == 3)) {

							indexCurrent = 2;


						} else if ((rx_buffer[7] == 'D')
								&& (rx_buffer[8] == 4)) {

							indexCurrent = 3;


						} else if ((rx_buffer[7] == 'E')
								&& (rx_buffer[8] == 5)) {

							indexCurrent = 4;


						} else if ((rx_buffer[7] == 'F')
								&& (rx_buffer[8] == 6)) {

							indexCurrent = 5;



						}


						if (distance > 0) { //if distance is postive, update position with LPF distance
							position[indexCurrent] = (1 - alpha) * position[indexCurrent] + (alpha) * distance;

							signal_strength[indexCurrent] = rssi;
						}

						if ((distance > 0) && (counter[indexCurrent] < NUM_MES)) { //if distance is positive and within the first 100 readings
							reldistance[indexCurrent][counter[indexCurrent]] = distance;
							counter[indexCurrent]++;
						}

						if (counter[indexCurrent] == NUM_MES) { //if 100 readings done, find the median distance

							qsort(reldistance[indexCurrent], NUM_MES, sizeof(double), compare); //sort the 100 readings, smallest to largest



							median[indexCurrent] = reldistance[indexCurrent][NUM_MES/2]; //take the median value from the sorted array


							for (int i = 0; i < NUM_MES; i++){
								avg_reldistance[indexCurrent] += reldistance[indexCurrent][i];
							}


							avg_reldistance[indexCurrent] /= NUM_MES;

							counter[indexCurrent]++; //increment the counter so qsort isnt called again


						}

						converter.float1 = (float)position[indexCurrent];
						converter.float2 = (float)rssi;


				  		HAL_CAN_AddTxMessage(&hcan1, &TxHeader[indexCurrent], converter.txBuffer, &TxMailbox);

				  		char txBuffer[100];

				  		if (printData == 1) {
				  			/*
				  			 * printData = 1 send the averaged distance to USB, can be changing by
				  			 * sending command from PC to UWB board
				  			 */

				  		sprintf(txBuffer, "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\r\n",
				  				(float)avg_reldistance[0],(float)avg_reldistance[1],(float)avg_reldistance[2],
								(float)avg_reldistance[3],(float)avg_reldistance[4],(float)avg_reldistance[5]);
				  		} else if (printData == 2) {
				  			sprintf(txBuffer, "%0.4f,%0.4f,%0.4f,%0.4f,%0.4f,%0.4f\r\n",
				  							  				(float)position[0],(float)position[1],(float)position[2],
				  											(float)position[3],(float)position[4],(float)position[5]);

				  		}

				  		test_run_info((unsigned char *)txBuffer);


/*
 *
 * SENDING THE UWB DATA OVER UART INSTEAD OF CAN BUS
 *
 *
						char buffer[128];
						memset(buffer, 0, sizeof(buffer));
						//     sprintf(buffer, "$G,%0.3f, %0.3f, %0.3f, %0.3f,%0.3f, %0.3f\r\n",position[0],position[1],position[2],position[3],position[4],position[5]);
						//  test_run_info((unsigned char *)buffer);

						uint16_t j =
								snprintf(buffer, sizeof(buffer),
										"$G,%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f \r\n",
										position[0], position[1], position[2],
										position[3], position[4], position[5]);
						test_run_info((unsigned char*) buffer);
						//  sprintf(buffer, "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f,%0.3f, %0.3f, %0.3f", ahrs.q0[0],ahrs.q0[1],ahrs.q0[2],ahrs.q0[3], imu1.acc_g[0], imu1.acc_g[1], imu1.acc_g[2], ahrs.linAccel[0], ahrs.linAccel[1], ahrs.linAccel[2], mag1.mag_g[0], mag1.mag_g[1] , mag1.mag_g[2]);

						// sprintf(buffer, "%0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f\r\n"  , ahrs.q0[0],ahrs.q0[1],ahrs.q0[2],ahrs.q0[3], mag1.mag_g[0], mag1.mag_g[1] , mag1.mag_g[2]);

						HAL_UART_Transmit_IT(&huart2, (uint8_t*) buffer, j);
*/



						/*
						 *  CALCULATE RSSI with the simple_rx_nlos.c method
						 // Select IPATOV to read Ipatov diagnostic registers from API function dwt_nlos_alldiag()
						 all_diag.diag_type = IPATOV;
						 dwt_nlos_alldiag(&all_diag);
						 ip_alpha =
						 (config.rxCode > RX_CODE_THRESHOLD) ?
						 (-(ALPHA_PRF_64 + 1)) : -(ALPHA_PRF_16);
						 ip_n = all_diag.accumCount; // The number of preamble symbols accumulated
						 ip_f1 = all_diag.F1 / 4; // The First Path Amplitude (point 1) magnitude value (it has 2 fractional bits),
						 ip_f2 = all_diag.F2 / 4; // The First Path Amplitude (point 2) magnitude value (it has 2 fractional bits),
						 ip_f3 = all_diag.F3 / 4; // The First Path Amplitude (point 3) magnitude value (it has 2 fractional bits),
						 ip_cp = all_diag.cir_power;
						 // For IPATOV
						 ip_n *= ip_n;
						 ip_f1 *= ip_f1;
						 ip_f2 *= ip_f2;
						 ip_f3 *= ip_f3;

						 D = all_diag.D * 6;

						 ip_rsl = 10 * log10((float) ip_cp / ip_n) + ip_alpha
						 + log_constant + D;*/

						Sleep(RNG_DELAY_MS - 10); // start couple of ms earlier
					}
				} else {
					/* Clear RX error/timeout events in the DW IC status register. */
					dwt_writesysstatuslo(
					SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
				}
			}
		} else {
			/* Clear RX error/timeout events in the DW IC status register. */
			dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
		}
	}
}

void add_distANDrssi_to_PCmsg(uint8_t *a, char *b, char *c, uint8_t dist_idx,
		uint8_t addressTag) {
	uint8_t i = 0;
	uint8_t temp = 0;

	a[dist_idx] = addressTag;
	//a[dist_idx + 1] = "/";
	for (i = 0; i < strlen(b) + 1; i++) {
		a[dist_idx + i + 1] = b[i];
	}

	temp = dist_idx + i;
	for (i = 0; i < strlen(c); i++) {
		a[temp + i] = c[i];
	}

	//a[temp + i] = addressTag;
}

float getRSSI(uint32_t C, uint16_t N, uint8_t D, float A) {
	float rssi;
	rssi = 10 * log10((C * pow(2.0, 21.0)) / pow((double) N, 2.0)) + (6 * D)
			- A;
	return rssi;
}





#endif
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW IC.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 *    Initiator: |Poll TX| ..... |Resp RX| ........ |Final TX|
 *    Responder: |Poll RX| ..... |Resp TX| ........ |Final RX|
 *                   ^|P RMARKER|                                    - time of Poll TX/RX
 *                                   ^|R RMARKER|                    - time of Resp TX/RX
 *                                                      ^|R RMARKER| - time of Final TX/RX
 *
 *                       <--TDLY->                                   - POLL_TX_TO_RESP_RX_DLY_UUS (RDLY-RLEN)
 *                               <-RLEN->                            - RESP_RX_TIMEOUT_UUS   (length of poll frame)
 *                    <----RDLY------>                               - POLL_RX_TO_RESP_TX_DLY_UUS (depends on how quickly responder
 *                                                                                                                      can turn around and reply)
 *
 *
 *                                        <--T2DLY->                 - RESP_TX_TO_FINAL_RX_DLY_UUS (R2DLY-FLEN)
 *                                                  <-FLEN--->       - FINAL_RX_TIMEOUT_UUS   (length of response frame)
 *                                    <----RDLY--------->            - RESP_RX_TO_FINAL_TX_DLY_UUS (depends on how quickly initiator
 *                                                                                                                      can turn around and reply)
 *
 * EXAMPLE 1: with SPI rate set to 18 MHz (default on this platform), and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 400uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 400uus (TXtoRX delays are set to 210uus)
 *            reducing the delays further can be achieved by using interrupt to handle the TX/RX events, or other code optimisations/faster SPI
 *
 * EXAMPLE 2: with SPI rate set to 4.5 MHz, and frame lengths of ~190 us, the delays can be set to:
 *            POLL_RX_TO_RESP_TX_DLY_UUS of 550uus, and RESP_RX_TO_FINAL_TX_DLY_UUS of 600uus (TXtoRX delays are set to 360 and 410 uus respectively)
 *
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    6.81 Mbps data rate used (around 220 us).
 * 6. The preamble timeout allows the receiver to stop listening in situations where preamble is not starting (which might be because the responder is
 *    out of range or did not receive the message to respond to). This saves the power waste of listening for a message that is not coming. We
 *    recommend a minimum preamble timeout of 5 PACs for short range applications and a larger value (e.g. in the range of 50% to 80% of the preamble
 *    length) for more challenging longer range, NLOS or noisy environments.
 * 7. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW IC OTP memory.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW IC User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 10. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *     automatically appended by the DW IC. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *     work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
 * 11. When running this example on the DWK3000 platform with the POLL_RX_TO_RESP_TX_DLY response delay provided, the dwt_starttx() is always
 *     successful. However, in cases where the delay is too short (or something else interrupts the code flow), then the dwt_starttx() might be issued
 *     too late for the configured start time. The code below provides an example of how to handle this condition: In this case it abandons the
 *     ranging exchange and simply goes back to awaiting another poll message. If this error handling code was not here, a late dwt_starttx() would
 *     result in the code flow getting stuck waiting subsequent RX event that will will never come. The companion "initiator" example (ex_05a) should
 *     timeout from awaiting the "response" and proceed to send another poll in due course to initiate another ranging exchange.
 * 12. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 13. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW IC API Guide for more details on the DW IC driver functions.
 * 14. In this example, the DW IC is put into IDLE state after calling dwt_initialise(). This means that a fast SPI rate of up to 36 MHz can be used
 *     thereafter.
 * 15. Desired configuration by user may be different to the current programmed configuration. dwt_configure is called to set desired
 *     configuration.
 ****************************************************************************************************************************************************/

