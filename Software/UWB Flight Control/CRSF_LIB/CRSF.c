#include "Crsf.h"

uint8_t RxBuf[CRSF_MAX_PACKET_SIZE];		//Crsf���ݰ�������
uint8_t RxBuf_Index;

//uint16_t CRSF_CH[17];
//uint16_t CRSF_CH_LAST[17];

int CrsfChannels[CRSF_NUM_CHANNELS];		//Crsfͨ��ֵ
CrsfLinkStatistics_t LinkStatistics;

extern bool armed_drone;
extern bool collectpsi;
float r[3];
float mot_offset;
int throttle = 0;
int dshotMax = 2047;
extern TIM_HandleTypeDef htim2;
extern float psi_init;
extern bool UWB_data;
extern bool Uwbtoggleon;

extern bool setpoint_collected;
extern State_Vector state;


uint32_t GetTick(void) {
	return uwTick;
}
uint32_t get_ticks() {
	uint32_t ticks = __HAL_TIM_GET_COUNTER(&htim2);
	return ticks;
}


int elrs_error = 0;
float h_p = 1792; //previous toggle command

const int validValuesCh4to7[] = { 191, 1004, 1792 };
const int validValuesCh8to9[] = { 191, 1792 };

// Function to check if a value is valid for a given range
int isValidValue(int value, const int validValues[], int size) {
	for (int i = 0; i < size; i++) {
		if (value == validValues[i]) {
			return 1; // Valid value found
		}
	}
	return 0; // Value not found in the valid values array
}

extern void test_run_info(unsigned char *data);
extern float linear_interp(uint32_t x, uint32_t in_min, uint32_t in_max, float out_min,float out_max);
extern uint32_t map_angle(uint32_t x, uint32_t in_min, uint32_t in_max,	uint32_t out_min, uint32_t out_max);
char txStats[200];

extern int armed_status;

extern bool YawOn;


void HandleByteReceived(void) {
	bool reprocess;
	do {
		reprocess = false;
		if (RxBuf_Index > 1) {
			uint8_t len = RxBuf[1];
			// Sanity check the declared length isn't outside Type + X{1,CRSF_MAX_PAYLOAD_LEN} + CRC
			// assumes there never will be a CRSF message that just has a type and no data (X)
			if (len < 3 || len > (CRSF_MAX_PAYLOAD_LEN + 2)) {
				ShiftRxBuffer(1);
				reprocess = true;
			} else if (RxBuf_Index >= (len + 2)) {
				uint8_t inCrc = RxBuf[2 + len - 1];
				uint8_t crc = Calc(&RxBuf[2], len - 1);
				if (crc == inCrc) {
					ProcessPacketIn();
					ShiftRxBuffer(len + 2);
					reprocess = true;
				} else {
					ShiftRxBuffer(1);
					reprocess = true;
				}
			}  // if complete packet
		} // if pos > 1
	} while (reprocess);
}

void ShiftRxBuffer(uint8_t cnt) {
	// If removing the whole thing, just set pos to 0
	if (cnt >= RxBuf_Index) {
		RxBuf_Index = 0;
		return;
	}
	// Otherwise do the slow shift down
	uint8_t *src = &RxBuf[cnt];
	uint8_t *dst = &RxBuf[0];
	RxBuf_Index -= cnt;
	uint8_t left = RxBuf_Index;
	while (left--)
		*dst++ = *src++;
}

void ProcessPacketIn(void)										//���ݰ����ദ��
{
	const Crsf_Header_t *hdr = (Crsf_Header_t*) RxBuf;
	if (hdr->device_addr == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
		switch (hdr->type) {
//        case CRSF_FRAMETYPE_GPS:									//GPS����
//            packetGps(hdr);
//            break;
		case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:			//ң����ͨ��ֵ����
			PacketChannelsPacked(hdr);
			break;
		case CRSF_FRAMETYPE_LINK_STATISTICS:				//������Ϣ
			PacketLinkStatistics(hdr);
			break;
		}// CRSF_ADDRESS_FLIGHT_CONTROLLER
	}
//	else if (hdr->device_addr == ){
//		PacketLinkStatistics(hdr);
//	}
}

void PacketChannelsPacked(const Crsf_Header_t *p) {

	crsf_channels_t *ch = (crsf_channels_t*) &p->data;


	//Check the buttons and switches return correct values, only return correct values when data is not corrupted
	//Therefore checking buttons and switches ensures only correct throttle, roll etc data is passed to control loop
	if (isValidValue(ch->ch4, validValuesCh4to7,
			sizeof(validValuesCh4to7) / sizeof(validValuesCh4to7[0]))) {
		if (isValidValue(ch->ch5, validValuesCh4to7,
				sizeof(validValuesCh4to7) / sizeof(validValuesCh4to7[0]))) {
			if (isValidValue(ch->ch6, validValuesCh4to7,
					sizeof(validValuesCh4to7) / sizeof(validValuesCh4to7[0]))) {
				if (isValidValue(ch->ch7, validValuesCh4to7,
						sizeof(validValuesCh4to7)
								/ sizeof(validValuesCh4to7[0]))) {
					if (isValidValue(ch->ch8, validValuesCh8to9,
							sizeof(validValuesCh8to9)
									/ sizeof(validValuesCh8to9[0]))) {
						if (isValidValue(ch->ch9, validValuesCh8to9,
								sizeof(validValuesCh8to9)
										/ sizeof(validValuesCh8to9[0]))) {

							CrsfChannels[0] = ch->ch0;
							CrsfChannels[1] = ch->ch1;
							CrsfChannels[2] = ch->ch2;
							CrsfChannels[3] = ch->ch3;
							CrsfChannels[4] = ch->ch4;
							CrsfChannels[5] = ch->ch5;
							CrsfChannels[6] = ch->ch6;
							CrsfChannels[7] = ch->ch7;
							CrsfChannels[8] = ch->ch8;
							CrsfChannels[9] = ch->ch9;
							CrsfChannels[10] = ch->ch10;
							CrsfChannels[11] = ch->ch11;
							CrsfChannels[12] = ch->ch12;
							CrsfChannels[13] = ch->ch13;
							CrsfChannels[14] = ch->ch14;
							CrsfChannels[15] = ch->ch15;


							if (CrsfChannels[0] < xmin + 50) {
								CrsfChannels[0] = xmin + 50;

							} else if (CrsfChannels[0] > xmax - 50) {
								CrsfChannels[0] = xmax - 50;
							}
							if (CrsfChannels[1] < xmin + 50) {
								CrsfChannels[1] = xmin + 50;

							} else if (CrsfChannels[1] > xmax - 50) {
								CrsfChannels[1] = xmax - 50;
							}
							if (CrsfChannels[3] < xmin + 50) {
								CrsfChannels[3] = xmin + 50;

							} else if (CrsfChannels[3] > xmax - 50) {
								CrsfChannels[3] = xmax - 50;
							}

							//r = commanded value

#ifdef velControl
						 float rollpitch_max = 0.5;
														r[0] = linear_interp(CrsfChannels[0], xmin + 50, xmax - 50, -rollpitch_max*100, rollpitch_max*100)/100;
														r[1] = linear_interp(CrsfChannels[1], xmin + 50, xmax - 50, -rollpitch_max, rollpitch_max);

							#else
							int rollpitch_max = 20;
														r[0] = linear_interp(CrsfChannels[0], xmin + 50, xmax - 50, -rollpitch_max, rollpitch_max);
														r[1] = linear_interp(CrsfChannels[1], xmin + 50, xmax - 50, -rollpitch_max, rollpitch_max);
#endif

#ifdef AltitudeHoldChanges

													if ((UWB_data == true) && (setpoint_collected == true)) {

														float tempvar = CrsfChannels[2] / (xmax-xmin);

														if (tempvar > 0.6) {
															state.ZvelSetPoint = map_angle(CrsfChannels[2], xmax*0.6, xmax, 0, MAX_V);
															state.ZSP_changing = true;
														} else if (tempvar < 0.4) {
															state.ZvelSetPoint = map_angle(CrsfChannels[2], xmin, xmax*0.4, -MAX_V, 0);
															state.ZSP_changing = true;

														} else  {

															if (state.ZSP_changing == true){
																state.UWB_setpoint[2] = state.UWB_XY[2];

																throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, MAX_THROTTLE);
																throttleUWB_setP = throttle;

															}



															state.ZvelSetPoint = 0;
															state.ZSP_changing = false;

														}

													} else  {

														throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, MAX_THROTTLE);
														if (throttle > throttleHover) {
															throttle = throttleHover;
														}
													}
#else
													throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, MAX_THROTTLE);
													if (throttle > throttleHover) {
														throttle = throttleHover;
													}
#endif
						//	throttle = map_angle(CrsfChannels[2], xmin, xmax, 0, 0.5);
							r[2] = linear_interp(CrsfChannels[3], xmin + 50, xmax - 50, -250, 250);
						//	r[2] = linear_interp(CrsfChannels[3], xmin + 50, xmax - 50, -250, 250)-150;

							mot_offset =  linear_interp(CrsfChannels[10], 191 , 1792 , 0, 100);

//							if ((r[0] < 0.5) && (r[0] > -0.5)) {
//								r[0] = 0;
//							}
//							if ((r[1] < 0.5) && (r[1] > -0.5)) {
//								r[1] = 0;
//							}
							if ((r[2] < 0.5) && (r[2] > -0.5)) {
								r[2] = 0;
							}
						}
					}
				}
			}
		}
	} else {
		elrs_error++;

	}
	/*
	 *
	 * Write the safety feature here
	 *
	 * button A = channel[8]
	 * switch B = channel[6]
	 *
	 *
	 * throttle must be 0 before arming, button A and switch B must be true to arm,
	 * disarming is done through switch B being false, drone must disarm when radio disconnects
	 *
	 *
	 *
	 */

	if ((CrsfChannels[6] == 191) && (armed_drone == true)) { //if switch not activated

		armed_drone = false;
		armed_status = QUAD_DISARMED;
	}


#ifdef UWBtoggleSwitch
				if (CrsfChannels[5] == 1792) {
					UWB_data = true;
					if (CrsfChannels[5] != h_p) { //if toggle is high (UWB) and last toggle was low (Manual), trigger flag for new uwb setpoint
						Uwbtoggleon = true;
					}

				} else {
					UWB_data = false;




				}

				h_p = CrsfChannels[5];
#endif

				//switch or button = 191, 1792

				if (armed_drone == false) {
		if ((CrsfChannels[8] == 1792) && (CrsfChannels[6] == 1792)
				&& (CrsfChannels[2] < 200)) { //Check throttle = 0, button A is pressed, switch B is toggled

			//		uint32_t arm_time = get_ticks();

			// 		uint32_t toggle_time = get_ticks();
			// 		uint32_t currentT = get_ticks();
			/*
			 * removed due to timer issue
			 *
			 * while ((currentT - arm_time) < 5000) { //Wait for 5s, toggle the leds at 250ms then increase rate to 125ms after 2.5s

			 currentT = get_ticks();
			 if ((currentT - arm_time) < 2500) {
			 if ((currentT - toggle_time) > 250){
			 toggle_time = currentT;

			 HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			 HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			 HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			 }
			 } else {
			 if ((currentT - toggle_time) > 125){
			 toggle_time = currentT;

			 HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			 HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			 HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			 }
			 }
			 }*/

			if (CrsfChannels[2] > 1000) {
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);

			}

			if ((CrsfChannels[8] == 1792) && (CrsfChannels[6] == 1792)
					&& (CrsfChannels[2] < 200)) { //same check

				armed_drone = true; //update status to armed if true for 5s
				collectpsi = true;
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
				armed_status = QUAD_ARMED;


			} else {
				armed_drone = false;
				armed_status = QUAD_DISARMED;
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

			}

		} else {
			armed_drone = false;
			armed_status = QUAD_DISARMED;
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

		}

	}

}

void PacketLinkStatistics(const Crsf_Header_t *p) {
	const CrsfLinkStatistics_t *link = (CrsfLinkStatistics_t*) p->data;
	memcpy(&LinkStatistics, &link, sizeof(LinkStatistics));
	onPacketLinkStatistics(&LinkStatistics);
}

//void Get_Crsf_CH(void)
//{
//		CRSF_CH[1]  = ((int16_t)RxBuf[ 3] >> 0 | ((int16_t)RxBuf[ 4] << 8 )) & 0x07FF;
//		CRSF_CH[2] = ((int16_t)RxBuf[ 4] >> 3 | ((int16_t)RxBuf[ 5] << 5 )) & 0x07FF;
//		CRSF_CH[3]  = ((int16_t)RxBuf[ 5] >> 6 | ((int16_t)RxBuf[ 6] << 2 ) | (int16_t)RxBuf[ 7] << 10 ) & 0x07FF;
//		CRSF_CH[4]  = ((int16_t)RxBuf[ 7] >> 1 | ((int16_t)RxBuf[ 8] << 7 )) & 0x07FF;
//		CRSF_CH[5] = ((int16_t)RxBuf[ 8] >> 4 | ((int16_t)RxBuf[ 9] << 4 )) & 0x07FF;
//		CRSF_CH[6]  = ((int16_t)RxBuf[ 9] >> 7 | ((int16_t)RxBuf[10] << 1 ) | (int16_t)RxBuf[11] << 9 ) & 0x07FF;
//		CRSF_CH[7]  = ((int16_t)RxBuf[11] >> 2 | ((int16_t)RxBuf[12] << 6 )) & 0x07FF;
//		CRSF_CH[8]  = ((int16_t)RxBuf[12] >> 5 | ((int16_t)RxBuf[13] << 3 )) & 0x07FF;
//		CRSF_CH[9]	 = ((int16_t)RxBuf[14] << 0 | ((int16_t)RxBuf[15] << 8 )) & 0x07FF;
//		CRSF_CH[10] = ((int16_t)RxBuf[15] >> 3 | ((int16_t)RxBuf[16] << 5 )) & 0x07FF;
//		CRSF_CH[11] = ((int16_t)RxBuf[16] >> 6 | ((int16_t)RxBuf[17] << 2 ) | (int16_t)RxBuf[18] << 10 ) & 0x07FF;
//		CRSF_CH[12] = ((int16_t)RxBuf[18] >> 1 | ((int16_t)RxBuf[19] << 7 )) & 0x07FF;
//		CRSF_CH[13] = ((int16_t)RxBuf[19] >> 4 | ((int16_t)RxBuf[20] << 4 )) & 0x07FF;
//		CRSF_CH[14] = ((int16_t)RxBuf[20] >> 7 | ((int16_t)RxBuf[21] << 1 ) | (int16_t)RxBuf[22] << 9 ) & 0x07FF;
//		CRSF_CH[15] = ((int16_t)RxBuf[22] >> 2 | ((int16_t)RxBuf[23] << 6 )) & 0x07FF;
//		CRSF_CH[16] = ((int16_t)RxBuf[23] >> 5 | ((int16_t)RxBuf[24] << 3 )) & 0x07FF;				
//	
//		for(int i=1;i<17;i++)
//		{
//				if((CRSF_CH[i]<CRSF_CHANNEL_VALUE_MIN)||((CRSF_CH[i]>CRSF_CHANNEL_VALUE_MAX))&&(CRSF_CH[i]!=0))
//				CRSF_CH[i] = CRSF_CH_LAST[i];				
//				CRSF_CH_LAST[i] = CRSF_CH[i];
//		}
//}

static void onPacketLinkStatistics(CrsfLinkStatistics_t *link)//�ش��ź���������ѹ...
{
	//printf("Link_quality:%d\n",link->uplink_Link_quality);

	//	sprintf(txStats, "Link_quality:%d, RSSI_1:%d, RSSI_2:%d, SNR: %d, roll:%d, pitch: %d, throttle:%d, yaw:%d  \n",link->uplink_Link_quality, link->uplink_RSSI_1,link->uplink_RSSI_2, link->uplink_SNR, CrsfChannels[0],CrsfChannels[1],CrsfChannels[2],CrsfChannels[3] );

	sprintf(txStats, "%d,%d,%d\r\n", link->uplink_Link_quality,
			link->uplink_RSSI_1, link->uplink_SNR);
//test_run_info((unsigned char *)txStats);

	if (link->uplink_Link_quality == 0) {
		armed_drone = false;
		armed_status = QUAD_BAD_LINK;
	//	HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);


	}

}

