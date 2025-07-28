/*
 * GNSS.c
 *
 *  Created on: 03.10.2020
 *      Author: SimpleMethod
 *
 *Copyright 2020 SimpleMethod
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy of
 *this software and associated documentation files (the "Software"), to deal in
 *the Software without restriction, including without limitation the rights to
 *use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 *of the Software, and to permit persons to whom the Software is furnished to do
 *so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE SOFTWARE.
 ******************************************************************************
 */

#include "GNSS.h"
#include "stdlib.h"
union u_Short uShort;
union i_Short iShort;
union u_Long uLong;
union i_Long iLong;

GNSS_StateHandle GNSS_Handle;
LAT_LONG lat;
LAT_LONG lon;

//extern GNSS_StateHandle GNSS_Handle;

/*!
 * Structure initialization.
 * @param GNSS Pointer to main GNSS structure.
 * @param huart Pointer to uart handle.
 */
void GNSS_Init(GNSS_StateHandle *GNSS, UART_HandleTypeDef *huart) {
	GNSS->huart = huart;
	memset(GNSS->month, 0, sizeof(GNSS->month));
	memset(GNSS->day, 0, sizeof(GNSS->day));
	memset(GNSS->hour, 0, sizeof(GNSS->hour));
	memset(GNSS->min, 0, sizeof(GNSS->min));
	memset(GNSS->sec, 0, sizeof(GNSS->sec));
	GNSS->fixType = 0;
	GNSS->lon = 0;
	GNSS->lat = 0;
	GNSS->height = 0;
	GNSS->hMSL = 0;
	GNSS->hAcc = 0;
	GNSS->vAcc = 0;
	GNSS->gSpeed = 0;
	GNSS->headMot = 0;
}
int var_init;
/*!
 * Searching for a header in data buffer and matching class and message ID to buffer data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_ParseBuffer(GNSS_StateHandle *GNSS) {

	for (int var = 0; var <= 100; ++var) {
		if (GNSS->uartWorkingBuffer[var] == 0xB5
				&& GNSS->uartWorkingBuffer[var + 1] == 0x62) {
			if (GNSS->uartWorkingBuffer[var + 2] == 0x27
					&& GNSS->uartWorkingBuffer[var + 3] == 0x03) { //Look at: 32.19.1.1 u-blox 8 Receiver description
				GNSS_ParseUniqID(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x21) { //Look at: 32.17.14.1 u-blox 8 Receiver description
				GNSS_ParseNavigatorData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x07) { //ook at: 32.17.30.1 u-blox 8 Receiver description

				 var_init  = var + 6;
				 GNSS_ParsePVTData(GNSS);
			} else if (GNSS->uartWorkingBuffer[var + 2] == 0x01
					&& GNSS->uartWorkingBuffer[var + 3] == 0x02) { // Look at: 32.17.15.1 u-blox 8 Receiver description
				GNSS_ParsePOSLLHData(GNSS);
			}
		}
	}

/*
int index[2];

//Loop the buffer looking for sentence start index = $G.. and end index \r\n
	for (int var = 0; var <= 100; ++var) {
		if ((GNSS->uartWorkingBuffer[var] == '$') && (GNSS->uartWorkingBuffer[var+1] == 'G')) {
			int end = var;
			for (end = var; end < 100; end++){
				if ((GNSS->uartWorkingBuffer[end] == '\r') && (GNSS->uartWorkingBuffer[end+1] == '\n')){
					end++;
					break;
				}
			}
			char NMEAstr[4] = {(char)GNSS->uartWorkingBuffer[var+3], (char)GNSS->uartWorkingBuffer[var+4], (char)GNSS->uartWorkingBuffer[var+5], '\0'};

			index[0] = var;
			index[1] = end+1;
			//memcpy(GNSS->sentence, GNSS->uartWorkingBuffer + var, end+1-var);//copies buffer from start index to last index



	char sentence[index[1]- index[0]];
	memcpy(sentence, GNSS->uartWorkingBuffer + index[0], index[1]-index[0]);

			char *token;
			 int fieldIndex = 0;
			token = strtok(sentence + 1, ",");

			    while (token != NULL && fieldIndex < 16) {
			        strcpy(GNSS->field[fieldIndex], token);
			        fieldIndex++;
			        token = strtok(NULL, ",");
			    }


			    if (strcmp(NMEAstr,"RMC") == 0){

			    	if (strcmp(GNSS->field[2], "A") == 0 ) {


				memcpy(GNSS->hour, GNSS->field[1], 2);
			memcpy(GNSS->min, GNSS->field[1]+2, 2);
			memcpy(GNSS->sec, GNSS->field[1]+4, 2);

			sscanf(GNSS->field[3], "%2d%lf", &lat.degrees, &lat.minutes);
				  GNSS->fLat = lat.degrees + lat.minutes/60.0;
				  if (strcmp(GNSS->field[4],"S")==0){
					GNSS->fLat *= -1;
				  }

				sscanf(GNSS->field[5], "%3d%lf", &lon.degrees, &lon.minutes);
				GNSS->fLon = lon.degrees + lon.minutes/60.0;
				if (strcmp(GNSS->field[6],"W")==0){
					GNSS->fLon *= -1;
				}

				GNSS->gSpeed = atof(GNSS->field[7]) * 0.51444;
				GNSS->course = atof(GNSS->field[8]);

				memcpy(GNSS->day, GNSS->field[9], 2);
				memcpy(GNSS->month, GNSS->field[9]+2, 2);
				memcpy(GNSS->year, GNSS->field[9]+4, 2);

			    	}



			} else if (strcmp(NMEAstr, "GGA")== 0) {

				if (GNSS->field[6] == 1) {

				memcpy(GNSS->hour, GNSS->field[1], 2);
				memcpy(GNSS->min, GNSS->field[1]+2, 2);
				memcpy(GNSS->sec, GNSS->field[1]+4, 2);

				sscanf(GNSS->field[2], "%2d%lf", &lat.degrees, &lat.minutes);
	    		  GNSS->fLat = lat.degrees + lat.minutes/60.0;
				  if (strcmp(GNSS->field[3],"S")==0){
					GNSS->fLat *= -1;
				 }

				sscanf(GNSS->field[4], "%3d%lf", &lon.degrees, &lon.minutes);
				GNSS->fLon = lon.degrees + lon.minutes/60.0;
				if (strcmp(GNSS->field[5],"W")==0){
					GNSS->fLon *= -1;
					}

				GNSS->sats = atof(GNSS->field[7]);
				GNSS->altitude = atof(GNSS->field[8]);

				}



			} else if (strcmp(NMEAstr, "GLL")==0){

				if (strcmp(GNSS->field[6], "A") == 0) {

				sscanf(GNSS->field[1], "%2d%lf", &lat.degrees, &lat.minutes);
				 GNSS->fLat = lat.degrees + lat.minutes/60.0;
				  if (strcmp(GNSS->field[2],"S")==0){
					  GNSS->fLat *= -1;
					 }

				sscanf(GNSS->field[3], "%3d%lf", &lon.degrees, &lon.minutes);
				GNSS->fLon = lon.degrees + lon.minutes/60.0;
				if (strcmp(GNSS->field[4],"W")==0){
					GNSS->fLon *= -1;
				 }

				memcpy(GNSS->hour, GNSS->field[5], 2);
				memcpy(GNSS->min, GNSS->field[5]+2, 2);
				memcpy(GNSS->sec, GNSS->field[5]+4, 2);


				}

			}

				break;
				}
			}
*/
}




/*!
 * Make request for unique chip ID data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetUniqID(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getDeviceID,
			sizeof(getDeviceID) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 17);
}

/*!
 * Make request for UTC time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetNavigatorData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getNavigatorData,
			sizeof(getNavigatorData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 28);
}

/*!
 * Make request for geodetic position solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPOSLLHData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getPOSLLHData,
			sizeof(getPOSLLHData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 36);
}

/*!
 * Make request for navigation position velocity time solution data.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_GetPVTData(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, getPVTData,
			sizeof(getPVTData) / sizeof(uint8_t));
	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer,100);
}

/*!
 * Parse data to unique chip ID standard.
 * Look at: 32.19.1.1 u-blox 8 Receiver description
 * @param GNSS Pointer to main GNSS structure.
 */

void GNSS_ParseUniqID(GNSS_StateHandle *GNSS) {
	for (int var = 0; var < 5; ++var) {
		GNSS->uniqueID[var] = GNSS_Handle.uartWorkingBuffer[10 + var];
	}
}

/*!
 * Changing the GNSS mode.
 * Look at: 32.10.19 u-blox 8 Receiver description
 */
void GNSS_SetMode(GNSS_StateHandle *GNSS, short gnssMode) {
	if (gnssMode == 0) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPortableMode,sizeof(setPortableMode) / sizeof(uint8_t));
	} else if (gnssMode == 1) {
		HAL_UART_Transmit_DMA(GNSS->huart, setStationaryMode,sizeof(setStationaryMode) / sizeof(uint8_t));
	} else if (gnssMode == 2) {
		HAL_UART_Transmit_DMA(GNSS->huart, setPedestrianMode,sizeof(setPedestrianMode) / sizeof(uint8_t));
	} else if (gnssMode == 3) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 4) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAutomotiveMode,sizeof(setAutomotiveMode) / sizeof(uint8_t));
	} else if (gnssMode == 5) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone1GMode,sizeof(setAirbone1GMode) / sizeof(uint8_t));
	} else if (gnssMode == 6) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone2GMode,sizeof(setAirbone2GMode) / sizeof(uint8_t));
	} else if (gnssMode == 7) {
		HAL_UART_Transmit_DMA(GNSS->huart, setAirbone4GMode,sizeof(setAirbone4GMode) / sizeof(uint8_t));
	} else if (gnssMode == 8) {
		HAL_UART_Transmit_DMA(GNSS->huart, setWirstMode,sizeof(setWirstMode) / sizeof(uint8_t));
	} else if (gnssMode == 9) {
		HAL_UART_Transmit_DMA(GNSS->huart, setBikeMode,sizeof(setBikeMode) / sizeof(uint8_t));
	}
}
/*!
 * Parse data to navigation position velocity time solution standard.
 * Look at: 32.17.15.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */


void GNSS_ParsePVTData(GNSS_StateHandle *GNSS) {



/*	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[10];
	GNSS->yearBytes[0]=GNSS_Handle.uartWorkingBuffer[10];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[11];
	GNSS->yearBytes[1]=GNSS_Handle.uartWorkingBuffer[11];
	GNSS->year = uShort.uShort;
	GNSS->month = GNSS_Handle.uartWorkingBuffer[12];
	GNSS->day = GNSS_Handle.uartWorkingBuffer[13];
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[14];
	GNSS->min = GNSS_Handle.uartWorkingBuffer[15];
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[16];
	GNSS->timeValid = GNSS_Handle.uartWorkingBuffer[17];

	GNSS->fixType = GNSS_Handle.uartWorkingBuffer[26];
	//new - numSV - Number of satellites used in Nav Solution
	GNSS->numSV = GNSS_Handle.uartWorkingBuffer[29];
*/
 if (var_init <= 32) {


	 uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[ 4 + var_init];
	 	GNSS->yearBytes[0]=GNSS_Handle.uartWorkingBuffer[4 + var_init];
	 	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[5 + var_init];
	 	GNSS->yearBytes[1]=GNSS_Handle.uartWorkingBuffer[5 + var_init];

//		GNSS->year = uShort.uShort;

	 	GNSS->month = GNSS_Handle.uartWorkingBuffer[6 + var_init];
	 	GNSS->day = GNSS_Handle.uartWorkingBuffer[7 + var_init];
	 	GNSS->hour = GNSS_Handle.uartWorkingBuffer[8 + var_init];
	 	GNSS->min = GNSS_Handle.uartWorkingBuffer[9 + var_init];
	 	GNSS->sec = GNSS_Handle.uartWorkingBuffer[10 + var_init];
	 //	GNSS->timeValid = GNSS_Handle.uartWorkingBuffer[17];

	 	GNSS->fixType = GNSS_Handle.uartWorkingBuffer[20 + var_init];
	 	//new - numSV - Number of satellites used in Nav Solution
	 	GNSS->numSV = GNSS_Handle.uartWorkingBuffer[23 + var_init];


	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 24 + var_init];
		GNSS->lonBytes[var]= GNSS_Handle.uartWorkingBuffer[var + 24 + var_init];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 28 + var_init];
		GNSS->latBytes[var]=GNSS_Handle.uartWorkingBuffer[var + 28 + var_init];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 32 + var_init];
	}
	GNSS->height = iLong.iLong/1000;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 36 + var_init];
		GNSS->hMSLBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 36 + var_init];
	}
	GNSS->hMSL = iLong.iLong/1000;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 40 + var_init];
	}
	GNSS->hAcc = (float)uLong.uLong/1000.0;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 44 + var_init];
	}
	GNSS->vAcc = uLong.uLong;


	//new velN - NED north velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 48 + var_init];
	}
	GNSS->velN = (float)iLong.iLong/1000.0;

	//new velE - NED east velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 52 + var_init];
	}
	GNSS->velE = (float)iLong.iLong/1000.0;

	//new velD - NED down velocity
	for (uint8_t var = 0; var < 4; ++var){
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 56 + var_init];
	}
	GNSS->velD = (float)iLong.iLong/1000.0;



	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 60 + var_init];
		GNSS->gSpeedBytes[var] = GNSS_Handle.uartWorkingBuffer[var + 60 + var_init];
	}
	GNSS->gSpeed = (float)iLong.iLong/1000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 64 + var_init];
	}
	GNSS->headMot = iLong.iLong;
	GNSS->course = (float)GNSS->headMot* 1e-5;

 }

}

/*!
 * Parse data to UTC time solution standard.
 * Look at: 32.17.30.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */

void GNSS_ParseNavigatorData(GNSS_StateHandle *GNSS) {
	uShort.bytes[0] = GNSS_Handle.uartWorkingBuffer[18];
	uShort.bytes[1] = GNSS_Handle.uartWorkingBuffer[19];
/*	GNSS->year = uShort.uShort;
	GNSS->month = GNSS_Handle.uartWorkingBuffer[20];
	GNSS->day = GNSS_Handle.uartWorkingBuffer[21];
	GNSS->hour = GNSS_Handle.uartWorkingBuffer[22];
	GNSS->min = GNSS_Handle.uartWorkingBuffer[23];
	GNSS->sec = GNSS_Handle.uartWorkingBuffer[24];*/
}

/*!
 * Parse data to geodetic position solution standard.
 * Look at: 32.17.14.1 u-blox 8 Receiver description.
 * @param GNSS Pointer to main GNSS structure.
 */


void GNSS_ParsePOSLLHData(GNSS_StateHandle *GNSS) {
	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 10];
	}
	GNSS->lon = iLong.iLong;
	GNSS->fLon=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 14];
	}
	GNSS->lat = iLong.iLong;
	GNSS->fLat=(float)iLong.iLong/10000000.0;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 18];
	}
	GNSS->height = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		iLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 22];
	}
	GNSS->hMSL = iLong.iLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 26];
	}
	GNSS->hAcc = uLong.uLong;

	for (int var = 0; var < 4; ++var) {
		uLong.bytes[var] = GNSS_Handle.uartWorkingBuffer[var + 30];
	}
	GNSS->vAcc = uLong.uLong;
}

/*!
 *  Sends the basic configuration: Activation of the UBX standard, change of NMEA version to 4.10 and turn on of the Galileo system.
 * @param GNSS Pointer to main GNSS structure.
 */
void GNSS_LoadConfig(GNSS_StateHandle *GNSS) {
	HAL_UART_Transmit_DMA(GNSS->huart, configUBX,
			sizeof(configUBX) / sizeof(uint8_t));

	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);
	HAL_Delay(250);

	HAL_UART_Transmit_DMA(GNSS->huart, setNMEA410,
			sizeof(setNMEA410) / sizeof(uint8_t));

	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);

	HAL_Delay(250);

	HAL_UART_Transmit_DMA(GNSS->huart, setGNSS,
			sizeof(setGNSS) / sizeof(uint8_t));

	HAL_UART_Receive_IT(GNSS->huart, GNSS_Handle.uartWorkingBuffer, 10);
	HAL_Delay(250);


}



/*!
 *  Creates a checksum based on UBX standard.
 * @param class Class value from UBX doc.
 * @param messageID MessageID value from UBX doc.
 * @param dataLength Data length value from UBX doc.
 * @param payload Just payload.
 * @return  Returns checksum.
 */
uint8_t GNSS_Checksum(uint8_t class, uint8_t messageID, uint8_t dataLength,uint8_t *payload) {
//todo: Look at 32.4 UBX Checksum
	return 0;
}
