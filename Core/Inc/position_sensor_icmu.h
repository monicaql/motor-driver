/*
 * position_sensor_icmu.h
 *
 */

#ifndef INC_POSITION_SENSOR_ICMU_H_
#define INC_POSITION_SENSOR_ICMU_H_


//#include "structs.h"
#include "spi.h"
#include <stdint.h>

#define N_POS_SAMPLES 20		// Number of position samples to store.  should put this somewhere else...
#define N_LUT 128

typedef struct{
//	union{
//		uint8_t spi_tx_buff[2];
//		uint16_t spi_tx_word;
//	};
//	union{
//		uint8_t spi_rx_buff[2];
//		uint16_t spi_rx_word;
//	};
	union{
		uint8_t spi_tx_buff[1];
		uint8_t spi_tx_word;
	};
	union{
		uint8_t spi_rx_buff[1];
		uint8_t spi_rx_word;
	};
	uint8_t spi_tx_byte;
	uint8_t spi_rx_byte;
	float position, ElecPosition, ElecOffset, MechPosition, MechOffset, modPosition, oldModPosition, oldVel, velVec[40], MechVelocity, ElecVelocity, ElecVelocityFilt;
	int raw, lut_shift, _CPR, rotations, old_counts, _ppairs, first_sample, raw_bytes[8];
//		SPI *spi; // will have to change
//		DigitalOut *cs; // will have to change
	int readAngleCmd;
	int offset_lut[N_LUT];
} EncoderCMUStruct;

//    PositionSensoriCMU(int CPR, float offset, int ppairs);
void Init(EncoderCMUStruct *encoder, int CPR, float offset, int ppairs);
void Sample(EncoderCMUStruct * encoder,float dt);
void ZeroPosition(EncoderCMUStruct * encoder);
void WriteLUT(EncoderCMUStruct * encoder, int new_lut[128]);
void SetElecOffset(EncoderCMUStruct * encoder, float offset);
void SetMechOffset(EncoderCMUStruct * encoder, float offset);
void printCMUEncoder(EncoderCMUStruct * encoder);




#endif /*INC_POSITION_SENSOR_ICMU_H_ */
