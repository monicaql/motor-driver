/*
 * position_sensor_icmu.c
 *
 *
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor_icmu.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"
#include <sys/time.h>

struct timeval tv_start;
struct timeval tv_end;


void Init(EncoderCMUStruct *encoder, int CPR, float offset, int ppairs) {
	printf("init encoder cmu");
	encoder->_CPR = CPR;
	encoder->_ppairs = ppairs;
	encoder->ElecOffset = offset;
	encoder->rotations = 0;
	encoder->readAngleCmd = 0xffff;
	encoder->MechOffset = offset;
	encoder->modPosition = 0;
	encoder->oldModPosition = 0;
	encoder->oldVel = 0;
	encoder->raw = 0;
	encoder->lut_shift = 12; // shift from 19bit data to 7bit lut index
	encoder->first_sample = 0;

	for(int i = 0; i<400; i++){
		if (i%4 == 0){ // alternate between A600 and 0000 to get A6 00 00 00
			encoder->spi_tx_byte = 0xA6;
		} else {
			encoder->spi_tx_byte = 0x00;
		}
		printf("encoder tx: %x \n\r", (unsigned int)encoder->spi_tx_byte);

		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t *)&encoder->spi_tx_byte, (uint8_t *)&encoder->spi_rx_byte, 1, 100);
//		printf("encoder rx: %x \n\r", (unsigned int)encoder->spi_rx_byte);
		printf("encoder rx: %d \n\r", (int)encoder->spi_rx_byte);
//		HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)encoder->spi_tx_byte, 1, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high


	}

}

void Sample(EncoderCMUStruct * encoder, float dt){
//	unsigned long t1 = DWT->CYCCNT;
    /* SPI read/write */
	for(int i = 0; i<4; i++){
		if (i%4 == 0){ // alternate between A600 and 0000 to get A6 00 00 00
			encoder->spi_tx_byte = 0xA6;
		} else {
			encoder->spi_tx_byte = 0x00;
		}
//		printf("encoder tx: %x \n\r", (unsigned int)encoder->spi_tx_byte);

		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//		HAL_SPI_Transmit(&ENC_SPI, (uint8_t*)&encoder->spi_tx_byte, 1, 100);
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t *)&encoder->spi_tx_byte, (uint8_t *)&encoder->spi_rx_byte, 1, 2000);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
		encoder->raw_bytes[i] = encoder -> spi_rx_byte;
//		printf("encoder rx: %x \n\r", (unsigned int)encoder->spi_rx_byte);
	}


	encoder->raw = encoder->raw_bytes[1]<<16 | encoder->raw_bytes[2]<<8 | encoder->raw_bytes[3];
	// From 24 received bits: 14bits MAS, 5 bits NON, 5 zero bits
	// With default filter, get 14 bits of usable data
	// But, still extract 19 bits
	encoder->raw = encoder->raw>>5;
//	printf("encoder raw: %d \n\r", (unsigned int)encoder->raw);
//	printf("encoder raw: %x  %x  %x  %x \n\r", (unsigned int)encoder->raw_bytes[3], (unsigned int)encoder->raw_bytes[2], (unsigned int)encoder->raw_bytes[1], (unsigned int)encoder->raw_bytes[0]);


	//GPIOA->ODR |= (1 << 15);
	int off_1 = encoder->offset_lut[encoder->raw>>encoder->lut_shift]; // shift 12 bits to go from 19-bit to 7-bit lut index
	int off_2 = encoder->offset_lut[((encoder->raw>>encoder->lut_shift)+1)%128];


	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>encoder->lut_shift)<<encoder->lut_shift))>>encoder->lut_shift);        // Interpolate between lookup table entries
	int angle = encoder->raw + off_interp;                                               // Correct for nonlinearity with lookup table from calibration

	if(encoder->first_sample){
		if(angle - encoder->old_counts > encoder->_CPR/2){
			encoder->rotations -= 1;
			}
		else if (angle - encoder->old_counts < -(encoder->_CPR)/2){
			encoder->rotations += 1;
			}
	}
	if(!encoder->first_sample){encoder->first_sample = 1;}

	encoder->old_counts = angle;
	encoder->oldModPosition = encoder->modPosition;
	encoder->modPosition = ((2.0f*PI_F * ((float) angle))/ (float)encoder->_CPR);
	encoder->position = (2.0f*PI_F * ((float) angle+(encoder->_CPR*encoder->rotations)))/ (float)encoder->_CPR;

	encoder->MechPosition = encoder->position - encoder->MechOffset; // is this mech position of the rotor or output?

	float elec = ((2.0f*PI_F/(float)encoder->_CPR) * (float) ((encoder->_ppairs*angle)%encoder->_CPR)) + encoder->ElecOffset;
	if(elec < 0) elec += 2.0f*PI_F;
	else if(elec > 2.0f*PI_F) elec -= 2.0f*PI_F ;
	encoder->ElecPosition = elec;

	float vel;
	//if(modPosition<.1f && oldModPosition>6.1f){

	if((encoder->modPosition-encoder->oldModPosition) < -3.0f){
		vel = (encoder->modPosition - encoder->oldModPosition + 2.0f*PI_F)/dt;
		}
	//else if(modPosition>6.1f && oldModPosition<0.1f){
	else if((encoder->modPosition - encoder->oldModPosition) > 3.0f){
		vel = (encoder->modPosition - encoder->oldModPosition - 2.0f*PI_F)/dt;
		}
	else{
		vel = (encoder->modPosition-encoder->oldModPosition)/dt;
	}

	int n = 40;
	float sum = vel;
	for (int i = 1; i < (n); i++){
		encoder->velVec[n - i] = encoder->velVec[n-i-1];
		sum += encoder->velVec[n-i];
		}
	encoder->velVec[0] = vel;
	encoder->MechVelocity =  sum/((float)n); // is this mech velocity of the rotor or output?

	encoder->ElecVelocity = encoder->MechVelocity*encoder->_ppairs;
	encoder->ElecVelocityFilt = 0.99f*encoder->ElecVelocityFilt + 0.01f*encoder->ElecVelocity;

//	unsigned long t2 = DWT->CYCCNT;
//	unsigned long diff = t2 - t1;
//	printf("diff: %d", (int)diff);
}


void ZeroPosition(EncoderCMUStruct * encoder){
	encoder->rotations = 0;
	encoder->MechOffset = 0;
	Sample(encoder, .00025f);
	encoder->MechOffset = encoder->MechPosition;

}
//
//void WriteLUT(EncoderCMUStruct * encoder,  int new_lut[128]){
//	memcpy(encoder->offset_lut, new_lut, sizeof(encoder->offset_lut));
//
//}

void printCMUEncoder(EncoderCMUStruct * encoder){
	printf( " Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d \n\r$", encoder->MechPosition, encoder->ElecPosition, encoder->raw);

//	printf("Raw: %d", encoder->raw);
//	printf("   Linearized Count: %d", encoder->count);
//	printf("   Single Turn: %f", encoder->angle_singleturn);
//	printf("   Multiturn: %f", encoder->angle_multiturn[0]);
//	printf("   Electrical: %f", encoder->elec_angle);
//	printf("   Turns:  %d\r\n", encoder->turns);
//	HAL_Delay(dt_ms);
}
