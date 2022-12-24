/*
 * calibration.c
 *
 *  Created on: Aug 11, 2020
 *      Author: ben
 */


#include "calibration.h"
#include "hw_config.h"
#include "user_config.h"
#include <stdio.h>
#include <stdlib.h>
#include "usart.h"
#include "math_ops.h"
#include "structs.h"


// order phases has code ported over from newer motor control repo (not tested)
void order_phases(EncoderCMUStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
	///Checks phase order, to ensure that positive Q current produces
	///torque in the positive direction wrt the position sensor.
	printf("\n\r Checking phase ordering\n\r");
	float theta_ref = 0;
	float theta_actual = 0;
	float v_d = V_CAL;                                                             //Put all volts on the D-Axis
	float v_q = 0.0f;
	float v_u, v_v, v_w = 0;
	float dtc_u, dtc_v, dtc_w = .5f;
	int sample_counter = 0;

	///Set voltage angle to zero, wait for rotor position to settle
	abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 //inverse dq0 transform on voltages
	//printf("\n\rorder phases abc %f   %f   %f   %f   %f   %f   \n\r",  theta_ref, v_d, v_q, &v_u, &v_v, &v_w); //TEST//
	svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            //space vector modulation
	//printf("\n\rorder phases svm %f   %f   %f   %f   %f   %f   \n\r",  1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w); //TEST//
	for(int i = 0; i<20000; i++){
		TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
		TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
		TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
//		wait_us(100);
		HAL_Delay(1); // might have to change to 100 us later
		}
	//ps->ZeroPosition();
	Sample(encoder, DT);
//	wait_us(1000);
	HAL_Delay(1);
	//float theta_start = ps->GetMechPositionFixed();                                  //get initial rotor position
	float theta_start;
	controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
	controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
	controller->i_a = -controller->i_b - controller->i_c;
	dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
	float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
	printf("\n\rCurrent\n\r");
	printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
	/// Rotate voltage angle
	while(theta_ref < 4*PI_F){                                                    //rotate for 2 electrical cycles
		abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //inverse dq0 transform on voltages
		// printf("\n\rorder phases abc %f   %f   %f   %f   %f   %f   \n\r",  theta_ref, v_d, v_q, &v_u, &v_v, &v_w); //TEST//
		// wait_us(10000);
		svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                        //space vector modulation
		// printf("\n\rorder phases svm %f   %f   %f   %f   %f   %f   \n\r",  1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w); //TEST//
//		wait_us(100);
		HAL_Delay(1); // might have to change to 100 us later
		TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        //Set duty cycles
		TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
		TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
	   Sample(encoder, DT);                                                            //sample position sensor
	   theta_actual = encoder->MechPosition + encoder->MechOffset;
	   if(theta_ref==0){theta_start = theta_actual;}
	   if(sample_counter > 200){
		   sample_counter = 0 ;
		printf("%.4f   %.4f\n\r", theta_ref/(NPP), theta_actual);
		}
		sample_counter++;
	   theta_ref += 0.001f;
		}
	float theta_end = encoder->MechPosition + encoder->MechOffset;
	int direction = (theta_end - theta_start)>0;
	printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
	printf("Direction:  %d\n\r", direction);
	if(direction){printf("Phasing correct\n\r");}
	else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
	PHASE_ORDER = direction;
}


//this function is also ported over from new repo (not tested)
float check_encoder_init(EncoderCMUStruct *encoder, ControllerStruct *controller){

    printf("\n\r Checking encoder initialization\n\r");

    float theta_ref = 0.0f;
    float theta_elec_read = 0.0f;
    float theta_elec_calc = 0.0f;
    float theta_elec_err = 0.0f;
    float v_d = V_CAL;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0.0f;
    float dtc_u, dtc_v, dtc_w = .5f;
    int encoder_status = 0;

    // Get some data about initial encoder state

    ///Set voltage angle to zero, wait for rotor position to settle
    theta_ref = PI_F/2.0f; // set arbitrary angle for rotor to settle to
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation

//    int statusA = ps->ReadRegister(0x76); // read status0 register
//    int statusB = ps->ReadRegister(0x77); // read status1 register
//    int statusAB = (statusB<<8)|(statusA);
//    printf(" Initial Encoder State: %.4f,   %.4f,   %d,   %d\n\r", ps->GetMechPosition(), ps->GetElecPosition(), ps->GetRawPosition(), statusAB);


    for(int i = 0; i<10000; i++){
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
        if(PHASE_ORDER){
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
//        wait_us(100);
        HAL_Delay(1);
        }

    theta_ref = 0.0f; // set arbitrary angle for rotor to settle to
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    for(int j = 0; j<20000; j++){
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
        if(PHASE_ORDER){
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        wait_us(100);
        if (j==15000) {
            //ps->Sample(DT);
            theta_elec_read = encoder->ElecPosition;
            //printf("Theta_elec = %.2f\n\r", theta_elec_read);
        }
        }
    // Read encoder
//    ps->Sample(DT);

    // Calculate electrical angle...should be at 0.0
//    theta_elec_calc = ps->GetElecPosition();

    // Print difference and status of initialization
    if (theta_elec_read > PI_F) { theta_elec_calc = theta_elec_read - 2.0f*PI_F; } // wrap from -PI to PI instead of 0 to 2*PI
    else { theta_elec_calc = theta_elec_read; };
    if (theta_elec_calc >= 0.0f) { theta_elec_err = theta_elec_calc; }
    else { theta_elec_err = -theta_elec_calc; }
//    theta_elec_err = abs(theta_elec_calc);

    if (theta_elec_err < (PI_F/2.0f)) { // initialization is good
        printf(" Good initialization! theta_elec = %.2f, theta_elec_calc = %.2f\r\n", theta_elec_read, theta_elec_calc);
        encoder_status = 1;
    } else { // electrical angle error is larger than 90deg
        printf(" BAD initialization, theta_elec = %.2f, theta_elec_calc = %.2f\r\n", theta_elec_read, theta_elec_calc);
        encoder_status = 0;
    }

//    // Print final encoder status
//    statusA = ps->ReadRegister(0x76); // read status0 register
//    statusB = ps->ReadRegister(0x77); // read status1 register
//    statusAB = (statusB<<8)|(statusA);
//    printf(" Final Encoder State: %.4f,   %.4f,   %d,   %d\n\r", ps->GetMechPosition(), ps->GetElecPosition(), ps->GetRawPosition(), statusAB);


    return theta_elec_read;

} // end check_encoder_init function


// ported over from new repo, but there are issues with the vectors
void calibrate(EncoderCMUStruct *encoder, ControllerStruct *controller){
//    /// Measures the electrical angle offset of the position sensor
//    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
//    printf("Starting calibration procedure\n\r");
//    float * error_f;
//    float * error_b;
//    int * lut;
//    int * raw_f;
//    int * raw_b;
//    float * error;
//    float * error_filt;
//
//    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
//    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
//    float delta = 2*PI_F*NPP/(n*n2);                                              // change in angle between samples
//    error_f = new float[n]();                                                     // error vector rotating forwards
//    error_b = new float[n]();                                                     // error vector rotating backwards
//    const int  n_lut = 128;
//    lut = new int[n_lut]();                                                        // clear any old lookup table before starting.
//
//    error = new float[n]();
//    const int window = 128;
//    error_filt = new float[n]();
//    float cogging_current[window] = {0};
//
//    ps->WriteLUT(lut);
////    memcpy(offset_lut, new_lut, sizeof(offset_lut));
//    raw_f = new int[n]();
//    raw_b = new int[n]();
//    float theta_ref = 0;
//    float theta_actual = 0;
//    float v_d = V_CAL;                                                             // Put volts on the D-Axis
//    float v_q = 0.0f;
//    float v_u, v_v, v_w = 0;
//    float dtc_u, dtc_v, dtc_w = .5f;
//
//
//    ///Set voltage angle to zero, wait for rotor position to settle
//    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
//    svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
//    for(int i = 0; i<40000; i++){
//        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
//        if(PHASE_ORDER){
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//        else{
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//        wait_us(100);
//        }
//    ps->Sample(DT);
//    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
//    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
//    controller->i_a = -controller->i_b - controller->i_c;
//    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
//    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
//    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
//    for(int i = 0; i<n; i++){                                                   // rotate forwards
//       for(int j = 0; j<n2; j++){
//        theta_ref += delta;
//       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
//       svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
//        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
//        if(PHASE_ORDER){                                                        // Check phase ordering
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);                                    // Set duty cycles
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//        else{
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//            wait_us(100);
//            encoder->Sample(DT);
//        }
//       encoder->Sample(DT);
//       theta_actual = ps->GetMechPositionFixed();
//       error_f[i] = theta_ref/NPP - theta_actual;
//       raw_f[i] = ps->GetRawPosition();
//        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);
//       //theta_ref += delta;
//        }
//
//    for(int i = 0; i<n; i++){                                                   // rotate backwards
//       for(int j = 0; j<n2; j++){
//       theta_ref -= delta;
//       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
//       svm(1.0, v_u, v_v, v_w, 0, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
//        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
//        if(PHASE_ORDER){
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//        else{
//            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
//            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
//            }
//            wait_us(100);
//            ps->Sample(DT);
//        }
//       ps->Sample(DT);                                                            // sample position sensor
//       theta_actual = ps->GetMechPositionFixed();                                    // get mechanical position
//       error_b[i] = theta_ref/NPP - theta_actual;
//       raw_b[i] = ps->GetRawPosition();
//       printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
//       //theta_ref -= delta;
//        }
//
//        float offset = 0;
//        for(int i = 0; i<n; i++){
//            offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
//            }
//        offset = fmod(offset*NPP, 2*PI_F);                                        // convert mechanical angle to electrical angle
//
//
//        ps->SetElecOffset(offset);                                              // Set position sensor offset
//        __float_reg[0] = offset;
//        E_OFFSET = offset;
//
//        /// Perform filtering to linearize position sensor eccentricity
//        /// FIR n-sample average, where n = number of samples in one electrical cycle
//        /// This filter has zero gain at electrical frequency and all integer multiples
//        /// So cogging effects should be completely filtered out.
//
//
//        float mean = 0;
//        for (int i = 0; i<n; i++){                                              //Average the forward and back directions
//            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
//            }
//        for (int i = 0; i<n; i++){
//            for(int j = 0; j<window; j++){
//                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
//                if(ind<0){
//                    ind += n;}                                                  // Moving average wraps around
//                else if(ind > n-1) {
//                    ind -= n;}
//                error_filt[i] += error[ind]/(float)window;
//                }
//            if(i<window){
//                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
//                }
//            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
//            mean += error_filt[i]/n;
//            }
//        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty
//
//        printf("Raw offset: %d \n\r", raw_offset);
//
//        printf("\n\r Encoder non-linearity compensation table\n\r");
//        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
//        for (int i = 0; i<n_lut; i++){
//                                             // build lookup table
//            int ind = (raw_offset>>12) + i;  // shifted by lut_shift to ensure index is from 0 to 128
//
//            if(ind > (n_lut-1)){
//                ind -= n_lut;
//                }
//            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
//            //int temp = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
//            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
//            // wait(.001); // waits .001 ms
//            // .001 ms = 1 us
//            wait_us(1);
//
//            }
//
//        ps->WriteLUT(lut);                                                      // write lookup table to position sensor object
//
//        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
//
//        memcpy(&ENCODER_LUT, lut, 128*4);                                 // copy the lookup table to the flash array
//        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
//        //for(int i = 0; i<128; i++){printf("%d\n\r", __int_reg[i]);}
//        //printf("\n\r %d \n\r", sizeof(lut));
//
//
////        if (!prefs->ready()) prefs->open();
////        prefs->flush();         //TEST                                                // write offset and lookup table to flash
////        prefs->close();
////
//        if (!preference_writer_ready(prefs)){ preference_writer_open(&prefs);}
//		 preference_writer_flush(&prefs);
//		 preference_writer_close(&prefs);
//
//        delete[] error_f;       //gotta free up that ram
//        delete[] error_b;
//        delete[] lut;
//        delete[] raw_f;
//        delete[] raw_b;

    }


void calibrate_encoder(EncoderCMUStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){
//	/* Calibrates e-zero and encoder nonliearity */
//
//	if(!cal->started){
//			printf("Starting offset cal and linearization\r\n");
//			cal->started = 1;
//			cal->start_count = loop_count;
//			cal->next_sample_time = T1;
//			cal->sample_count = 0;
//		}
//
//	cal->time = (float)(loop_count - cal->start_count)*DT;
//
//    if(cal->time < T1){
//        // Set voltage angle to zero, wait for rotor position to settle
//        cal->theta_ref = 0;//W_CAL*cal->time;
//        cal->cal_position.elec_angle = cal->theta_ref;
//        controller->i_d_des = I_CAL;
//        controller->i_q_des = 0.0f;
//        commutate(controller, &cal->cal_position);
//
//    	cal->theta_start = encoder->angle_multiturn[0];
//    	cal->next_sample_time = cal->time;
//    	return;
//    }
//    else if (cal->time < T1+2.0f*PI_F*PPAIRS/W_CAL){
//    	// rotate voltage vector through one mechanical rotation in the positive direction
//		cal->theta_ref += W_CAL*DT;//(cal->time-T1);
//		cal->cal_position.elec_angle = cal->theta_ref;
//		commutate(controller, &cal->cal_position);
//
//		// sample SAMPLES_PER_PPAIR times per pole-pair
//		if(cal->time > cal->next_sample_time){
//			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
//			int error = encoder->raw - count_ref;//- encoder->raw;
//			cal->error_arr[cal->sample_count] = error + ENC_CPR*(error<0);
//			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
//			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
//			if(cal->sample_count == PPAIRS*SAMPLES_PER_PPAIR-1){
//				return;
//			}
//			cal->sample_count++;
//
//		}
//		return;
//    }
//	else if (cal->time < T1+4.0f*PI_F*PPAIRS/W_CAL){
//		// rotate voltage vector through one mechanical rotation in the negative direction
//		cal->theta_ref -= W_CAL*DT;//(cal->time-T1);
//		controller->i_d_des = I_CAL;
//		controller->i_q_des = 0.0f;
//		cal->cal_position.elec_angle = cal->theta_ref;
//		commutate(controller, &cal->cal_position);
//
//		// sample SAMPLES_PER_PPAIR times per pole-pair
//		if((cal->time > cal->next_sample_time)&&(cal->sample_count>0)){
//			int count_ref = cal->theta_ref * (float)ENC_CPR/(2.0f*PI_F*PPAIRS);
//			int error = encoder->raw - count_ref;// - encoder->raw;
//			error = error + ENC_CPR*(error<0);
//
//			cal->error_arr[cal->sample_count] = (cal->error_arr[cal->sample_count] + error)/2;
//			printf("%d %d %d %.3f\r\n", cal->sample_count, count_ref, cal->error_arr[cal->sample_count], cal->theta_ref);
//			cal->sample_count--;
//			cal->next_sample_time += 2.0f*PI_F/(W_CAL*SAMPLES_PER_PPAIR);
//		}
//		return;
//    }
//
//    reset_foc(controller);
//
//    // Calculate average offset
//    int ezero_mean = 0;
//	for(int i = 0; i<((int)PPAIRS*SAMPLES_PER_PPAIR); i++){
//		ezero_mean += cal->error_arr[i];
//	}
//	cal->ezero = ezero_mean/(SAMPLES_PER_PPAIR*PPAIRS);
//
//	// Moving average to filter out cogging ripple
//
//	int window = SAMPLES_PER_PPAIR;
//	int lut_offset = (ENC_CPR-cal->error_arr[0])*N_LUT/ENC_CPR;
//	for(int i = 0; i<N_LUT; i++){
//			int moving_avg = 0;
//			for(int j = (-window)/2; j<(window)/2; j++){
//				int index = i*PPAIRS*SAMPLES_PER_PPAIR/N_LUT + j;
//				if(index<0){index += (SAMPLES_PER_PPAIR*PPAIRS);}
//				else if(index>(SAMPLES_PER_PPAIR*PPAIRS-1)){index -= (SAMPLES_PER_PPAIR*PPAIRS);}
//				moving_avg += cal->error_arr[index];
//			}
//			moving_avg = moving_avg/window;
//			int lut_index = lut_offset + i;
//			if(lut_index>(N_LUT-1)){lut_index -= N_LUT;}
//			cal->lut_arr[lut_index] = moving_avg - cal->ezero;
//			printf("%d  %d\r\n", lut_index, moving_avg - cal->ezero);
//
//		}
//
//	cal->started = 0;
//	cal->done_cal = 1;
}

void measure_lr(EncoderStruct *encoder, ControllerStruct *controller, CalStruct * cal, int loop_count){

}
