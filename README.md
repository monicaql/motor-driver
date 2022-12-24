# motor-driver

Changes so far:
- in main.c:
	- updated the flash variables to match the new motor driver repo
  
- encoder:
	- created position_sensor_icmu.h and .c for the new sensor; 
	- new struct name is EncoderCMUStruct
	- converted mbed functions for encoder into hal, sending A6 00 00 00 to encoder
	- datasheet for new position sensor: https://www.ichaus.de/upload/pdf/MU_datasheet_F2en.pdf 
	- position_sensor.h and .c are for the old sensor; old struct name was EncoderStruct
	
	- spi3 is for the encoder
	- in spi.c, edit MX_SPI3_Init function to adjust baud rate for encoder
	- currently it seems only 256 prescaler works
	
 	- started working on moving functions in calibrate from the mbed repo into this one
  	- didn't get the chance to test these functions, only compiled and built them

- other things:
	- mbed used wait_us, but hal has no built in function for microsecond delays, just wrote Hal_Delay(1) 
