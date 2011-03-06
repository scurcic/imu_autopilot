/*=====================================================================

PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

(c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

This file is part of the PIXHAWK project

    PIXHAWK is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    PIXHAWK is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief adns9500 laser mouse sensor device driver
 *
 *   This file contains the device driver for the Avago ADNS9500 laser mouse sensor.
 *   @author Srdjan Curcic <scurcic@ee.ethz.ch>
 *
 */

#include "adns9500.h"
#include "spi.h"
#include "armVIC.h"
#include "conf.h"
#include "sys_time.h"
#include "comm.h"
#include "led.h"
#include "inttypes.h"
#include <stdio.h>
#include "..\mavlink\include\pixhawk\mavlink.h"
#include "adns9500_srom.h"

//#if(FEATURE_ACC==FEATURE_ACC_3100)

int adns9500_values[5];						// temporary storage for measured x and y values

void adns9500_select(void){				// pull ADNS9500 chip select low
	ADNS9500_SS_IOCLR|=1<<ADNS9500_SS_PIN;
}
void adns9500_unselect(void){			// pull ADNS9500 chip select high
	ADNS9500_SS_IOSET|=1<<ADNS9500_SS_PIN;
}

void adns9500_init(void) {

	/* configure SS pin */
	ADNS9500_SS_IODIR|=1<<ADNS9500_SS_PIN; 		/* pin is output  */
	adns9500_unselect(); 						/* pin idles high */

	for(int i=0; i<2; i++){
		adns9500_values[i]=-10;
	}

	adns9500_power_up_sequence();

}

void adns9500_power_up_sequence(void)
{
	adns9500_write_reg(POWER_UP_RESET,0x5A);

	us_delay(50000);



	adns9500_read_reg(MOTION);
	us_delay(100);
	adns9500_read_reg(DELTA_X_L);
	us_delay(100);
	adns9500_read_reg(DELTA_X_H);
	us_delay(100);
	adns9500_read_reg(DELTA_Y_L);
	us_delay(100);
	adns9500_read_reg(DELTA_Y_H);
	us_delay(100);

	adns9500_srom_download();

	adns9500_write_reg(LASER_CTRL0,0x80);
	us_delay(500);

}

void check_connection(void)
{
adns9500_read_reg(PRODUCT_ID);
unsigned char prod_id=adns9500_get_value(1);

adns9500_read_reg(REVISION_ID);
unsigned char rev_id=adns9500_get_value(1);

adns9500_read_reg(SROM_ID);
unsigned char rom_id=adns9500_get_value(1);


if ((prod_id==51) & (rev_id==3) & (rom_id==145))
	led_on(LED_GREEN);
else
	led_off(LED_GREEN);
}

void adns9500_check_motion(void)
{
	adns9500_read_reg(MOTION);
	unsigned char val=adns9500_get_value(1);
	if (val==160)
	{
		led_on(LED_RED);
	}
	else
	{
		led_off(LED_RED);
	}
}
//
//adns9500_get_values(void)
//{
//	adns9500_read_reg(DELTA_X_L);
//	x_l=adns9500_get_value(1);
//	adns9500_read_reg(DELTA_X_H);
//	x_h=adns9500_get_value(1);
//	led_on(LED_GREEN);
//	adns9500_read_reg(DELTA_Y_L);
//	y_l=adns9500_get_value(1);
//	adns9500_read_reg(DELTA_Y_H);
//	y_h=adns9500_get_value(1);
//	led_on(LED_GREEN);
//}

void adns9500_srom_download(void)
{
	adns9500_write_reg(CONFIGURATION_IV,0x02);
	us_delay(150);
	adns9500_write_reg(SROM_ENABLE,0x1D);
	us_delay(800);
	adns9500_write_reg(SROM_ENABLE,0x18);
	us_delay(150);

	adns9500_write_srom();

}

void adns9500_write_reg(unsigned char reg,unsigned char data)
{
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char addr;
	//unsigned char data;
	addr = set_bits(reg,MASK);
	package.data[0] = addr;
	package.data[1] = data;
	package.length = 2;
	package.slave_select = &adns9500_select;
	package.slave_unselect = &adns9500_unselect;
	package.spi_interrupt_handler = &adns9500_on_spi_write_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
	spi_transmit(&package);
}

void adns9500_write_srom(void)
{
	unsigned char addr;
	addr = set_bits(SROM_LOAD_BURST,MASK);
	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;

	adns9500_select();
	SSPDR=addr;
	SpiEnable();

	while(SSPSR&(1<<BSY));
	/*disable spi*/
	SpiDisable();

	us_delay(20);

	for (int i=0;i<3070;i++){
		SSPDR=srom_file[i];
		SpiEnable();
		while(SSPSR&(1<<BSY))
		{}
		SpiDisable();
		us_delay(15);
	}
	us_delay(100);

	adns9500_unselect();
}

void adns9500_read_motion_burst(void)
{
	int byte1[14];
	int byte2[15];
	for (int j=0;j<14;j++)
		{
		byte2[j]=-1;
		}

	unsigned char addr;
	addr = set_bits(MOTION_BURST,MASK);
	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;

	adns9500_select();
	SSPDR=addr;
	SSPDR=0x50;
	SpiEnable();

	while(SSPSR&(1<<BSY));
	/*disable spi*/
	SpiDisable();

	us_delay(120);

	SSPDR=MOTION_BURST;

	SpiEnable();
	while(SSPSR&(1<<BSY));
	SpiDisable();

	us_delay(120);

	SpiEnable();

	while(SSPSR&(0<<RNE));

	byte2[0]=SSPSR;
	byte1[0]=SSPDR;
	byte2[1]=SSPSR;
	byte1[1]=SSPDR;
	byte2[2]=SSPSR;
	byte1[2]=SSPDR;
	byte2[3]=SSPSR;
	byte1[3]=SSPDR;
	byte2[4]=SSPSR;
	byte1[4]=SSPDR;
	byte2[5]=SSPSR;
	byte1[5]=SSPDR;
	byte2[6]=SSPSR;
	byte1[6]=SSPDR;
	byte2[7]=SSPSR;
	byte1[7]=SSPDR;
	byte2[8]=SSPSR;
	byte1[8]=SSPDR;
	byte2[9]=SSPSR;
	byte1[9]=SSPDR;
	byte2[10]=SSPSR;
	byte1[10]=SSPDR;
	byte2[11]=SSPSR;
	byte1[11]=SSPDR;
	byte2[12]=SSPSR;
	byte1[12]=SSPDR;
	byte2[13]=SSPSR;
	byte1[13]=SSPDR;
	byte2[14]=SSPSR;
	byte1[14]=SSPDR;
	SpiDisable();

	us_delay(100);

	addr = set_bits(MOTION,MASK);

	SSPDR=addr;
	SSPDR=0x00;
//	SpiDisable();
	SpiEnable();

	while(SSPSR&(1<<BSY));
		/*disable spi*/
	SpiDisable();
	us_delay(20);
	adns9500_unselect();

	adns9500_values[0]=byte2[1];//package.data[1];
	adns9500_values[1]=byte1[1];
}

void adns9500_read_reg(unsigned char reg) {
		spi_package package;
		package.bit_mode = SPI_8_BIT_MODE;
		//unsigned char cmd;
		//cmd = reg;					// 0x5800 is the command to read the status register
		package.data[0] = reg;
		package.data[1] = 0;
		package.length = 2;
		package.slave_select = &adns9500_select;
		package.slave_unselect = &adns9500_unselect;
		package.spi_interrupt_handler = &adns9500_on_spi_read_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
		spi_adns9500_read_reg_transmit(&package);
}


//void adns9500_read_motion_burst(void) {
//unsigned char addr;
//addr = set_bits(MOTION_BURST,MASK);
//
//SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
//
//adns9500_select();
//SSPDR=addr;
//SpiEnable();
//
//while(SSPSR&(1<<BSY));
///*disable spi*/
//SpiDisable();
//
//us_delay(200);
//
//for (int i=0;i<3070;i++){
//		SSPDR=0;
//		SpiEnable();
//		while(SSPSR&(1<<BSY))
//		{}
//		SpiDisable();
//		us_delay(15);
//	}
//
//
//
//	us_delay(100);
//	SpiDisable();

//	adns9500_unselect();

//	package.data[0] = addr;
//
//	package.length =3071;
//	package.slave_select = &adns9500_select;
//	package.slave_unselect = &adns9500_unselect;
//	package.spi_interrupt_handler = &adns9500_on_spi_write_srom;//srom;	// sca3100_on_spi_read_reg() is invoked at SPI completion
//	spi_transmit_srom(&package);
//}

//{
//	unsigned char addr;
//	addr = set_bits(SROM_LOAD_BURST,MASK);
////	unsigned short length_srom_file=sizeof(srom_file)/sizeof(unsigned char);
////	addr = SROM_LOAD_BURST;
//	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL | SSP_CPHA | SSP_SCR;
//
//
//	adns9500_select();
//	SSPDR=addr;
//	SpiEnable();
//
//	while(SSPSR&(1<<BSY));
//	/*disable spi*/
//	SpiDisable();
//
//	us_delay(20);
//
//	for (int i=0;i<3070;i++){
//		SSPDR=srom_file[i];
//		SpiEnable();
//		while(SSPSR&(1<<BSY))
//		{}
//		SpiDisable();
//		us_delay(15);
//	}
//
//	us_delay(100);
//
//	adns9500_unselect();
//}

//void adns9500_read_motion_burst(void) {
//		adns9500_write_reg(MOTION_BURST,0x50);
//		us_delay(200);
//
//		srom_spi_package package;
//		package.bit_mode = SPI_8_BIT_MODE;
////		unsigned char addr;
//		//addr = set_bits(MOTION_BURST,MASK);
//		package.data[0] = MOTION_BURST;
//		package.data[1] = 0;
//		package.data[2] = 0;
//		package.data[3] = 0;
//		package.data[4] = 0;
//		package.data[5] = 0;
//		package.data[6] = 0;
//		package.data[7] = 0;
//		package.data[8] = 0;
//		package.data[9] = 0xff;
//		package.data[10] = 0;
//		package.data[11] = 0;
//		package.data[12] = 0;
//		package.data[13] = 0;
//		package.data[14] = 0;
//		package.length = 15;
//		package.slave_select = &adns9500_select;
//		package.slave_unselect = &adns9500_unselect;
//		package.spi_interrupt_handler = &adns9500_on_spi_motion_burst;	// sca3100_on_spi_read_reg() is invoked at SPI completion
//		spi_motion_burst(&package);
//}

void delay_us(unsigned long us) {
    unsigned long ticks=us*6; //For the STM32 running at 72MHz
    my_delay(ticks);
}


void adns9500_on_spi_write_reg(void) {
//			unsigned short foo __attribute__ ((unused)) = SSPDR;	// this was a write operation -> discard any received data (2 bytes)
//			foo = SSPDR;
			//led_on(LED_GREEN);

			unsigned char foo __attribute__ ((unused)) = SSPDR;	// discard first byte
			foo = SSPDR;
//			unsigned char data=SSPDR;
//			unsigned char status = SSPSR;
//
//			adns9500_values[0]=status;//package.data[1];
//			adns9500_values[1]=data;

//			adns9500_values[0]=int_status;//package.data[1];
//			return;
}

void adns9500_on_spi_write_srom(void) {
//			unsigned short foo __attribute__ ((unused)) = SSPDR;	// this was a write operation -> discard any received data (2 bytes)
//			foo = SSPDR;
			//led_on(LED_GREEN);

			unsigned char foo __attribute__ ((unused)) = SSPDR;	// discard first byte
			unsigned char check = SSPSR;
			unsigned char tr=1;

			while (tr==1)
			{
				foo = SSPDR;
				check=SSPSR;
				if (check==3)
				{
					tr=0;
				}
				else
					{
					tr=1;
					}
				}
//			unsigned char data=package.data[0];
//			data=SSPDR;
//			data=SSPDR;
//			data=SSPDR;


//			unsigned char status = SSPSR;

//			adns9500_values[0]=status;//package.data[1];
//			adns9500_values[1]=data;

//			adns9500_values[0]=int_status;//package.data[1];
//			return;
}
unsigned char clear_bits(unsigned char data,unsigned char mask){
    return (data & ~(mask));
}

unsigned char set_bits(unsigned char data,unsigned char mask){
    return (data | mask);
}

void my_delay(unsigned long delay ){
    volatile unsigned long tmp=delay;
    while(tmp){
        tmp--;
    }
}

//void delay_ms(unsigned long ms) {
//    unsigned long ticks=ms*6000; //For the STM32 running at 72MHz
//    my_delay(ticks);
//   //Delay(ms*8);	// use systick	-> tick is 1ms at 9MHZ -> 1/8ms at 72MHz
//}

void adns9500_on_spi_read_reg(void) {		// INT_STATUS register has been read

	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;

	unsigned char status = SSPSR;
	unsigned char data=0;

	while (status!=3)
	{
		data=SSPDR;
		status=SSPSR;
	}

	status = SSPSR;

	adns9500_values[0]=status;
	adns9500_values[1]=data;
	return;
}

void adns9500_read_sensor(void){

	unsigned char msb;
	unsigned char lsb;

	uint64_t time_read=sys_time_clock_get_time_usec();
	adns9500_read_reg(MOTION);

	adns9500_read_reg(DELTA_X_L);
	lsb=adns9500_get_value(1);

	adns9500_read_reg(DELTA_X_H);
	msb=adns9500_get_value(1);

	adns9500_read_reg(DELTA_Y_L);
	lsb=adns9500_get_value(1);

	adns9500_read_reg(DELTA_Y_H);
	msb=adns9500_get_value(1);


//	int16_t delta_x = ((uint16_t)msb<<8) | lsb;
	signed short delta_x=msb<<8;
//	int delta_x2=msb;
//	int delta_x3=lsb;
	short val = (short)delta_x + lsb;

//	adns9500_values[ADNS9500_DX] = msb;
//	adns9500_values[ADNS9500_DY] = lsb; 	// store measurement for X axis
//	adns9500_values[2] = delta_x; 	// store measurement for X axis
	adns9500_values[ADNS9500_DX] = val; 	// store measurement for X axis
	adns9500_values[ADNS9500_DY] = time_read;
//	adns9500_values[3] = val2; 	// store measurement for X axis

	//	int a3;
//	int a4;
//	adns9500_read_reg(DELTA_Y_L);
//	a3=adns9500_get_value(1);
//	adns9500_read_reg(DELTA_Y_H);
//	a4=adns9500_get_value(1);
}

//int adns9500_median(int values){
//	return int sorted_values=sort(values);
//}

//void adns9500_on_spi_motion_burst(void) {		// INT_STATUS register has been read
//
//	srom_spi_package package;
//	package.bit_mode = SPI_8_BIT_MODE;
//	int byte1[14];
//
////	for (int iw=1;iw<14;iw++)
////	{
////		byte1[iw]=-1;
////	}
//
//	unsigned char foo __attribute__ ((unused)) = SSPDR;	// discard first byte
//
//
//	byte1[0] = SSPDR;					// MOTION register
////	unsigned char status1 = SSPSR;
//	byte1[1] = SSPDR;					// OBSERVATION register
////	unsigned char status2 = SSPSR;
//	byte1[2] = SSPDR;					// DELTA_X_L register
////	unsigned char status3 = SSPSR;
//	byte1[3] = SSPDR;					// DELTA_X_H register
////	unsigned char status4 = SSPSR;
//	byte1[4] = SSPDR;					// DELTA_Y_L register
////	unsigned char status5 = SSPSR;
//	byte1[5] = SSPDR;					// DELTA_Y_H register
//	unsigned char status6 = SSPSR;
//	byte1[6] = SSPDR;					// SQUAL register
//	unsigned char status7 = SSPSR;
//	byte1[7] = SSPDR;					// PIXEL_SUM register
//	unsigned char status8 = SSPSR;
//	byte1[8] = SSPDR;					// MAXIMUM_PIXEL
//	unsigned char status9 = SSPSR;
//	byte1[9] = SSPDR;					// MINIMUM_PIXEL
//	unsigned char status10 = SSPSR;
//	byte1[10] = SSPDR;					// SHUTTER_UPPER
//	unsigned char status11 = SSPSR;
//	byte1[11] = SSPDR;					// SHUTTER_LOWER
//	unsigned char status12 = SSPSR;
//	byte1[12] = SSPDR;					// FRAME_PERIOD_UPPER
//	unsigned char status13 = SSPSR;
//	byte1[13] = SSPDR;					// FRAME_PERIOD_LOWER
//	unsigned char status14 = SSPSR;
//
//
//	adns9500_values[0]=status7;//package.data[1];
//	adns9500_values[1]=byte1[2];
//
////	for (int iq=0;iq<14;iq++)
////	{
////		adns9500_values(iq)=byte1[iq];
////	}
//
//
////		if(int_status & (1<<5)) {
////		//sprintf((char *)buffer, "STS: Start-up self-test failed\n");
////		//message_send_debug(COMM_1, buffer);
////	}
////	// if STC has failed
////	if(int_status & (1<<4)) {
////		//sprintf((char *)buffer, "STC: Continuous self-test failed\n");
////		//message_send_debug(COMM_1, buffer);
////	}
////	// if not SAT & not STS & not STC, then memory self test has failed -> do another memory self-test
////	if(!((int_status & (1<<6)) || (int_status & (1<<4)) || (int_status & (1<<5)))) {
////		//sprintf((char *)buffer, "ST: Memory self test-failed\n");
////		//message_send_debug(COMM_1, buffer);
////		cmd = 0x00;				// 0x0704 is the command to start a new memory self test
////		package.data[0] = cmd;
////		package.length = 1;
////		package.slave_select = &adns9500_select;
////		package.slave_unselect = &adns9500_unselect;
////		package.spi_interrupt_handler = &adns9500_on_spi_write;	// sca3100_on_spi_write() is invoked at SPI completion
////		spi_transmit(&package);
////	}
////	led_off(LED_GREEN);
//	return;
//}

int adns9500_get_value(int axis){
	return adns9500_values[axis];	// return value of specified axis
}

void us_delay(unsigned long us)
{
	unsigned long initial_time = sys_time_clock_get_time_usec();
	unsigned long current_time = initial_time;
	while (current_time < initial_time + us)
	{
		current_time = sys_time_clock_get_time_usec();
	}
}

//#endif

