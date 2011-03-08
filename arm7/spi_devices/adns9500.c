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

int adns9500_1_values[2];						// temporary storage for measured x and y values
int adns9500_2_values[2];						// temporary storage for measured x and y values

void adns9500_1_select(void){					// pull ADNS9500 chip select low
	ADNS9500_1_SS_IOCLR|=1<<ADNS9500_1_SS_PIN;
}

void adns9500_1_unselect(void){					// pull ADNS9500 chip select high
	ADNS9500_1_SS_IOSET|=1<<ADNS9500_1_SS_PIN;
}

void adns9500_2_select(void){					// pull ADNS9500 chip select low
	ADNS9500_2_SS_IOCLR|=1<<ADNS9500_2_SS_PIN;
}

void adns9500_2_unselect(void){					// pull ADNS9500 chip select high
	ADNS9500_2_SS_IOSET|=1<<ADNS9500_2_SS_PIN;
}

void adns9500_init(void){

	for(int i=0; i<2; i++){
		adns9500_1_values[i]=-1;
		adns9500_2_values[i]=-1;
	}

	/* configure SS pin */
	ADNS9500_1_SS_IODIR|=1<<ADNS9500_1_SS_PIN; 		/* pin is output  */
	adns9500_1_unselect(); 							/* pin idles high */

	ADNS9500_2_SS_IODIR|=1<<ADNS9500_2_SS_PIN; 		/* pin is output  */
	adns9500_2_unselect(); 							/* pin idles high */

	adns9500_1_power_up_sequence();
	adns9500_2_power_up_sequence();

	unsigned char addr;
	addr = set_bits(CONFIGURATION_I,MASK);
	adns9500_1_write_reg(addr,0x38);
	adns9500_2_write_reg(addr,0x38);

	check_connection();
}

void adns9500_1_power_up_sequence(void)
{
	adns9500_1_write_reg(POWER_UP_RESET,0x5A);

	us_delay(500);

	adns9500_1_read_reg(MOTION);
	us_delay(100);
	adns9500_1_read_reg(DELTA_X_L);
	us_delay(100);
	adns9500_1_read_reg(DELTA_X_H);
	us_delay(100);
	adns9500_1_read_reg(DELTA_Y_L);
	us_delay(100);
	adns9500_1_read_reg(DELTA_Y_H);
	us_delay(100);

	adns9500_1_srom_download();

	adns9500_1_write_reg(LASER_CTRL0,0x80);
	us_delay(500);
}

void adns9500_2_power_up_sequence(void)
{
	adns9500_2_write_reg(POWER_UP_RESET,0x5A);

	us_delay(500);

	adns9500_2_read_reg(MOTION);
	us_delay(100);
	adns9500_2_read_reg(DELTA_X_L);
	us_delay(100);
	adns9500_2_read_reg(DELTA_X_H);
	us_delay(100);
	adns9500_2_read_reg(DELTA_Y_L);
	us_delay(100);
	adns9500_2_read_reg(DELTA_Y_H);
	us_delay(100);

	adns9500_2_srom_download();

	adns9500_2_write_reg(LASER_CTRL0,0x80);
	us_delay(500);
}

void check_connection(void)
{
adns9500_1_read_reg(PRODUCT_ID);
unsigned char prod_id_s1=adns9500_1_get_value(1);

adns9500_1_read_reg(REVISION_ID);
unsigned char rev_id_s1=adns9500_1_get_value(1);

adns9500_1_read_reg(SROM_ID);
unsigned char rom_id_s1=adns9500_1_get_value(1);

adns9500_2_read_reg(PRODUCT_ID);
unsigned char prod_id_s2=adns9500_2_get_value(1);

adns9500_2_read_reg(REVISION_ID);
unsigned char rev_id_s2=adns9500_2_get_value(1);

adns9500_2_read_reg(SROM_ID);
unsigned char rom_id_s2=adns9500_2_get_value(1);

if ((prod_id_s1==51) & (prod_id_s2==51) & (rev_id_s1==3) & (rev_id_s2==3) & (rom_id_s1==145) & (rom_id_s2==145))
	led_on(LED_GREEN);
else
	led_off(LED_GREEN);
}

//void adns9500_1_check_motion(void)
//{
//	adns9500_1_read_reg(MOTION);
//	unsigned char val=adns9500_1_get_value(1);
//	if (val==160)
//	{
//		led_on(LED_RED);
//	}
//	else
//	{
//		led_off(LED_RED);
//	}
//}

//void adns9500_2_check_motion(void)
//{
//	adns9500_2_read_reg(MOTION);
//	unsigned char val=adns9500_2_get_value(1);
//	if (val==160)
//	{
//		led_on(LED_YELLOW);
//	}
//	else
//	{
//		led_off(LED_YELLOW);
//	}
//}

void adns9500_1_srom_download(void)
{
	adns9500_1_write_reg(CONFIGURATION_IV,0x02);
	us_delay(150);
	adns9500_1_write_reg(SROM_ENABLE,0x1D);
	us_delay(800);
	adns9500_1_write_reg(SROM_ENABLE,0x18);
	us_delay(150);

	adns9500_1_write_srom();
}

void adns9500_2_srom_download(void)
{
	adns9500_2_write_reg(CONFIGURATION_IV,0x02);
	us_delay(150);
	adns9500_2_write_reg(SROM_ENABLE,0x1D);
	us_delay(800);
	adns9500_2_write_reg(SROM_ENABLE,0x18);
	us_delay(150);

	adns9500_2_write_srom();
}

void adns9500_1_write_reg(unsigned char reg,unsigned char data)
{
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char addr;
	//unsigned char data;
	addr = set_bits(reg,MASK);
	package.data[0] = addr;
	package.data[1] = data;
	package.length = 2;
	package.slave_select = &adns9500_1_select;
	package.slave_unselect = &adns9500_1_unselect;
	package.spi_interrupt_handler = &adns9500_on_spi_write_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
	spi_transmit_adns(&package);
}

void adns9500_2_write_reg(unsigned char reg,unsigned char data)
{
	spi_package package;
	package.bit_mode = SPI_8_BIT_MODE;
	unsigned char addr;
	//unsigned char data;
	addr = set_bits(reg,MASK);
	package.data[0] = addr;
	package.data[1] = data;
	package.length = 2;
	package.slave_select = &adns9500_2_select;
	package.slave_unselect = &adns9500_2_unselect;
	package.spi_interrupt_handler = &adns9500_on_spi_write_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
	spi_transmit_adns(&package);
}

void adns9500_1_write_srom(void)
{
	unsigned char addr;
	addr = set_bits(SROM_LOAD_BURST,MASK);
	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL_ADNS | SSP_CPHA_ADNS | SSP_SCR;

	adns9500_1_select();
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

	adns9500_1_unselect();
}

void adns9500_2_write_srom(void)
{
	unsigned char addr;
	addr = set_bits(SROM_LOAD_BURST,MASK);
	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL_ADNS | SSP_CPHA_ADNS | SSP_SCR;

	adns9500_2_select();
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

	adns9500_2_unselect();
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
	SSPCR0 = SPI_8_BIT_MODE |  SSP_FRF | SSP_CPOL_ADNS | SSP_CPHA_ADNS | SSP_SCR;

	adns9500_1_select();
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
	adns9500_1_unselect();

	adns9500_1_values[0]=byte2[1];//package.data[1];
	adns9500_1_values[1]=byte1[1];
}

void adns9500_1_read_reg(unsigned char reg) {
		spi_package package;
		package.bit_mode = SPI_8_BIT_MODE;
		//unsigned char cmd;
		//cmd = reg;					// 0x5800 is the command to read the status register
		package.data[0] = reg;
		package.data[1] = 0;
		package.length = 2;
		package.slave_select = &adns9500_1_select;
		package.slave_unselect = &adns9500_1_unselect;
		package.spi_interrupt_handler = &adns9500_1_on_spi_read_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
		spi_adns9500_read_reg_transmit(&package);
}

void adns9500_2_read_reg(unsigned char reg) {
		spi_package package;
		package.bit_mode = SPI_8_BIT_MODE;
		//unsigned char cmd;
		//cmd = reg;					// 0x5800 is the command to read the status register
		package.data[0] = reg;
		package.data[1] = 0;
		package.length = 2;
		package.slave_select = &adns9500_2_select;
		package.slave_unselect = &adns9500_2_unselect;
		package.spi_interrupt_handler = &adns9500_2_on_spi_read_reg;	// sca3100_on_spi_read_reg() is invoked at SPI completion
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

void adns9500_1_on_spi_read_reg(void) {		// INT_STATUS register has been read

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

	adns9500_1_values[0]=status;
	adns9500_1_values[1]=data;
	return;
}

void adns9500_2_on_spi_read_reg(void) {		// INT_STATUS register has been read

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

	adns9500_2_values[0]=status;
	adns9500_2_values[1]=data;
	return;
}
void adns9500_1_read_sensor(void){

	unsigned char msb_x, msb_y;
	unsigned char lsb_x, lsb_y;

//	uint64_t time_read=sys_time_clock_get_time_usec();

	adns9500_1_read_reg(MOTION);
	unsigned char val=adns9500_1_get_value(1);
	if (val==160)
	{
		led_on(LED_YELLOW);
	}
	else
	{
		led_off(LED_YELLOW);
	}

	adns9500_1_read_reg(DELTA_X_L);
	lsb_x=adns9500_1_get_value(1);

	adns9500_1_read_reg(DELTA_X_H);
	msb_x=adns9500_1_get_value(1);

	adns9500_1_read_reg(DELTA_Y_L);
	lsb_y=adns9500_1_get_value(1);

	adns9500_1_read_reg(DELTA_Y_H);
	msb_y=adns9500_1_get_value(1);

	signed short delta_x=msb_x<<8;
	short dx=(short)delta_x + lsb_x;		// Measurement along x-axis of the sensor

	signed short delta_y=msb_y<<8;
	short dy=(short)delta_y + lsb_y;		// Measurement along y-axis of the sensor

	adns9500_1_values[ADNS9500_DX] = -dy;	// y-axis of the sensor = negative x-axis of the car
	adns9500_1_values[ADNS9500_DY] = dx;	// x-axis of the sensor = y-axis of the car
}

void adns9500_2_read_sensor(void){

	unsigned char msb_x, msb_y;
	unsigned char lsb_x, lsb_y;

//	uint64_t time_read=sys_time_clock_get_time_usec();

	adns9500_2_read_reg(MOTION);
	unsigned char val=adns9500_2_get_value(1);
	if (val==160)
	{
		led_on(LED_RED);
	}
	else
	{
		led_off(LED_RED);
	}

	adns9500_2_read_reg(DELTA_X_L);
	lsb_x=adns9500_2_get_value(1);

	adns9500_2_read_reg(DELTA_X_H);
	msb_x=adns9500_2_get_value(1);

	adns9500_2_read_reg(DELTA_Y_L);
	lsb_y=adns9500_2_get_value(1);

	adns9500_2_read_reg(DELTA_Y_H);
	msb_y=adns9500_2_get_value(1);

	signed short delta_x=msb_x<<8;
	short dx=(short)delta_x + lsb_x;		// Measurement along x-axis of the sensor

	signed short delta_y=msb_y<<8;
	short dy=(short)delta_y + lsb_y;		// Measurement along y-axis of the sensor

	adns9500_2_values[ADNS9500_DX] = -dy;	// y-axis of the sensor = negative x-axis of the car
	adns9500_2_values[ADNS9500_DY] = dx;	// x-axis of the sensor = y-axis of the car
}

int adns9500_1_get_value(int axis){
	return adns9500_1_values[axis];	// return value of specified axis
}

int adns9500_2_get_value(int axis){
	return adns9500_2_values[axis];	// return value of specified axis
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

