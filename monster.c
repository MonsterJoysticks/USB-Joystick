/* Name: monster.c
 * Project: Monster Joysticks USB / GPIO Joystick Adapter V1
 * Author: Monster Joysticks Ltd. <info@monsterjoysticks.com>
 * Copyright: (C) 2017 - 2019 Monster Joysticks Ltd. <info@monsterjoysticks.com>
 * License: GPLv2
 * Tabsize: 4
 * Comments: Based on Multiple NES/SNES to USB converter by Christian Starkjohann
 */
 #define F_CPU   12000000L

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <string.h>

#include "joystick.h"
#include "monster.h"

#define REPORT_SIZE		3
#define GAMEPAD_BYTES	2	/* 2 byte per snes controller * 4 controllers */

/*********** prototypes *************/
static void joystickInit(void);
static void joystickUpdate(void);
static char joystickChanged(unsigned char report_id);
static char joystickBuildReport(unsigned char *reportBuffer, char report_id);


// the most recent bytes we fetched from the controller
static unsigned char last_read_controller_bytes[GAMEPAD_BYTES];

// the most recently reported bytes
static unsigned char *last_reported_controller_bytes[GAMEPAD_BYTES];

static void joystickInit(void)
{
	unsigned char sreg;
	sreg = SREG;
	cli();

	DDRC &= ~0x00; // PC0->PC5= Pins1,2,3,4,6,7 = U,D,L,R,B1,B2 INPUT
	PORTC |= 0xFF; // Set to high

	// PD4-> PD7= 8,9,10,11 = B3,4,5,6 INPUT
	DDRD &= ~(0 << 4); 
	DDRD &= ~(0 << 5); 
	DDRD &= ~(0 << 6); 
	DDRD &= ~(0 << 7);
	// Set to High
	PORTD |= (1 << 4);
	PORTD |= (1 << 5);
	PORTD |= (1 << 6);
	PORTD |= (1 << 7);

	// PB0-> PB2= 12,13,14 = B7,8,9 INPUT
	DDRB &= ~(0 << 0); 
	DDRB &= ~(0 << 1); 
	DDRB &= ~(0 << 2);
	// Set to High
	PORTB |= (1 << 0);
	PORTB |= (1 << 1);
	PORTB |= (1 << 2);

	_delay_ms(10); /* let pins settle */

	joystickUpdate();

	SREG = sreg;
}

static void joystickUpdate(void)
{
	unsigned char tmp1=0xff;
	
	tmp1 = (PINC & 0x01); // Up/Up/Z
	tmp1 |= (PINC & 0x02); // Down/Down/Y
	tmp1 |= (PINC & 0x04); // Left/0/X
	tmp1 |= (PINC & 0x08); // Right/0
	tmp1 |= (PINC & 0x10); // B1
	tmp1 |= (PINC & 0x20); // B2
	tmp1 |= (PIND & 0x10) << 2; // B3
	tmp1 |= (PIND & 0x20) << 2; // B4

	last_read_controller_bytes[0] = tmp1;
	
	tmp1=0xff;

	tmp1 = (PIND & 0x40) >> 6; // B5
	tmp1 |= (PIND & 0x80) >> 6; // B6
	tmp1 |= (PINB & 0x01) << 2; // B7
	tmp1 |= (PINB & 0x02) << 2; // B8
	tmp1 |= (PINB & 0x04) << 2; // B9
	
	last_read_controller_bytes[1] = tmp1;

}

static char joystickChanged(unsigned char report_id)
{
	report_id--; // first report is 1

	return memcmp(	&last_read_controller_bytes[report_id<<1], 
					&last_reported_controller_bytes[report_id<<1], 
					2);
}

static char getX(unsigned char inputByte)
{
	char x = 128;
	/*if (inputByte&0x1) { x = 255; }
	if (inputByte&0x2) { x = 0; }*/
	inputByte = inputByte ^ 0x0ff;
	if (inputByte & 0x04) { x = 0x00; }  // left
	if (inputByte & 0x08) { x = 0xff; } // right
	return x;
}

static char getY(unsigned char inputByte)
{
	char y = 128;
	/*if (inputByte&0x4) { y = 255; }
	if (inputByte&0x8) { y = 0; }*/
	inputByte = inputByte ^ 0x0ff;
	if (inputByte & 0x01) { y = 0x00; } // up
	if (inputByte & 0x02) { y = 0xff; } //down
	return y;
}

/* Move the bits around so that identical NES and SNES buttons
 * use the same USB button IDs. */

static unsigned char joystickReorderButtons(unsigned char bytes[2])
{
	unsigned char v;

	/* pack the snes button bits, which are on two bytes, in
	 * one single byte. */

	v =  (bytes[0]&0x10)>>4;
    v |= (bytes[0]&0x20)>>4;
    v |= (bytes[0]&0x40)>>4;
    v |= (bytes[0]&0x80)>>4;
    v |= (bytes[1]&0x0f)<<4;

	v = v ^ 0xff;
	return v;
}

static unsigned char joystickExtraButtons(unsigned char bytes[2])
{
    unsigned char v;
    v = (bytes[1]&0x10)>>4;
	v = v ^ 0xff;
    return v;
}

static char joystickBuildReport(unsigned char *reportBuffer, char id)
{
	int idx;

	if (id < 0 || id > 4)
		return 0;

	idx = id - 1;
	if (reportBuffer != NULL)
	{
		// reportBuffer[0]=id;
		reportBuffer[0]=getX(last_read_controller_bytes[idx*2]);
		reportBuffer[1]=getY(last_read_controller_bytes[idx*2]);
		reportBuffer[2]=joystickReorderButtons(&last_read_controller_bytes[idx*2]);
		reportBuffer[3]=joystickExtraButtons(&last_read_controller_bytes[idx*2]);
	}

	memcpy(&last_reported_controller_bytes[idx*2], 
			&last_read_controller_bytes[idx*2], 
			2);

	return 4;
}

const char usbHidReportDescriptor[] PROGMEM = {

	/* Controller and report_id 1 */
    0x05, 0x01,			// USAGE_PAGE (Generic Desktop)
    0x09, 0x04,			// USAGE (Joystick)
    0xa1, 0x01,			//	COLLECTION (Application)
    0x09, 0x01,			//		USAGE (Pointer)
    0xa1, 0x00,			//		COLLECTION (Physical)
	// 0x85, 0x01,			//			REPORT_ID (1)
	0x09, 0x30,			//			USAGE (X)
    0x09, 0x31,			//			USAGE (Y)
    0x15, 0x00,			//			LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,	//			LOGICAL_MAXIMUM (255)
    0x75, 0x08,			//			REPORT_SIZE (8)
    0x95, 0x02,			//			REPORT_COUNT (2)
    0x81, 0x02,			//			INPUT (Data,Var,Abs)

    0x05, 0x09,			//			USAGE_PAGE (Button)
    0x19, 0x01,			//   		USAGE_MINIMUM (Button 1)
    0x29, 0x09,			//   		USAGE_MAXIMUM (Button 8)
    0x15, 0x00,			//   		LOGICAL_MINIMUM (0)
    0x25, 0x01,			//   		LOGICAL_MAXIMUM (1)
    0x75, 0x01,			// 			REPORT_SIZE (1)
    0x95, 0x09,			//			REPORT_COUNT (8)
    0x81, 0x02,			//			INPUT (Data,Var,Abs)
    
    0x75, 0x01,			// 			REPORT_SIZE (1)
    0x95, 0x07,			//			REPORT_COUNT (8)
    0x81, 0x01,			//			INPUT (Data,Var,Abs)
	
	0xc0,				//		END_COLLECTION
    0xc0,				// END_COLLECTION

};

Joystick joystick = {
	.num_reports 			= 1,
	.reportDescriptorSize	= sizeof(usbHidReportDescriptor),
	.init					= joystickInit,
	.update					= joystickUpdate,
	.changed				= joystickChanged,
	.buildReport			= joystickBuildReport
};

Joystick *getJoystick(void)
{
	joystick.reportDescriptor = (void*)usbHidReportDescriptor;

	return &joystick;
}

