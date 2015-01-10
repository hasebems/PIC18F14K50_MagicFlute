/*
 * File:   main18.c
 * Author: hasebems
 *
 * Created on 2014/12/19, 22:57
 */


#include <xc.h>

#include <stdbool.h>
#include <stdint.h>

#include "system.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_midi.h"
#include "i2cdevice.h"

#include	"analyse_pressure.h"
#include	"analyse_touch.h"

/*----------------------------------------------------------------------------*/
#ifndef _XTAL_FREQ
    /* 例：4MHzの場合、4000000 をセットする */
    #define _XTAL_FREQ 48000000
#endif
//#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000UL)))
//#define __delay_ms(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000UL)))

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config PLLEN = ON       // 4 X PLL Enable bit (Oscillator multiplied by 4)
#pragma config PCLKEN = ON      // Primary Clock Enable bit (Primary clock enabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = ON      // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 19        // Brown-out Reset Voltage bits (VBOR set to 1.9 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up bit (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = OFF      // MCLR Pin Enable bit (RA3 input pin enabled; MCLR disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config BBSIZ = OFF      // Boot Block Size Select bit (1kW boot block size)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Table Write Protection bit (Block 0 not write-protected)
#pragma config WRT1 = OFF       // Table Write Protection bit (Block 1 not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block not protected from table reads executed in other blocks)


#define	SW1         PORTCbits.RC7
#define	LED         PORTCbits.RC6
#define	LED2         PORTCbits.RC5

/*----------------------------------------------------------------------------*/
#define		USE_I2C_PRESSURE_SENSOR			1
#define		USE_I2C_ACCELERATOR_SENSOR		0
#define		USE_I2C_TOUCH_SENSOR			1
#define		USE_I2C_BLINKM					1

/*----------------------------------------------------------------------------*/
//
//      Variables
//
/*----------------------------------------------------------------------------*/
static uint8_t ReceivedDataBuffer[64] @ DEVCE_AUDIO_MIDI_RX_DATA_BUFFER_ADDRESS;
static USB_AUDIO_MIDI_EVENT_PACKET midiData @ DEVCE_AUDIO_MIDI_EVENT_DATA_BUFFER_ADDRESS;

static USB_HANDLE USBTxHandle;
static USB_HANDLE USBRxHandle;

static uint8_t pitch;
static bool sentNoteOff;

#define	MIDI_BUF_MAX		8
#define	MIDI_BUF_MAX_MASK	0x07;
static uint8_t	midiEvent[MIDI_BUF_MAX][3];
static int	midiEventReadPointer;
static int	midiEventWritePointer;

static bool nowPlaying;
static uint8_t crntNote;
static uint8_t lastMod, lastPrt;

static long			counter10msec;	//	one loop 243 days
static bool	event10msec, event100msec, event350msec;
static uint16_t timerStock;

static int i2cComErr;

/*----------------------------------------------------------------------------*/
//
//      Common Initialize
//
/*----------------------------------------------------------------------------*/
void initCommon( void )
{
	midiEventReadPointer = 0;
	midiEventWritePointer = 0;
	nowPlaying = false;
	crntNote = 96;
	lastMod = 0;
	lastPrt = 0;
	i2cComErr = 0;

	AnalysePressure_Init();
	AnalyseTouch_init();
}

/*----------------------------------------------------------------------------*/
//
//      Init I2C Hardware
//
/*----------------------------------------------------------------------------*/
void initAllI2cHw( void )
{
	initI2c();

#if USE_I2C_PRESSURE_SENSOR
	LPS331AP_init();
#endif
#if USE_I2C_TOUCH_SENSOR
	MPR121_init();
#endif
#if USE_I2C_ACCELERATOR_SENSOR
	ADXL345_init();
#endif
#if USE_I2C_BLINKM
	BlinkM_init();
#endif
}

/*----------------------------------------------------------------------------*/
//
//      Initialize
//
/*----------------------------------------------------------------------------*/
void initMain(void)
{
	int		i;

	//	PIC H/W registor
	//    ADCON1  =	0b00001111;
    TRISA   =	0b00000000;			//D-,D+
    TRISB   =	0b01010000;			//I2C master mode
    TRISC   =	0b10000000;			//SW1

	ANSEL	=	0b00000000;			//not use ADC. use PORT
	ANSELH	=	0b00000000;

	T0CON	=	0b10010111;			// 1:256 of System Clock
									//	 48/4MHz -> 46875Hz 21.333..usec
	INTCON	=	0b00000000;

//    LATA    =	0b00000000;
//    LATB    =	0b00000000;
//    LATC    =	0b01000000;

	TMR0H	= 0;					//	set TImer0
	TMR0L	= 0;
	LED		= 0;
	LED2	= 0;

	//	Initialize Variables only when the power turns on
	for ( i=0; i<MIDI_BUF_MAX; i++ ){
		midiEvent[i][0] = 0;
		midiEvent[i][1] = 0;
		midiEvent[i][2] = 0;
	}
	counter10msec = 0;
	event10msec = false;
	event100msec = false;
	event350msec = false;
	timerStock = 0;

	//	Iitialize other H/W
	initAllI2cHw();

	//	common Initialize
	initCommon();
}

/*********************************************************************
* Function: void APP_DeviceAudioMIDIInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void USBMIDIInitialize()
{
    USBTxHandle = NULL;
    USBRxHandle = NULL;

    pitch = 0x3C;
    sentNoteOff = true;

	initCommon();

    //enable the HID endpoint
    USBEnableEndpoint(USB_DEVICE_AUDIO_MIDI_ENDPOINT,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
}

/*********************************************************************
* Function: void APP_DeviceAudioMIDIInitialize(void);
*
* Overview: Initializes the demo code
*
* PreCondition: None
*
* Input: None
*
* Output: None
*
********************************************************************/
void APP_DeviceAudioMIDISOFHandler()
{
}

/*----------------------------------------------------------------------------*/
//
//      USB Callback Function
//
/*----------------------------------------------------------------------------*/
bool USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, uint16_t size)
{
    switch( (int) event )
    {
        case EVENT_TRANSFER:
            break;

        case EVENT_SOF:
            /* We are using the SOF as a timer to time the LED indicator.  Call
             * the LED update function here. */
			APP_DeviceAudioMIDISOFHandler();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            break;

        case EVENT_CONFIGURED:
            /* When the device is configured, we can (re)initialize the demo
             * code. */
            USBMIDIInitialize();
            break;

        case EVENT_SET_DESCRIPTOR:
            break;

        case EVENT_EP0_REQUEST:
            break;

        case EVENT_BUS_ERROR:
            break;

        case EVENT_TRANSFER_TERMINATED:
            break;

        default:
            break;
    }
    return true;
}

/*----------------------------------------------------------------------------*/
//
//      Generate Counter
//
/*----------------------------------------------------------------------------*/
void generateCounter( void )
{
	uint16_t tmr;

	//	Make Master Counter
	tmr = (uint16_t)TMR0L;
	tmr |= (uint16_t)(TMR0H << 8);

	//	Generate Timer Event
	if (( tmr & 0x0100 ) && !( timerStock & 0x0100 )){
		//	10msec Event ( precise time : 10.92msec )
		event10msec = true;
		counter10msec++;
	}
	else event10msec = false;

	if (( tmr & 0x0800 ) && !( timerStock & 0x0800 )){
		//	100msec Event ( precise time : 87.37msec )
		event100msec = true;
	}
	else event100msec = false;

	if (( tmr & 0x2000 ) && !( timerStock & 0x2000 )){
		//	350msec Event ( precise time : 349msec )
		event350msec = true;
	}
	else event350msec = false;

	timerStock = tmr;
}

/*----------------------------------------------------------------------------*/
//
//      Set MIDI Buffer
//
/*----------------------------------------------------------------------------*/
void setMidiBuffer( uint8_t status, uint8_t dt1, uint8_t dt2 )
{
	midiEvent[midiEventWritePointer][0] = status;
	midiEvent[midiEventWritePointer][1] = dt1;
	midiEvent[midiEventWritePointer][2] = dt2;
	midiEventWritePointer++;
	midiEventWritePointer &= MIDI_BUF_MAX_MASK;
}

/*----------------------------------------------------------------------------*/
//
//      MIDI Test Note Event
//
/*----------------------------------------------------------------------------*/
void testNoteEvent( void )
{
	/* If the user button is pressed... */
    if ( SW1 == 0 ){    //(BUTTON_IsPressed(BUTTON_DEVICE_AUDIO_MIDI) == true
        if ( sentNoteOff == true ){
			setMidiBuffer( 0x90, ++pitch, 0x7f );
            sentNoteOff = false;
#if USE_I2C_BLINKM
			BlinkM_changeColor(pitch%12);
#endif
		}
    }
    else {
        if ( sentNoteOff == false ){
			setMidiBuffer( 0x90, pitch, 0x00 );
            if ( pitch == 0x49 ){
				pitch = 0x3C;
            }
            sentNoteOff = true;
#if USE_I2C_BLINKM
			BlinkM_changeColor(12);
#endif
		}
    }
}
/*----------------------------------------------------------------------------*/
void pressureSensor( void )
{
	//  Pressure Sensor

	int	prs, err;

#if 0	//	for debug
	if ( event350msec == true ){
		//	350msec
		setMidiBuffer(0xb0,0x10,prs/10000);
		setMidiBuffer(0xb0,0x11,(prs%10000)/100);
		setMidiBuffer(0xb0,0x12,(prs%100));
	}
#endif

	err = LPS331AP_getPressure(&prs);
	if ( err != 0 ) i2cComErr = err;
	AnalysePressure_setNewRawPressure(prs);
	if ( event10msec == true ){
		uint8_t mdExp;
		if ( AnalysePressure_catchEventOfPeriodic(&mdExp) == true ){
			if (( mdExp > 0 ) && ( nowPlaying == false )){
	            setMidiBuffer(0x90,crntNote,0x7f);
		        nowPlaying = true;
			}
			setMidiBuffer(0xb0,0x0b,mdExp);
		    if (( mdExp == 0 ) && ( nowPlaying == true )){
			    setMidiBuffer(0x90,crntNote,0);
				nowPlaying = false;
			}
		}
    }
}
/*----------------------------------------------------------------------------*/
#define		MAX_ANGLE		32
const int tCnvModDpt[MAX_ANGLE] = {
	0,	0,	0,	0,	0,	0,	0,	0,
	0,	0,	0,	0,	1,	1,	2,	2,
	3,	4,	5,	6,	7,	8,	9,	10,
	12,	14,	16,	19,	22,	25,	28,	31,
};
//-------------------------------------------------------------------------
const int tCnvPrtDpt[MAX_ANGLE] = {
	0,	0,	0,	0,	0,	0,	0,	0,
	10,	20,	30,	40,	50,	60,	70,	70,
	80,	80,	80,	80,	90,	90,	90,	90,
	100,100,100,100,110,110,110,110,
};
//-------------------------------------------------------------------------
void acceleratorSensor( void )
{
	signed short acl[3] = { 0,0,0 };
	int incli, err;

	err = ADXL345_getAccel(acl);
	if ( err != 0 ) i2cComErr = err + 40;
	incli = acl[0]/512;

	if ( incli >= MAX_ANGLE ) incli = MAX_ANGLE-1;

	uint8_t modVal, prtVal;
	if ( incli < 0 ){
		prtVal = incli * (-1);
		modVal = 0;
	}
	else {
		prtVal = 0;
		modVal = incli;
	}
	
	//	lessen variation of modulation
	modVal = tCnvModDpt[modVal];
	if ( modVal > lastMod ){
		lastMod++;
		setMidiBuffer(0xb0,0x01,lastMod);
	}
	else if ( modVal < lastMod ){
		lastMod--;
		setMidiBuffer(0xb0,0x01,lastMod);
	}

	prtVal = tCnvPrtDpt[prtVal];
	if ( prtVal != lastPrt ){
		lastPrt = prtVal;
		setMidiBuffer(0xb0,0x05,lastPrt);
	}
}
/*----------------------------------------------------------------------------*/
void touchSensor( void )
{
	uint8_t	tch;
	int	err;

	err = MPR121_getTchSwData(&tch);
	if ( err != 0 ) i2cComErr = err + 80;
	AnalyseTouch_setNewTouch(tch);
	if ( event10msec ){
		uint8_t mdNote;
		if ( AnalyseTouch_catchEventOfPeriodic(&mdNote,counter10msec) == true ){
			if ( nowPlaying == true ){
				setMidiBuffer(0x90,mdNote,0x7f);
				setMidiBuffer(0x90,crntNote,0x00);
			}
			else {
				setMidiBuffer(0x90,mdNote,0x01);
				setMidiBuffer(0x90,crntNote,0x00);
			}
	        crntNote = mdNote;
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Full Color LED
//
/*----------------------------------------------------------------------------*/
const unsigned char tColorTable[13][3] = {
    //  R     G     B
    { 0xff,  0x00,  0x00  },   //  red		C
    { 0xd0,  0x30,  0x00  },   //  red		C#
    { 0xb0,  0x50,  0x00  },   //  orange	D
    { 0xa0,  0x60,  0x00  },   //  orange	D#
    { 0x80,  0x80,  0x00  },   //  yellow	E
    { 0x00,  0xff,  0x00  },   //  green	F
    { 0x00,  0x80,  0x80  },   //  green	F#
    { 0x00,  0x00,  0xff  },   //  blue		G
    { 0x20,  0x00,  0xe0  },   //  blue		G#
    { 0x40,  0x00,  0xc0  },   //  violet	A
    { 0x60,  0x00,  0xa0  },   //  violet	A#
    { 0xc0,  0x00,  0x40  },   //  violet	B
    { 0x00,  0x00,  0x00  }    //  none
};
//-------------------------------------------------------------------------
void lightFullColorLed( void )
{
	//	Heartbeat
	LED = ((counter10msec & 0x001e) == 0x0000)? 1:0;		//	350msec

	//	Debug by using LED
	if ( i2cErr == true ){
		if ( event100msec == true ){
			LED2 = 0;
			i2cErr = false;
		}
		else LED2 = 1;
	}

	//	Debug by using USB MIDI
	if ( event100msec == true ){
		if ( i2cComErr != 0 ){
			setMidiBuffer(0xb0,0x10,(unsigned char)i2cComErr);
		}
	}

	//	PWM Full Color LED
	int doremi = crntNote%12;
	unsigned char pwm = (unsigned char)((timerStock>>2) & 0x00ff);

	if ( nowPlaying == false ) doremi = 12;
	PORTCbits.RC2 = (pwm >= tColorTable[doremi][0])? 1:0;
	PORTCbits.RC1 = (pwm >= tColorTable[doremi][1])? 1:0;
	PORTCbits.RC0 = (pwm >= tColorTable[doremi][2])? 1:0;
}

/*----------------------------------------------------------------------------*/
//
//      Send one event to USB ( just only one event for each time )
//
/*----------------------------------------------------------------------------*/
void sendEventToUsb( void )
{
	if ( midiEventReadPointer != midiEventWritePointer ){

		midiData.Val = 0;   //must set all unused values to 0 so go ahead
                                    //  and set them all to 0
		midiData.CableNumber = 1;
        midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
        midiData.DATA_0 = midiEvent[midiEventReadPointer][0];     // Status Byte
        midiData.DATA_1 = midiEvent[midiEventReadPointer][1];     // Data Byte 1
        midiData.DATA_2 = midiEvent[midiEventReadPointer][2];     // Data Byte 2
        USBTxHandle = USBTxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&midiData,4);

		midiEventReadPointer++;
		midiEventReadPointer &= MIDI_BUF_MAX_MASK;
	}
}

/*----------------------------------------------------------------------------*/
//
//      Main Function
//
/*----------------------------------------------------------------------------*/
void main(void)
{
    initMain();

	USBDeviceInit();

    while(1){
		generateCounter();

		//	USB
		USBDeviceTasks();
        if ( USBGetDeviceState() < CONFIGURED_STATE ){
            /* Jump back to the top of the while loop. */
            continue;
        }
        if ( USBIsDeviceSuspended() == true ){
            /* Jump back to the top of the while loop. */
            continue;
        }
		if ( USBHandleBusy(USBTxHandle) == true ){
			continue;
		}
	    if ( !USBHandleBusy(USBRxHandle) ){
		    //We have received a MIDI packet from the host, process it and then
			//  prepare to receive the next packet

	        //INSERT MIDI PROCESSING CODE HERE

		    //Get ready for next packet (this will overwrite the old data)
			USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
		}

		//	Test with Tact Swtich
		testNoteEvent();

		//  Touch Sensor
#if USE_I2C_TOUCH_SENSOR
		touchSensor();
#endif

		//	Air Pressure Sensor
#if USE_I2C_PRESSURE_SENSOR
		pressureSensor();
#endif

		//  accelerator sensor
#if USE_I2C_ACCELERATOR_SENSOR
		acceleratorSensor();
#endif
		//	Display
		lightFullColorLed();

		//	USB MIDI Out
		sendEventToUsb();
	}

	return;
}
