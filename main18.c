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
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

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

static uint16_t masterCounter;
static long			loopCnt;

#define	MIDI_BUF_MAX		8
#define	MIDI_BUF_MAX_MASK	0x07;
static uint8_t	midiEvent[MIDI_BUF_MAX][3];
static int	midiEventReadPointer;
static int	midiEventWritePointer;

static bool nowPlaying;
static uint8_t crntNote;
static uint8_t lastMod;

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

	AnalysePressure_Init();
	AnalyseTouch_init();
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
	masterCounter = 0;

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
//      Initialize
//
/*----------------------------------------------------------------------------*/
void initMain(void)
{
	int		i;

	//    ADCON1  =	0b00001111;
    TRISA   =	0b00000000;			//D-,D+
    TRISB   =	0b01010000;			//I2C master mode
    TRISC   =	0b10000000;			//SW1

	ANSEL	=	0b00000000;
	ANSELH	=	0b00000000;

	T0CON	=	0b10010111;

//    LATA    =	0b00000000;
//    LATB    =	0b00000000;
//    LATC    =	0b01000000;

	TMR0H	= 0;
	TMR0L	= 0;
	LED		= 0;

	for ( i=0; i<MIDI_BUF_MAX; i++ ){
		midiEvent[i][0] = 0;
		midiEvent[i][1] = 0;
		midiEvent[i][2] = 0;
	}
	loopCnt = 0;

	//	Iitialize H/W
	initI2c();
	LPS331AP_init();
	MPR121_init();
	BlinkM_init();
	ADXL345_init();

	initCommon();
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
			BlinkM_changeColor(pitch%12);
		}
    }
    else {
        if ( sentNoteOff == false ){
			setMidiBuffer( 0x90, pitch, 0x00 );
            if ( pitch == 0x49 ){
				pitch = 0x3C;
            }
            sentNoteOff = true;
			BlinkM_changeColor(12);
		}
    }
}
/*----------------------------------------------------------------------------*/
void pressureSensor( bool event10msec, bool event100msec)
{
	//  Pressure Sensor
#if 0
	//	Check Real Air Pressure
	float prs = LPS331AP_getPressure();
	if ( event100msec ){
		if ( prs < 950 ) prs = 950;
		else if ( prs > 1077 ) prs = 1077;
		prs -= 950;
		if ( prs > 127 ) prs = 127;
		setMidiBuffer(0xb0,0x0b,(uint8_t)prs);	// 0=950hPa...127=1077hPa
	}
#else
    float pval = LPS331AP_getPressure();
	AnalysePressure_setNewRawPressure(pval);
	if ( event10msec ){
		uint8_t mdExp;
		if ( AnalysePressure_catchEventOfPeriodic(&mdExp) == true ){
			if (( mdExp > 0 ) && ( nowPlaying == false )){
	            setMidiBuffer(0x90,crntNote,0x7f);
		        nowPlaying = true;
			    //int doremi = crntNote%12;
				//ll.color( tColorTableFloat[doremi][0], tColorTableFloat[doremi][1], tColorTableFloat[doremi][2] );
			}
			setMidiBuffer(0xb0,0x0b,mdExp);
		    if (( mdExp == 0 ) && ( nowPlaying == true )){
			    setMidiBuffer(0x90,crntNote,0);
				nowPlaying = false;
			    //ll.color( tColorTableFloat[12][0], tColorTableFloat[12][1], tColorTableFloat[12][2] );
			}
		}
    }
#endif
}
/*----------------------------------------------------------------------------*/
void acceleratorSensor( bool event10msec, bool event100msec )
{
	signed short acl[3];
	ADXL345_getAccel(acl);
    int incli = acl[0]/512;

	if ( event100msec ) setMidiBuffer(0xb0,0x01,incli&0x7f);

	uint8_t crntMod = 0;
	if ( incli > 256 ){
        //  Modulation
        incli -= 512;
		crntMod = -1*incli;
		if ( crntMod != lastMod ){
			setMidiBuffer(0xb0,0x01,crntMod);
            lastMod = crntMod;
	   }
    }
    else {
        incli -= 10;
        if ( incli < 0 ) incli = 0;
    }
}
/*----------------------------------------------------------------------------*/
void touchSensor( bool event10msec, bool event100msec )
{
	uint8_t	tch = MPR121_getTchSwData();
	AnalyseTouch_setNewTouch(tch);
	if ( event10msec ){
		uint8_t mdNote;
		if ( AnalyseTouch_catchEventOfPeriodic(&mdNote,loopCnt) == true ){
			if ( nowPlaying == true ){
				setMidiBuffer(0x90,mdNote,0x7f);
				setMidiBuffer(0x90,crntNote,0x00);
		        //int doremi = mdNote%12;
			    //ll.color( tColorTableFloat[doremi][0], tColorTableFloat[doremi][1], tColorTableFloat[doremi][2] );
			}
	        crntNote = mdNote;
		}
	}
}

/*----------------------------------------------------------------------------*/
//
//      Make MIDI Event Via Censers
//
/*----------------------------------------------------------------------------*/
static uint16_t		oldCounter;
/*----------------------------------------------------------------------------*/
void makeEventFromSensors()
{
	bool	event10msec = false, event100msec = false;

	//	Generate Timer Event
	if (( masterCounter & 0x0100 ) && !( oldCounter & 0x0100 )){
		//	10msec Event
		event10msec = true;
		loopCnt++;	// 243日で一回り
	}
	if (( masterCounter & 0x0800 ) && !( oldCounter & 0x0800 )){
		//	100msec Event
		event100msec = true;
	}
	oldCounter = masterCounter;


	//	Test with Tact Swtich
	testNoteEvent();

	//	Air Pressure Sensor
	pressureSensor(event10msec,event100msec);

	//  accelerator sensor
	acceleratorSensor(event10msec,event100msec);
	
	//  Touch Sensor
	touchSensor(event10msec,event100msec);
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
		LED = 1;
	}
}

/*----------------------------------------------------------------------------*/
//
//      Full Color LED
//
/*----------------------------------------------------------------------------*/
void lightFullColorLed( void )
{
	LED = ((loopCnt & 0x001e) == 0x0010)? 1:0;
}

/*----------------------------------------------------------------------------*/
//
//      Main Function
//
/*----------------------------------------------------------------------------*/
void main(void)
{
	uint16_t tmr;

    initMain();

	USBDeviceInit();

    while(1){
		//	Make Master Counter
		tmr = (uint16_t)TMR0L;
		tmr |= (uint16_t)(TMR0H << 8);
		masterCounter = tmr;


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


        //Application specific tasks
        makeEventFromSensors();
		sendEventToUsb();
		lightFullColorLed();
	}

	return;
}
