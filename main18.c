/*
 * File:   main18.c
 * Author: hasebems
 *
 * Created on 2014/12/19, 22:57
 */


#include <xc.h>
#include <p18f14k50.h>

#include "system.h"
#include "usb.h"
#include "usb_device.h"
#include "usb_device_midi.h"
#include "i2cdevice.h"


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1L
#pragma config CPUDIV = NOCLKDIV// CPU System Clock Selection bits (No CPU System Clock divide)
#pragma config USBDIV = OFF     // USB Clock Selection bit (USB clock comes directly from the OSC1/OSC2 oscillator block; no divide)

// CONFIG1H
#pragma config FOSC = ERC       // Oscillator Selection bits (External RC oscillator)
#pragma config PLLEN = OFF      // 4 X PLL Enable bit (PLL is under software control)
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
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RA3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
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

static uint8_t expression;

static USB_VOLATILE uint8_t msCounter;
static int	masterCounter;


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
	expression = 0;

    msCounter = 0;
	masterCounter = 0;

    //enable the HID endpoint
    USBEnableEndpoint(USB_DEVICE_AUDIO_MIDI_ENDPOINT,USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);

    //Re-arm the OUT endpoint for the next packet
    USBRxHandle = USBRxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&ReceivedDataBuffer,64);
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
            //APP_LEDUpdateUSBStatus();
            break;

        case EVENT_SUSPEND:
            /* Update the LED status for the suspend event. */
            //APP_LEDUpdateUSBStatus();
            break;

        case EVENT_RESUME:
            /* Update the LED status for the resume event. */
            //APP_LEDUpdateUSBStatus();
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
//    ADCON1  =	0b00001111;
    TRISA   =	0b00000000;			//D-,D+
    TRISB   =	0b01010000;			//I2C master mode
    TRISC   =	0b10000000;			//SW1
//    LATA    =	0b00000000;
//    LATB    =	0b00000000;
//    LATC    =	0b00000000;
//    LATD    =	0b00000000;
//    LATE    =	0b00000000;
    LED     =   1;

	//	Iitialize H/W
	initI2c();
	LPS331AP_init();
	BlinkM_init();
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
        /* and we haven't sent a transmission in the past 100ms... */
		if ( msCounter == 0 ){
            /* and we have sent the NOTE_OFF for the last note... */
            if ( sentNoteOff == true ){
                /* and we aren't currently trying to transmit data... */
                    //Then reset the 100ms counter
                msCounter = 100;
                midiData.Val = 0;   //must set all unused values to 0 so go ahead
                                    //  and set them all to 0
                midiData.CableNumber = 1;
                midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
                midiData.DATA_0 = 0x90;         //Note on
                midiData.DATA_1 = pitch;         //pitch
                midiData.DATA_2 = 0x7F;  //velocity
                USBTxHandle = USBTxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&midiData,4);

                /* we now need to send the NOTE_OFF for this note. */
                sentNoteOff = false;
                LED = 0;
				BlinkM_changeColor(pitch%12);
			}
        }
    }
    else {
        if ( msCounter == 0 ){
            if ( sentNoteOff == false ){
                //Debounce counter for 100ms
                msCounter = 100;
                midiData.Val = 0;   //must set all unused values to 0 so go ahead
                                    //  and set them all to 0
                midiData.CableNumber = 1;
                midiData.CodeIndexNumber = MIDI_CIN_NOTE_ON;
                midiData.DATA_0 = 0x90;     //Note off
                midiData.DATA_1 = pitch++;     //pitch
                midiData.DATA_2 = 0x00;        //velocity

                if ( pitch == 0x49 ){
                    pitch = 0x3C;
                }

                USBTxHandle = USBTxOnePacket(USB_DEVICE_AUDIO_MIDI_ENDPOINT,(uint8_t*)&midiData,4);
                sentNoteOff = true;
                LED = 0;
				BlinkM_changeColor(12);
			}
        }
    }
}

/*----------------------------------------------------------------------------*/
//
//      Make MIDI Event Via Censers
//
/*----------------------------------------------------------------------------*/
typedef enum {
	//AIR_PRESSURE,
	TEST_NOTE_EVENT,
	TOUCH_SENSOR,
	ACCELERATION_SENSOR,
	MAX_TASK_ELEMENT
} TASK_ELEMENT;
static TASK_ELEMENT taskElmCounter = TEST_NOTE_EVENT;
/*----------------------------------------------------------------------------*/
void makeEventViaSensors()
{
	switch (taskElmCounter){
		default:
		case TEST_NOTE_EVENT:{
			//	Test Note Event
			testNoteEvent();
			break;
		}
		case TOUCH_SENSOR:{

			break;
		}
		case ACCELERATION_SENSOR:{
			break;
		}
	}

	taskElmCounter++;
	if ( taskElmCounter >= MAX_TASK_ELEMENT ) taskElmCounter = 0;
}

/*----------------------------------------------------------------------------*/
//
//      Full Color LED
//
/*----------------------------------------------------------------------------*/
void lightFullColorLed( void )
{

}

/*----------------------------------------------------------------------------*/
//
//      Main Function
//
/*----------------------------------------------------------------------------*/
void main(void) {

    initMain();

	USBDeviceInit();

    while(1){
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
        makeEventViaSensors();

		lightFullColorLed();
	}

	return;
}
