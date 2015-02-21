/*
 * Firmware for Aerial Avionics Boad
 * Yashren Reddi
 * Department of Electrical Engineering
 * University of Cape Town
 * June 2013
 */

#include <plib.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "./USB/usb.h"
#include "./USB/usb_function_cdc.h"

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "HardwareProfile.h"

#include "usb_callback.h"
#include "io.h"
#include "ds18s20.h"

/* Processor configuration bits */
#pragma config UPLLEN   = ON                    // USB PLL enabled
#pragma config UPLLIDIV = DIV_2                 // USB PLL input divider
#pragma config FPLLMUL = MUL_20			// PLL multiplier
#pragma config FPLLIDIV = DIV_2			// PLL input divider
#pragma config FPLLODIV = DIV_1			// PLL output divider
#pragma config FWDTEN = OFF 			// Watchdog timer enable bit, done later in software
#pragma config WDTPS = PS1024			// Set watchdog postscaler to bit after 512ms
#pragma config POSCMOD = HS			// High speed oscillator mode
#pragma config FNOSC = PRIPLL			// Use primary oscillator with PLL
#pragma config FPBDIV = DIV_1			// Set peripheral clock divisor to 1
#pragma config ICESEL = ICS_PGx2		// Use PGD and PGC pair 2 for programming
#pragma config BWP = OFF				// Boot flash is not writable during execution
#pragma config DEBUG = OFF

#define STEP_ON_BIT         (1<<0)
#define LOOP_CLOSED_BIT     (1<<1)

float Kp_temp = 0.05;
float wi_temp = 0.0056;
float wz_temp = 0.0265;
float wp_temp = 0.0315;

int main(int argc, char** argv) {
    // Disable JTAG to enable corresponding IO pin functionality
    mJTAGPortEnable(0);

    // Configure MCU for maximum performance
    SYSTEMConfigWaitStatesAndPB(GetSystemClock());	// kill wait states
    CheKseg0CacheOn();                                  // Enable cache
    mCheConfigure(CHECON | 0x30);			// Enable pre-fetch module
    mBMXDisableDRMWaitState();                          // Disable RAM wait states

    InitializeUSB();

    IO_setup();
    
    INTEnableSystemMultiVectoredInt();
    INTEnableInterrupts();

    DS18S20_setup();

    char buffer_out[64];
    char buffer_in[64];
    float duty_cycle = 0.0;
    float closed_loop_duty_cycle = 0.0;
    bool step_on_state = false;
    char checksum;
    signed char reference_temperature = 0;
    unsigned int temp;
    BYTE numBytesRead;
    char system_state = 0;  // default state

    while(1){        
        if(USB_BUS_SENSE && (USBGetDeviceState() == DETACHED_STATE)){
            USBDeviceAttach();
        }            
        //ProcessIO(&count);
        
        // If user button is pressed, make dc 75%
//        if (!PORTReadBits(IOPORT_E, BIT_6)){
//            if (step_on_state){
//                step_on_state = false;
//                duty_cycle = 0.0;
//            }
//            else{
//                step_on_state = true;
//                duty_cycle = 50.0;
//            }
//        }
        if ((system_state & 0x03) == 0b01){
            duty_cycle = 50.0;
        }
        else{
            duty_cycle = 0.0;
        }

        if ((system_state & 0x03) == 0b10){
            duty_cycle = closed_loop_duty_cycle;
        }

        SetDCOC2PWM(IO_PWM_dc_inv(duty_cycle));

        // User Application USB tasks
        if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)){
            continue;
        }
        else{
            if(USBUSARTIsTxTrfReady()){
                // Get data from USB port
                numBytesRead = getsUSBUSART(buffer_in,64);
                if (numBytesRead == 19){
                    checksum = IO_getChecksum(&buffer_in[0], 18);
                    if (checksum == buffer_in[18]){
                        system_state = buffer_in[0];
                        reference_temperature = buffer_in[1];
                        memcpy(&Kp_temp, &buffer_in[2], 4);
                        memcpy(&wi_temp, &buffer_in[6], 4);
                        memcpy(&wz_temp, &buffer_in[10], 4);
                        memcpy(&wp_temp, &buffer_in[14], 4);
                    }
                }
                if(ADC_updated()){
                    memcpy(&buffer_out[0], &duty_cycle, 4);
                    temp = (unsigned int)IO_getBatteryVoltage();
                    memcpy(&buffer_out[4], &temp, 4);
                    temp = (unsigned int)IO_get_time_ms();
                    memcpy(&buffer_out[8], &temp, 4);
                    buffer_out[12] = reference_temperature;
                    buffer_out[13] = system_state;
                    checksum = IO_getChecksum(&buffer_out[0], 14);
                    buffer_out[14] = checksum;
                    putUSBUSART(&buffer_out[0], 15);
                    ADC_update_clear();
                    float measured_temp = (float)IO_getBatteryVoltage()*100*3.3/1023;
                    closed_loop_duty_cycle = IO_temperature_control((float) reference_temperature,
                        measured_temp, 0, 100, 50e-3, false, Kp_temp, wi_temp, wz_temp, wp_temp);
                }
            }
            CDCTxService();
        }
    }

    return (EXIT_SUCCESS);
}

