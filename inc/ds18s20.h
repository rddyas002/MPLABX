/* 
 * File:   ds18s20.h
 * Author: reddi
 *
 * Created on 20 February 2015, 11:45 AM
 */

#ifndef DS18S20_H
#define	DS18S20_H

#include <plib.h>
#include <stdbool.h>
#include "io.h"

#ifdef DS18S20_H_IMPORT
	#define DS18S20_EXTERN
#else
	#define DS18S20_EXTERN extern
#endif

#define DS18S20_UART_PIN_RX       TRISFbits.TRISF2
#define DS18S20_UART_PIN_TX       TRISFbits.TRISF8
#define DS18S20_UART              UART1
#define DS18S20_DisableIntUARTTX  DisableIntU1TX

#define DS1820_ADDR_LEN       8
#define DS1820_DEVICES        10

#define DS1820_CMD_SEARCHROM     0xF0
#define DS1820_CMD_READROM       0x33
#define DS1820_CMD_MATCHROM      0x55
#define DS1820_CMD_SKIPROM       0xCC
#define DS1820_CMD_ALARMSEARCH   0xEC
#define DS1820_CMD_CONVERTTEMP   0x44
#define DS1820_CMD_WRITESCRPAD   0x4E
#define DS1820_CMD_READSCRPAD    0xBE
#define DS1820_CMD_COPYSCRPAD    0x48
#define DS1820_CMD_RECALLEE      0xB8

DS18S20_EXTERN void DS18S20_setup(void);
DS18S20_EXTERN bool DS18S20_reset(void);
DS18S20_EXTERN UINT8 DS18S20_bit(UINT8 data_bit);
DS18S20_EXTERN UINT8 DS18S20_readByte(void);
DS18S20_EXTERN void DS18S20_readScratchPad(char ROM_code[], char scratch_pad[]);
DS18S20_EXTERN void DS18S20_configureUART(unsigned int baud_rate);
DS18S20_EXTERN void UART_test(void);
DS18S20_EXTERN bool DS1820_FindNextDevice(int device_num);
DS18S20_EXTERN int DS1820_FindDevices(void);
DS18S20_EXTERN bool DS18S20_startConversionAll(void);
DS18S20_EXTERN void DS1820_readBytes(char buffer[], int bytes);
DS18S20_EXTERN void DS1820_WriteEEPROM(char ROM_code[], unsigned char nTHigh, unsigned char nTLow);
DS18S20_EXTERN void DS18S20_writeBit(UINT8 data_bit);
DS18S20_EXTERN char DS18S20_readBit(void);

#endif	/* DS18S20_H */

