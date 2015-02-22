#define DS18S20_H_IMPORT
#include "ds18s20.h"

unsigned char DS18S20_sensor_id[5][8] = {{0x31,0x00,0x00,0x00,0xB2,0x5B,0xA0,0x28},
                                 {0xCE,0x00,0x00,0x00,0xB2,0x58,0x30,0x28},
                                 {0x0B,0x00,0x00,0x00,0xB2,0x0A,0xC8,0x28},
                                 {0x2F,0x00,0x00,0x00,0xB1,0xE2,0xD6,0x28},
                                 {0xE7,0x00,0x00,0x00,0xB1,0xDD,0x69,0x28}};
unsigned char DS18S20_scratch_pad[DS18S20_NUMDEVICES][9] = {0};
unsigned char temperature_array[10] = {0};

bool DS18S20_conversionStarted = false;
unsigned short int DS18S20_convTick = 0;
bool DS18S20_startConversion = false;
bool DS18S20updated = false;

void DS18S20_setup(void){
    // Set UART IO direction
    DS18S20_UART_PIN_RX = 1;
    DS18S20_UART_PIN_TX = 0;

    DS18S20_reset();
    // configure for 10 bits resolution, and min/max alarm values
    //DS18S20_setResolution(DS18S20_NUMDEVICES);
}

void DS18S20_configureUART(unsigned int baud_rate){
    //UARTEnable(DS18S20_UART, UART_DISABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    UARTConfigure(DS18S20_UART, UART_ENABLE_HIGH_SPEED);
    UARTSetLineControl(DS18S20_UART, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(DS18S20_UART, GetPeripheralClock(), baud_rate);
    UARTEnable(DS18S20_UART, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
}

unsigned short int DS18S20_convTickInc(void){
    if (DS18S20_conversionStarted)
        DS18S20_convTick++;
    return DS18S20_convTick;
}

unsigned short int DS18S20_msCount(void){
    return DS18S20_convTick;
}

void DS18S20_setStartConversion(bool val){
    DS18S20_startConversion = val;
}

bool DS18S20_getStartConversion(void){
    return DS18S20_startConversion;
}

char * DS18S20_getTemperatureArray(void){
    return &temperature_array[0];
}

void DS18S20_queryComplete(void){
    int i, j;
    // if time expired, retrieve data
    if (DS18S20_msCount() > DS18S20_CONV_TIME){
        
        // clear conversion ticker
        DS18S20_convTick = 0;
        // conversion should be complete
        DS18S20_conversionStarted = false;
        j = 0;
        for (i = 0; i < DS18S20_NUMDEVICES; i++){
            IO_DEBUG_LAT_HIGH();
            DS18S20_readScratchPad(&DS18S20_sensor_id[i][0], &DS18S20_scratch_pad[i][0]);
            temperature_array[j++] = DS18S20_scratch_pad[i][0];
            temperature_array[j++] = DS18S20_scratch_pad[i][1];
            IO_DEBUG_LAT_LOW();
        }
        DS18S20updated = true;
    }
}

bool DS18S20_updated(void){
    return DS18S20updated;
}

void DS18S20_updatedClear(void){
    DS18S20updated = false;
}

bool DS18S20_reset(void){
    char return_val = 0x00;
    DS18S20_configureUART(9600);
    WriteUART1(0xF0);
    while(!U1ASTAbits.URXDA);
    return_val = ReadUART1();

    if (return_val == (char)0xF0){
        return false;
    }
    else{
        DS18S20_configureUART(115600);
        return true;
    }
}

void DS18S20_setResolution(int devices){
    int i;
    char scratch_pad[9] = {0};
    for (i = 0; i < devices; i++){
        DS1820_WriteEEPROM(&DS18S20_sensor_id[i][0], 0x7F, 0xFF, 0x3F);
        DS18S20_readScratchPad(&DS18S20_sensor_id[i][0], &scratch_pad[0]);
    }
}

void UART_test(void){
    int i;
    while(1){
        DS18S20_configureUART(9600);
        WriteUART1(0xAA);
        for (i = 0; i < 50000; i++);
        DS18S20_configureUART(115200);
        WriteUART1(0xAA);
        for (i = 0; i < 50000; i++);
    }
}

void DS18S20_writeByte(UINT8 byte){
    int i;
    for (i = 0; i < 8; i++){
        DS18S20_bit(byte & 0x01);
        byte >>= 1;
    }
}

UINT8 DS18S20_readByte(void){
    UINT8 byte = 0;
    int i;
    for (i = 0; i < 8; i++){
        byte >>= 1;
        byte |= (DS18S20_bit(1) << 7);
        if(U1ASTAbits.URXDA){
            bool data_there = true;
        }
    }
    return byte;
}

UINT8 DS18S20_bit(UINT8 data_bit){
    data_bit = data_bit ? (UINT8)0xFF : 0x00;
    WriteUART1(data_bit);
    while(!U1ASTAbits.URXDA);
    UINT8 returned_bit = ReadUART1();
    return (returned_bit == (UINT8)0xFF ? (UINT8)1 : (UINT8)0);
}

bool DS18S20_startConversionAll(void){
    if (!DS18S20_conversionStarted){
        DS18S20_conversionStarted = true;
        // Reset the 1-Wire bus
        if(!DS18S20_reset())
            return false;
        // Skip ROM command (0xCC) to select all devices on the bus
        DS18S20_writeByte(0xCC);
        // Convert command (0x44) to tell all DS18B20s to get the temperature
        DS18S20_writeByte(0x44);
        return true;
    }
    else
        return false;
}

void DS18S20_readScratchPad(char ROM_code[], char scratch_pad[]){
    int i;

    DS18S20_reset();
    DS18S20_writeByte(DS1820_CMD_MATCHROM);
    for (i = 7 ; i >= 0; i--){
        DS18S20_writeByte(ROM_code[i]);
    }
    // Send READ SCRATCHPAD command (0xBE) to get temperature
    DS18S20_writeByte(DS1820_CMD_READSCRPAD);
    // Read 9 bytes from scratch pad
    DS1820_readBytes(scratch_pad, 9);
}

float DS1820_getTemp(char scratch_pad[]){
    unsigned short int temp = (unsigned short int)((unsigned short int)scratch_pad[1] << 8) | scratch_pad[0];
    return (float)temp/16;
}

void DS1820_readBytes(char buffer[], int bytes){
    int i;
    for (i = 0; i < bytes; i++){
        buffer[i] = DS18S20_readByte();
    }
}

static bool bDoneFlag;
static unsigned char nLastDiscrepancy_u8;
static unsigned char nRomAddr_au8[DS1820_DEVICES][DS1820_ADDR_LEN] = {0};

bool DS1820_FindNextDevice(int device_num){
    unsigned char state_u8;
    unsigned char byteidx_u8;
    unsigned char mask_u8 = 1;
    unsigned char bitpos_u8 = 1;
    unsigned char nDiscrepancyMarker_u8 = 0;
    bool bit_b;
    bool bStatus;
    bool next_b = FALSE;

    /* init ROM address */
    for (byteidx_u8=0; byteidx_u8 < 8; byteidx_u8 ++){
        nRomAddr_au8[device_num][byteidx_u8] = 0x00;
    }

    bStatus = !DS18S20_reset();        /* reset the 1-wire */

    if (bStatus || bDoneFlag)        /* no device found */
    {
        nLastDiscrepancy_u8 = 0;     /* reset the search */
        return FALSE;
    }

    /* send search rom command */
    DS18S20_writeByte(DS1820_CMD_SEARCHROM);

    byteidx_u8 = 0;
    do{
        state_u8 = 0;

        /* read bit */
        if ( DS18S20_bit(1) != 0 ){
            state_u8 = 2;
        }
        IO_delayms(1);

        /* read bit complement */
        if ( DS18S20_bit(1) != 0 ){
            state_u8 |= 1;
        }
        IO_delayms(1);

        /* description for values of state_u8: */
        /* 00    There are devices connected to the bus which have conflicting */
        /*       bits in the current ROM code bit position. */
        /* 01    All devices connected to the bus have a 0 in this bit position. */
        /* 10    All devices connected to the bus have a 1 in this bit position. */
        /* 11    There are no devices connected to the 1-wire bus. */

        /* if there are no devices on the bus */
        if (state_u8 == 3){
            break;
        }
        else
        {
            /* devices have the same logical value at this position */
            if (state_u8 > 0){
                /* get bit value */
                bit_b = (bool)(state_u8 >> 1);
            }
            /* devices have confilcting bits in the current ROM code */
            else
            {
                /* if there was a conflict on the last iteration */
                if (bitpos_u8 < nLastDiscrepancy_u8){
                    /* take same bit as in last iteration */
                    bit_b = ( (nRomAddr_au8[device_num][byteidx_u8] & mask_u8) > 0 );
                }
                else{
                    bit_b = (bitpos_u8 == nLastDiscrepancy_u8);
                }

                if (bit_b == 0){
                    nDiscrepancyMarker_u8 = bitpos_u8;
                }
            }

            /* store bit in ROM address */
           if (bit_b != 0){
               nRomAddr_au8[device_num][byteidx_u8] |= mask_u8;
           }
           else{
               nRomAddr_au8[device_num][byteidx_u8] &= ~mask_u8;
           }

           DS18S20_bit(bit_b);

           /* increment bit position */
           bitpos_u8 ++;

           /* calculate next mask value */
           mask_u8 = mask_u8 << 1;

           /* check if this byte has finished */
           if (mask_u8 == 0){
               byteidx_u8 ++;  /* advance to next byte of ROM mask */
               mask_u8 = 1;    /* update mask */
           }
        }
    } while (byteidx_u8 < DS1820_ADDR_LEN);


    /* if search was unsuccessful then */
    if (bitpos_u8 < 65){
        /* reset the last discrepancy to 0 */
        nLastDiscrepancy_u8 = 0;
    }
    else{
        /* search was successful */
        nLastDiscrepancy_u8 = nDiscrepancyMarker_u8;
        bDoneFlag = (nLastDiscrepancy_u8 == 0);

        /* indicates search is not complete yet, more parts remain */
        next_b = TRUE;
    }

    return next_b;
}

int DS1820_FindDevices(void){
    nLastDiscrepancy_u8 = 0;
    bDoneFlag = FALSE;
    int device_num = 0;
    while(DS1820_FindNextDevice(device_num++)){
        IO_delayms(10);
    }
    return (device_num - 1);
}

void DS1820_WriteEEPROM(char ROM_code[], unsigned char nTHigh, unsigned char nTLow, unsigned char config){
    int i;
    DS18S20_reset();
    DS18S20_writeByte(DS1820_CMD_MATCHROM);
    for (i = 7 ; i >= 0; i--){
        DS18S20_writeByte(ROM_code[i]);
    }
    DS18S20_writeByte(DS1820_CMD_WRITESCRPAD); /* start conversion */
    DS18S20_writeByte(nTHigh);
    DS18S20_writeByte(nTLow);
    DS18S20_writeByte(config);

    IO_delayms(1);

    DS18S20_reset();
    DS18S20_writeByte(DS1820_CMD_MATCHROM);
    for (i = 7 ; i >= 0; i--){
        DS18S20_writeByte(ROM_code[i]);
    }
    DS18S20_writeByte(DS1820_CMD_COPYSCRPAD); /* start conversion */

    IO_delayms(10);
}


#undef DS18S20_H_IMPORT