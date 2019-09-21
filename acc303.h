
#ifndef ACC303_H_
#define ACC303_H_



#include <acc303defines.h>
#include <i2c_simple_master.h>
#include <stdio.h>

#include <stdint.h>

#ifdef __cplusplus
	extern "C"{
#endif


#define ACC_ADDR 0b0011001
#define MAG_ADDR 0b0011110

union Reinterpret {										  //change datatype without changing bit values: reinterpret using unions-
	uint16_t convertedValueUnsigned;
	int16_t convertedValueSigned;
}reinterpret;
// union Reinterpret reinterpret;

union Reinterpret8b {										//change datatype without changing bit values: reinterpret using unions-
	uint8_t convertedValueUnsigned;
	int8_t convertedValueSigned;
}reinterpret8b;
// union Reinterpret8b reinterpret8b;

union {
	struct {
		uint8_t uByteLow;
		uint8_t uByteHigh;
		};
	struct {
		int8_t sByteLow;
		int8_t sByteHigh;
		};

	uint16_t uTwoByte;
	int16_t sTwoByte;
}bitConvert;

enum {
	accX = 0,
	accY,
	accZ,
	magX,
	magY,
	magZ,
}vector;

accelerometerMode currentMode;

uint8_t      read_data[2];

uint16_t vectordata[6];


/** Structure passed into read_handler to describe the actions to be performed by the handler */



//-----------Accelerometer functions-------------
bool acc_whoAmI();
void acc_init(accelerometerMode powerMode);
void acc_powerMode(accelerometerMode powerMode);
void acc_enable();
void acc_disable();
void acc_reboot();
void acc_enableBDU();

void acc_setScale(scale scale);

int16_t acc_getX();
int16_t acc_getY();
int16_t acc_getZ();

//-----------Magnetometer functions-------------


bool mag_whoAmI();

uint8_t mag_enableLowPass();
uint8_t mag_disableLowPass();
void mag_softwareReset();
void mag_enable();
void mag_disable();
void mag_reboot();

uint16_t mag_getX();
uint16_t mag_getY();
uint16_t mag_getZ();
void softReset();
void selfTest();


void enableInterrupt1();
void disableInterrupt1();
void enableInterrupt2();
void disableInterrupt2();
void mag_enableInterrupt();
void mag_disableInterrupt();
bool checkInterruptRegister();


uint8_t readRegister(uint8_t address, uint8_t reg);
uint16_t read16bRegister(uint8_t address, uint8_t reg);

void writeRegister(uint8_t address, uint8_t reg, uint16_t data);
void write2BRegister(uint8_t address, uint8_t reg, uint16_t data);

#ifdef __cplusplus
}
#endif

#endif
