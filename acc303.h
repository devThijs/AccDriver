
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


enum {
	accX = 0,
	accY,
	accZ,
	magX,
	magY,
	magZ,
}vector;

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

void acc_enableInterrupt0(uint8_t axesEvents, double threshold, uint8_t duration, interruptMode interruptMode);
void acc_enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, interruptMode interruptMode);

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

void acc_enableInterrupt0(uint8_t axesEvents, double threshold, uint8_t duration, interruptMode interruptMode);
void acc_enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, interruptMode interruptMode);
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
