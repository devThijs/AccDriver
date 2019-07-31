
#ifndef ACC303_H_
#define ACC303_H_

#include <acc303defines.h>

#define LSM303AGR_ACCEL_ADDRESS 0b0011001
#define LSM303AGR_MAG_ADDRESS 0b0011110
 uint8_t storedSetPowerMode;

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


    bool checkWhoAmI_A();
    bool checkWhoAmI_M();

  void initializeLSM(accelerometerMode powerMode);
  void setPowerMode(accelerometerMode powerMode);



//-----------Accelerometer functions-------------

 void enableAcceleroMeter();
    void disableAcceleroMeter();
    void rebootAcceleroMeter();

void setScale(scale scale);

int16_t getAccX();
int16_t getAccY();
int16_t getAccZ();

//-----------Magnetometer functions-------------

uint16_t getMagX();
uint16_t getMagY();
uint16_t getMagZ();
void softReset();
void selfTest();


    void enableMagnetoMeter();
    void disableMagnetoMeter();
    void rebootMagnetoMeter();
	uint8_t lowPassEn();
	uint8_t lowPassDis();
    void MagSoftwareReset();


    void enableInterrupt1();
    void disableInterrupt1();
    void enableInterrupt2();
    void disableInterrupt2();
    void enableMagnetometerInterrupt();
    void disableMagnetometerInterrupt();
    bool checkInterruptRegister();


uint8_t readRegister(uint8_t address, uint8_t reg);
uint16_t read2BRegister(uint8_t address, uint8_t reg);

void writeRegister(uint8_t address, uint8_t reg, uint16_t data);
void write2BRegister(uint8_t address, uint8_t reg, uint16_t data);

#endif

