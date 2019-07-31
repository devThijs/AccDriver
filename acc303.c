

//-------------------------------------LSM303AGR driver--------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////


#include <atmel_start.h>
#include <stdbool.h>
#include <stdlib.h>
#include <i2c_simple_master.h>
#include <stdio.h>
#include <util\delay.h>
#include <stdint.h>
// #include <math.h>


#include <acc303.h>
#include <acc303defines.h>
#include <hwProgHelperFunctions.h>

#define LSM303AGR_ACCEL_ADDRESS 0b0011001
#define LSM303AGR_MAG_ADDRESS 0b0011110



//---------------configuration variables-----------------
 //        LowPowerMode, NormalMode, or HighResMode
    //        Magnetometer always runs at 10hz LP,
    //         unless idle      



bool checkWhoAmI_A()
{		
    return ((readRegister(LSM303AGR_ACCEL_ADDRESS, WHO_AM_I_A) == 0b0011001));
}
bool checkWhoAmI_M()
{	
    return ((readRegister(LSM303AGR_MAG_ADDRESS, WHO_AM_I_M) == 0b01000000));
}

void initializeLSM(accelerometerMode powerMode){
    setPowerMode(powerMode);
		uint8_t ctrlReg6 = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG6_A)	//set interrupt pins as active-low
								|	(1<<H_LACTIVE);
						writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG6_A, ctrlReg6); 
}



//  power modes:
//  Low Power Mode = 8bit
//  Normal Mode = 10 bit
//  High Resolution Mode = 12bit

void setPowerMode(accelerometerMode powerMode){		//setPowerMode enables and sets power mode
    switch (powerMode){            //        LowPowerMode, NormalMode, or HighResMode -- 50hz low-power mode uses 7.7uA
        case LowPowerMode:

	        {uint8_t ctrlReg4 = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A)	//disable HR if active
						&	(~(1<<HR));
					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A, ctrlReg4); 
			
			 		uint8_t ctrlReg1 = 7 			|			(1<<LPen)			|			(ODR50Hz<<ODR0)	;	/*((((~ODR50Hz)<<ODR0)) | ((1<<(ODR0))-1))  ; */   //enable LowPower mode, set ODR to 50hz
					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG1_A, ctrlReg1);

					storedSetPowerMode        =		LowPowerMode;
			 	break;}

        case NormalMode:
		
					{uint8_t ctrlReg4 = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A)	//disable HR if active
			      &	(~(1<<HR));
					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A, ctrlReg4); 	

					uint8_t ctrlReg1 = 7 | (ODR400Hz<<ODR0);		//disable LP bit=normal mode, ODR set to 400hz

					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG1_A, ctrlReg1);

					storedSetPowerMode        =		NormalMode;
				break;}

        case HighResMode:
			 		{uint8_t ctrlReg4 = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A)    //ctrl register 4 houses high resolution enable bit
						| (1<<HR);
					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A, ctrlReg4);  

			  	uint8_t ctrlReg1  = 7 | (HrNormal1k344<<ODR0);		//disable LP bit=, ODR set to 1k344
					writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG1_A, ctrlReg1);
					storedSetPowerMode        =		HighResMode;
          break;}

			default:
			{break;}  }          
}


void setScale(scale scale){
	uint8_t ctrlReg4 = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A)	| (scale<<FS0);
	writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG4_A, ctrlReg4);
}





//---------------------------------Accelerometer functions-----------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////



 void enableAcceleroMeter(){		//after initial power mode set, this function is used to reactivate after slp, without the need to specify powermode again
  setPowerMode(storedSetPowerMode);
}



void disableAcceleroMeter(){
    uint8_t ctrlreg1A = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG1_A)
				&	((PowerDown<<ODR0) | ( (1<<ODR0)-1));   //ODR[0:3] = 0x00 power down
			writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG1_A, ctrlreg1A);
}

void rebootAcceleroMeter(){
    uint8_t ctrlreg5A = readRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG5_A)
				|	(1<<BOOT);  //write reboot memory register bit
			writeRegister(LSM303AGR_ACCEL_ADDRESS, CTRL_REG5_A, ctrlreg5A);
}


//Only 1byte read working atm
//2Byteread's not working, likely due to no repeated start signal in ASF4

int16_t getAccX(){

	if (storedSetPowerMode == LowPowerMode){			//8bit result
		reinterpret8b.convertedValueUnsigned = readRegister(LSM303AGR_ACCEL_ADDRESS, OUT_X_H_A);
		return reinterpret8b.convertedValueSigned;
	}

	else {		//10bit result
		 uint16_t convertedValueI16 = ~read2BRegister(LSM303AGR_ACCEL_ADDRESS, OUT_X_L_A);
      //invert bits
convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
convertedValueI16 += 1;                       //add 1
convertedValueI16 *=-1;
	reinterpret.convertedValueUnsigned =	convertedValueI16;
		return (reinterpret.convertedValueSigned);

		
	}
}



int16_t getAccY(){	

	if (storedSetPowerMode == LowPowerMode){			//8bit result
		reinterpret8b.convertedValueUnsigned = readRegister(LSM303AGR_ACCEL_ADDRESS, OUT_Y_H_A);
		return reinterpret8b.convertedValueSigned;
	}

	else {		//10bit result
		 uint16_t convertedValueI16 = ~read2BRegister(LSM303AGR_ACCEL_ADDRESS, OUT_Y_L_A);
      //invert bits
convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
convertedValueI16 += 1;                       //add 1
convertedValueI16 *=-1;
	reinterpret.convertedValueUnsigned =	convertedValueI16;
		return (reinterpret.convertedValueSigned);
}
}
int16_t getAccZ(){
	if (storedSetPowerMode == LowPowerMode){			//8bit result
		reinterpret8b.convertedValueUnsigned = readRegister(LSM303AGR_ACCEL_ADDRESS, OUT_Z_H_A);
		return reinterpret8b.convertedValueSigned;
	}

	else{		//10bit result
		 uint16_t convertedValueI16 = ~read2BRegister(LSM303AGR_ACCEL_ADDRESS, OUT_Z_L_A);
      //invert bits
convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
convertedValueI16 += 1;                       //add 1
convertedValueI16 *=-1;
	reinterpret.convertedValueUnsigned =	convertedValueI16;
		return (reinterpret.convertedValueSigned);
}

}


//---------------------------------Magnetometer functions------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////



void enableMagnetoMeter(){
	writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, 0b10010000);   //tempcompensation=1,LP=1,ODR=10hz, continuous mode
	lowPassEn();
  return;
}


void disableMagnetoMeter(){
		//go to idle mode, disable temperature compensation
uint8_t CFGregAM = (readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M) 
				|	(0x03))		 &		(~(1<<COMP_TEMP_EN));   //MD{0:1] 0x03/0x02= idle mode
			writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, CFGregAM);
			lowPassDis();
			}


void rebootMagnetoMeter(){
		//write reboot memory bit
	uint8_t CFGregAM = readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M)
				| (1<<REBOOT);
			writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, CFGregAM);
		//clear reboot memory bit
//   CFGregAM = readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M)
// 				& (~(1<<REBOOT));   //clear bit, as i don't know if it auto clears
// 			writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, CFGregAM);
 }


uint16_t getMagX(){			//magnetic reading data is stored as two's compliment
	uint16_t rawValue = 0;
		rawValue =   read2BRegister(LSM303AGR_MAG_ADDRESS, OUTX_L_REG_M);
		return rawValue; }

uint16_t getMagY(){
	uint16_t rawValue=0;
		rawValue =   read2BRegister(LSM303AGR_MAG_ADDRESS, OUTY_L_REG_M);
		return rawValue; }

uint16_t getMagZ(){
	uint16_t rawValue=0;
		rawValue =   read2BRegister(LSM303AGR_MAG_ADDRESS, OUTZ_L_REG_M);
		return rawValue; }



//------------------------------------M_SoftwareReset----------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////
//the configuration registers and user registers are reset. Flash registers keep their values

void MagSoftwareReset(){		
  writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_A_M, (1<<SOFT_RST));	
}



//-------------------------------------Low-pass filter----------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////

uint8_t lowPassEn(){
  uint8_t ctrlRegBM  = readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_B_M)   |   (1<<LPF);
  writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_B_M, ctrlRegBM);
	return ctrlRegBM;
}


uint8_t lowPassDis(){
  uint8_t ctrlRegBM  = readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_B_M)   &   ~(1<<LPF);
  writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_B_M, ctrlRegBM);
	return ctrlRegBM;
}



//---------------------------------------INTERRUPTS--------------------------------------------
///////////////////////////////////////////////////////////////////////////////////////////////

void enableInterrupt1(){
	
		
	}
void disableInterrupt1(){
		
	}
void enableInterrupt2(){
		
	}
void disableInterrupt2(){
		
	}


void enableMagnetometerInterrupt(){
		uint8_t CFGregC_M = readRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_C_M     )			//bit 6 enables INT_MAG_PIN
													|	(1<<INT_MAG_PIN);																	
					writeRegister(LSM303AGR_MAG_ADDRESS, CFG_REG_C_M, CFGregC_M);


	uint8_t intCTRLreg_M = readRegister(LSM303AGR_MAG_ADDRESS, INT_CTRL_REG_M)			//note to self: "IEL"-> 0 for interrupt pulse, 1 for latched
													|	(1<<IEN)	|		(1<<IEL);																//another note: magnetometer has no AND logic for INT, if any x y z mag threshold is reached an interrupt is created
					writeRegister(LSM303AGR_MAG_ADDRESS, INT_CTRL_REG_M, intCTRLreg_M);
	}
	
void disableMagnetometerInterrupt(){
	uint8_t intCTRLreg_M = readRegister(LSM303AGR_MAG_ADDRESS, INT_CTRL_REG_M)
													&		~(1<<IEN)	& 	~(1<<IEL);  
					writeRegister(LSM303AGR_MAG_ADDRESS, INT_CTRL_REG_M, intCTRLreg_M);
	}
bool checkInterruptRegister(){																										//true if int MROI flag is high 
//nothing yet
			}


//-----------------------------------Utility functions---------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////
//I2C communication



//			-----------I2C------------
//			uses ASF4 I2C functions

uint8_t readRegister(uint8_t address, uint8_t reg){
	return I2C_0_read1ByteRegister( address, reg);
}

uint16_t read2BRegister(uint8_t address, uint8_t reg){		//not working, likely because no repeated start signal
		return I2C_0_read2ByteRegister(address, reg);
}


void writeRegister(uint8_t address, uint8_t reg, uint16_t data){
	I2C_0_write1ByteRegister(address,	reg,	data);
}

void write2BRegister(uint8_t address, uint8_t reg, uint16_t data){
	I2C_0_write2ByteRegister(address, reg, data);
}
