//-------------------------------------LSM303AGR driver--------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////


#include <atmel_start.h>
#include <stdbool.h>

#include <i2c_simple_master.h>	//ASF4 I2C driver
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
// #include <math.h>


#include <acc303.h>
#include <acc303defines.h>


#define ACC_ADDR 0b0011001
#define MAG_ADDR 0b0011110



accelerometerMode currentMode;

uint8_t      read_data[2];

uint16_t vectordata[6];







getAxes(){
	for(int x=0; x<3; x++){
		vectordata[x] = read16bRegister(ACC_ADDR, (OUT_X_L_A + 2*x));
	}
	for (int x=0; x<6; x++){
		vectordata[x + 3] = read16bRegister(MAG_ADDR, (OUTX_L_REG_M + 2*x));
	}
}

//---------------configuration variables-----------------
 //        LowPowerMode, NormalMode, or HighResMode
    //        Magnetometer always runs at 10hz LP,
    //         unless idle      

bool acc_whoAmI(){		
    return ((readRegister(ACC_ADDR, WHO_AM_I_A) == 0b00110011));
}

bool mag_whoAmI(){	
    return ((readRegister(MAG_ADDR, WHO_AM_I_M) == 0b01000000));
}
void acc_enableBDU(){
	writeRegister	(ACC_ADDR, CTRL_REG4_A, (readRegister(ACC_ADDR, CTRL_REG4_A) | (1<<BDU)	)	);
}
void acc_init(accelerometerMode powerMode){
    acc_powerMode(powerMode);
		uint8_t ctrlReg6 = readRegister(ACC_ADDR, CTRL_REG6_A)	//set interrupt pins as active-low
								|	(1<<H_LACTIVE);
						writeRegister(ACC_ADDR, CTRL_REG6_A, ctrlReg6); 
		acc_enableBDU();
}



//  power modes:
//  Low Power Mode = 8bit
//  Normal Mode = 10 bit
//  High Resolution Mode = 12bit

void acc_powerMode(accelerometerMode powerMode){		//acc_powerMode enables and sets power mode
    switch (powerMode){            //        LowPowerMode, NormalMode, or HighResMode -- 50hz low-power mode uses 7.7uA
        case LowPowerMode:
	        {uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)	//disable HR if active
						&	(~(1<<HR));
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4); 
			
			 		uint8_t ctrlReg1 = 7 			|			(1<<LPen)			|			(ODR50Hz<<ODR0)	;	/*((((~ODR50Hz)<<ODR0)) | ((1<<(ODR0))-1))  ; */   //enable LowPower mode, set ODR to 50hz
					writeRegister(ACC_ADDR, CTRL_REG1_A, ctrlReg1);

					currentMode        =		LowPowerMode;
			 	break;}


        case NormalMode:
					{uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)	//disable HR if active
			      &	(~(1<<HR));
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4); 	

					uint8_t ctrlReg1 = 7 | (ODR400Hz<<ODR0);		//disable LP bit=normal mode, ODR set to 400hz

					writeRegister(ACC_ADDR, CTRL_REG1_A, ctrlReg1);

					currentMode        =		NormalMode;
				break;}


        case HighResMode:
			 		{uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)    //ctrl register 4 houses high resolution enable bit
						| (1<<HR);
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4);  

			  	uint8_t ctrlReg1  = 7 | (HrNormal1k344<<ODR0);		//disable LP bit=, ODR set to 1k344
					writeRegister(ACC_ADDR, CTRL_REG1_A, ctrlReg1);
					currentMode        =		HighResMode;
          break;}

			default:
			{break;}  
		
		}          
}


void acc_setScale(scale scale){
	uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)	| (scale<<FS0);
}



//---------------------------------Accelerometer functions-----------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////



 void acc_enable(){		//after initial power mode set, this function is used to reactivate after slp, without the need to specify powermode again
  acc_powerMode(currentMode);
}



void acc_disable(){
    uint8_t ctrlreg1A = readRegister(ACC_ADDR, CTRL_REG1_A)
				&	((PowerDown<<ODR0) | ( (1<<ODR0)-1));   //ODR[0:3] = 0x00 power down
			writeRegister(ACC_ADDR, CTRL_REG1_A, ctrlreg1A);
}

void acc_reboot(){
    uint8_t ctrlreg5A = readRegister(ACC_ADDR, CTRL_REG5_A)
				|	(1<<BOOT);  //write reboot memory register bit
			writeRegister(ACC_ADDR, CTRL_REG5_A, ctrlreg5A);
}


//Only 1byte read working atm
//2Byteread's not working, likely due to no repeated start signal in ASF4


int16_t acc_getX(){
	if (currentMode==LowPowerMode){

	}

// 	if (currentMode == LowPowerMode){			//8bit result
// 		reinterpret8b.convertedValueUnsigned = readRegister(ACC_ADDR, OUT_X_H_A);
// 		return reinterpret8b.convertedValueSigned;
// 	}

// 	else {		//10bit result
// 		 uint16_t convertedValueI16 = ~read16bRegister(ACC_ADDR, OUT_X_L_A);
//       //invert bits
// convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
// convertedValueI16 += 1;                       //add 1
// convertedValueI16 *=-1;
// 	reinterpret.convertedValueUnsigned =	convertedValueI16;
// 		return (reinterpret.convertedValueSigned);

		
// 	}
 }



// int16_t acc_getY(){	

// 	if (currentMode == LowPowerMode){			//8bit result
// 		reinterpret8b.convertedValueUnsigned = readRegister(ACC_ADDR, OUT_Y_H_A);
// 		return reinterpret8b.convertedValueSigned;
// 	}

// 	else {		//10bit result
// 		 uint16_t convertedValueI16 = ~read16bRegister(ACC_ADDR, OUT_Y_L_A);
//       //invert bits
// convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
// convertedValueI16 += 1;                       //add 1
// convertedValueI16 *=-1;
// 	reinterpret.convertedValueUnsigned =	convertedValueI16;
// 		return (reinterpret.convertedValueSigned);
// }
// }
// int16_t acc_getZ(){
// 	if (currentMode == LowPowerMode){			//8bit result
// 		reinterpret8b.convertedValueUnsigned = readRegister(ACC_ADDR, OUT_Z_H_A);
// 	return reinterpret8b.convertedValueSigned;
// 	}

// 	else{		//10bit result
// 		uint16_t convertedValueI16 = ~read16bRegister(ACC_ADDR, OUT_Z_L_A);
//       //invert bits
// 		convertedValueI16 &= ( 0xFFFF>>(16-10) ); //but keep just the 10-bits
// 		convertedValueI16 += 1;                       //add 1
// 		convertedValueI16 *=-1;
// 		reinterpret.convertedValueUnsigned =	convertedValueI16;
// 		return (reinterpret.convertedValueSigned);
// }

// }


//---------------------------------Magnetometer functions------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////



void mag_enable(){
	writeRegister(MAG_ADDR, CFG_REG_A_M, 0b10010000);   //tempcompensation=1,LP=1,ODR=10hz, continuous mode
	mag_enableLowPass();
  return;
}


void mag_disable(){
		//go to idle mode, disable temperature compensation
uint8_t CFGregAM = (readRegister(MAG_ADDR, CFG_REG_A_M) 
				|	(0x03))		 &		(~(1<<COMP_TEMP_EN));   //MD{0:1] 0x03/0x02= idle mode
			writeRegister(MAG_ADDR, CFG_REG_A_M, CFGregAM);
			mag_disableLowPass();
			}


void mag_reboot(){
		//write reboot memory bit
	uint8_t CFGregAM = readRegister(MAG_ADDR, CFG_REG_A_M)
				| (1<<REBOOT);
			writeRegister(MAG_ADDR, CFG_REG_A_M, CFGregAM);
		//clear reboot memory bit
//   CFGregAM = readRegister(MAG_ADDR, CFG_REG_A_M)
// 				& (~(1<<REBOOT));   //clear bit, as i don't know if it auto clears
// 			writeRegister(MAG_ADDR, CFG_REG_A_M, CFGregAM);
 }


uint16_t mag_getX(){			//magnetic reading data is stored as two's compliment
	uint16_t rawValue = 0;
		rawValue =   read16bRegister(MAG_ADDR, OUTX_L_REG_M);
		return rawValue; }

uint16_t mag_getY(){
	uint16_t rawValue=0;
		rawValue =   read16bRegister(MAG_ADDR, OUTY_L_REG_M);
		return rawValue; }

uint16_t mag_getZ(){
	uint16_t rawValue=0;
		rawValue =   read16bRegister(MAG_ADDR, OUTZ_L_REG_M);
		return rawValue; }



//------------------------------------M_SoftwareReset----------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////
//the configuration registers and user registers are reset. Flash registers keep their values

void mag_softwareReset(){		
  writeRegister(MAG_ADDR, CFG_REG_A_M, (1<<SOFT_RST));	
}



//-------------------------------------Low-pass filter----------------------------------------
//////////////////////////////////////////////////////////////////////////////////////////////

uint8_t mag_enableLowPass(){
  uint8_t ctrlRegBM  = readRegister(MAG_ADDR, CFG_REG_B_M)   |   (1<<LPF);
  writeRegister(MAG_ADDR, CFG_REG_B_M, ctrlRegBM);
	return ctrlRegBM;
}


uint8_t mag_disableLowPass(){
  uint8_t ctrlRegBM  = readRegister(MAG_ADDR, CFG_REG_B_M)   &   ~(1<<LPF);
  writeRegister(MAG_ADDR, CFG_REG_B_M, ctrlRegBM);
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


void mag_enableInterrupt(){
		uint8_t CFGregC_M = readRegister(MAG_ADDR, CFG_REG_C_M     )			//bit 6 enables INT_MAG_PIN
													|	(1<<INT_MAG_PIN);																	
					writeRegister(MAG_ADDR, CFG_REG_C_M, CFGregC_M);


	uint8_t intCTRLreg_M = readRegister(MAG_ADDR, INT_CTRL_REG_M)			//note to self: "IEL"-> 0 for interrupt pulse, 1 for latched
													|	(1<<IEN)	|		(1<<IEL);																//another note: magnetometer has no AND logic for INT, if any x y z mag threshold is reached an interrupt is created
					writeRegister(MAG_ADDR, INT_CTRL_REG_M, intCTRLreg_M);
	}
	
void mag_disableInterrupt(){
	uint8_t intCTRLreg_M = readRegister(MAG_ADDR, INT_CTRL_REG_M)
													&		~(1<<IEN)	& 	~(1<<IEL);  
					writeRegister(MAG_ADDR, INT_CTRL_REG_M, intCTRLreg_M);
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

uint16_t read16bRegister(uint8_t address, uint8_t reg){	
		bitConvert.uByteLow = I2C_0_read1ByteRegister(address, reg);
		bitConvert.uByteHigh = I2C_0_read1ByteRegister(address, (reg+1));

		return bitConvert.sTwoByte;
}


void writeRegister(uint8_t address, uint8_t reg, uint16_t data){
	I2C_0_write1ByteRegister(address,	reg,	data);
}

void write2BRegister(uint8_t address, uint8_t reg, uint16_t data){
	I2C_0_write2ByteRegister(address, reg, data);
}
