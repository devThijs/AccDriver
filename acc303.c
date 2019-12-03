//-------------------------------------LSM303AGR driver--------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////
#include <atmel_start.h>
#include <stdbool.h>
#include <i2c_simple_master.h>	//ASF4 I2C driver
#include <stdio.h>

#include <stdint.h>
#include <math.h>

#include <acc303.h>
#include <acc303defines.h>


#define ACC_ADDR 0b0011001
#define MAG_ADDR 0b0011110


static struct {
accelerometerMode powerMode;
scale scaleMode;
}configs;

int16_t vectordata[6];	    //raw data; signed
double vectordataGs[6];	    //applied scale, bitsize conversion

static union bitConvert{
	struct {
		uint8_t uByteLow;
		uint8_t uByteHigh;
	};
	struct {
		int8_t sByteLow;
		int8_t sByteHigh;
	};
	
	struct {
		int16_t	fourbitsfiller : 4;
		int16_t s12bit;
	};
	struct {
		int16_t sixbitsfillers : 6;
		int16_t s10bit;
	};

	uint16_t uTwoByte;
	int16_t sTwoByte;
}bitConvert;

//---------------------------------Accelerometer functions-----------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////


void getAxes(){
	for(int x=0; x<3; x++){

		bitConvert.uTwoByte = read16bRegister(ACC_ADDR, (OUT_X_L_A + 2*x));
		vectordata[x] =	bitConvert.sTwoByte;
	}

	for (int x=0; x<6; x++){
		bitConvert.uTwoByte = read16bRegister(MAG_ADDR, (OUTX_L_REG_M + 2*x));
		vectordata[x + 3] = bitConvert.sTwoByte;
	}
}

void getAxesScaled(){
	getAxes();
	for(int x=0; x<3; x++){
		vectordataGs[x] =	acc_getGsRawValue(vectordata[x]);		
	}

	for (int x=0; x<6; x++){
	 vectordataGs[x + 3] = mag_getScaledValue(vectordata[x+3]);
	}

}

//---------------configuration variables-----------------
 //        LowPowerMode, NormalMode, or HighResMode
    //        Magnetometer always runs at 10hz LP,
    //         unless idle      

bool acc_whoAmI(){		
    return ((readRegister(ACC_ADDR, WHO_AM_I_A) == 0b00110011));		}

bool mag_whoAmI(){	
    return ((readRegister(MAG_ADDR, WHO_AM_I_M) == 0b01000000));		}


void acc_enableBDU(){
	writeRegister	(ACC_ADDR, CTRL_REG4_A, (readRegister(ACC_ADDR, CTRL_REG4_A) | (1<<BDU))			);		}

void acc_setScale(scale scale){
	writeRegister(ACC_ADDR, CTRL_REG4_A, (readRegister(ACC_ADDR, CTRL_REG4_A)	| (scale<<FS0))		);		
	configs.scaleMode = scale;
	}

void acc_init(accelerometerMode powerMode){
    acc_powerMode(powerMode);
		uint8_t ctrlReg6 = readRegister(ACC_ADDR, CTRL_REG6_A)	//set interrupt pins as active-low
								|	(1<<H_LACTIVE);
						writeRegister(ACC_ADDR, CTRL_REG6_A, ctrlReg6); 
		acc_enableBDU();
}

 void acc_enable(){		//after initial power mode set, this function is used to reactivate after slp, without the need to specify powermode again
  acc_powerMode(configs.powerMode);
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


//  power modes:
//  Low Power Mode = 8bit
//  Normal Mode = 10 bit
//  High Resolution Mode = 12bit
/////////////CTRL_REG1_A//////////////
//ODR3|ODR2|ODR1|ODR0|LPen|Zen|Yen|Xen

void acc_powerMode(accelerometerMode powerMode){		//acc_powerMode enables and sets power mode
    switch (powerMode){            //        LowPowerMode, NormalMode, or HighResMode -- 50hz low-power mode uses 7.7uA
        case LowPowerMode:	//LPen=1, low operating frequency
	        {uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)	//disable HR if active
						&	(~(1<<HR));
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4); 
					writeRegister(ACC_ADDR, CTRL_REG1_A, (0b00001111 | (ODR50Hz<<4)));
					configs.powerMode        =		LowPowerMode;
			 	break;
				 }



        case NormalMode:	//LP=0, high frequency
					{uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)	//disable HR if active
			      &	(~(1<<HR));
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4); 	
					writeRegister(ACC_ADDR, CTRL_REG1_A, (0b00000111 | (ODR400Hz<<4)));
					configs.powerMode        =		NormalMode;
				break;
				}


        case HighResMode:	//LP=0. HR=1, HrNormal1k344 highest freq
			 		{uint8_t ctrlReg4 = readRegister(ACC_ADDR, CTRL_REG4_A)    //ctrl register 4 houses high resolution enable bit
						| (1<<HR);
					writeRegister(ACC_ADDR, CTRL_REG4_A, ctrlReg4);  
					writeRegister(ACC_ADDR, CTRL_REG1_A, (0b00000111 | HrNormal1k344<<4));
					configs.powerMode        =		HighResMode;
          break;
					}


				default:
				{break;}  
		
		}          
}
/////////////////////////CTRL_REG2_A////////////////////////////
/////////////////////High Pass Filter///////////////////////////
/////////HPM1|HPM0|HPCF2|HPCF1|FDS|HPCLICK|HPIS2|HPIS1//////////



/////////////////////////CTRL_REG3_A////////////////////////////
/////////////////////////Interrupt 1////////////////////////////
//I1_CLICK|I1_AOI1|I1_AOI2|I1_DRDY1|I1_DRDY2|I1_WTM|I1_OVERRUN|--

//INT1_CFG_A        (30h)
//AOI | 6D | ZHIE/ZUPE | ZLIE/ZDOWNE | YHIE/YUPE | YLIE\YDOWNE | XHIE/XUPE | XLIE/XDOWNE
            //  ^
            //  |
            //enable interrupt on high event or direction recognition

//AOI   6D      Interrupt Mode
//0     0       OR events 
//0     1       6-direction movement recognition 
//1     0       AND event
//1     1       6D position recognition 

//check INT1_SRC_A to check interrupt source
//0 | IA | ZH | ZL | YH | YL | XH | XL
//IA - interrupt active (one ore more interrupt has been generated)
//reading this register clears IA and allows refresh of data if the latched option was chosen

//Threshold for 6D orientation detection
//Angle °   |   Threshold g |   Threshold scaled 2g, 1 LSb=16mg   
//45        |   0.71        |
//50        |   0.77        |
//60        |   0.87        |
//70        |   0.94        |
//80        |   0.98        |

//acc_enableInterrupt0(0b00111111, 0.93, 1, PositionRecognition); example
void acc_enableInterrupt0(uint8_t  axesEvents, double threshold, uint8_t duration, interruptMode interruptMode, bool latchOrNah)
{
    // setup the interrupt
    writeRegister(ACC_ADDR, INT1_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeRegister(ACC_ADDR, INT1_THS_A, acc_getScaledIntTHS(threshold));
    writeRegister(ACC_ADDR, INT1_DURATION_A, duration); //Duration the event has to be in order to trigger the interrupt, measured in N/ODR, where N is the content of the duration register

    if (latchOrNah){
        acc_setRegBits(CTRL_REG5_A, LIR_IG1);
    }
    else{
    acc_unsetRegBits(CTRL_REG5_A, LIR_IG1);
    }
    // enable interrupt generator 1 on INT1 

    acc_setRegBits(CTRL_REG3_A, INT1_AOI1);
}

/////////////////////////CTRL_REG6_A////////////////////////////
/////////////////////////Interrupt 2////////////////////////////
////////I2_CLICKen|I2_INT1|I2_INT2|BOOT_I2|P2_ACT|--|H_LACTIVE|-

//INT2_CFG_A
//INT2_SRC_A
//structured like int1

void enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, interruptMode interruptMode)
{
    // setup the interrupt
    writeRegister(ACC_ADDR, INT2_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeRegister(ACC_ADDR, INT2_THS_A, acc_getScaledIntTHS(threshold));
    writeRegister(ACC_ADDR, INT2_DURATION_A, duration); //Duration time is measured in N/ODR, where N is the content of the duration register

    // disable latching
    acc_unsetRegBits(CTRL_REG5_A, LIR_IG2);

    // enable interrupt generator 2 on INT2
    acc_setRegBits(CTRL_REG6_A, I2_INT2);
}

uint8_t acc_readInterruptSRC(){
     return readRegister(ACC_ADDR, INT1_SRC_A);
 }


/////////////////////////CTRL_REG4_A////////////////////////////
////////////////////////modes, scales///////////////////////////
/////////////BDU|BLE|FS1|FS0|HR|ST1|ST0|SPI_enable//////////////

/////////////////////////CTRL_REG5_A////////////////////////////
//////////////////configs, int latch, 4D////////////////////////
////////BOOT|FIFO_EN|--|--|LIR_INT1|D4D_INT1|LIR_INT2|D4D_INT2//



uint8_t acc_getScaledIntTHS(double threshold)
{
    uint8_t divider = 0; // divider in mg

    switch (configs.scaleMode)
    {
    case Scale2g: divider = 16;
        break;
    case Scale4g: divider = 32;
        break;
    case Scale8g: divider = 62;
        break;
    case Scale16g: divider = 186;
        break;
    default:
        break;
    }

    return trunc(threshold * 1000.0f / divider);
}

uint8_t acc_getGsRawValue(int16_t value)
{
    if (configs.powerMode == HighResMode) {
        value /= 16; // 12-bit value
        
        switch (configs.scaleMode)
        {
            case Scale2g: return value * 1.0f / 1000.0f;
            case Scale4g: return value * 2.0f / 1000.0f;
            case Scale8g: return value * 4.0f / 1000.0f;
            case Scale16g: return value * 12.0f / 1000.0f;
            default:
                break;
        }
    }
    else if (configs.powerMode == NormalMode) {
        value /= 64; // 10-bit value
        
        switch (configs.scaleMode)
        {
            case Scale2g: return value * 4.0f / 1000.0f;
            case Scale4g: return value * 8.0f / 1000.0f;
            case Scale8g: return value * 16.0f / 1000.0f;
            case Scale16g: return value * 48.0f / 1000.0f;
            default:
                break;
        }
    }

    else if (configs.powerMode == LowPowerMode) {
        value /= 256; // 8-bit value
        
        switch (configs.scaleMode)
        {
            case Scale2g: return value * 16.0f / 1000.0f;
            case Scale4g: return value * 32.0f / 1000.0f;
            case Scale8g: return value * 64.0f / 1000.0f;
            case Scale16g: return value * 192.0f / 1000.0f;
            default:
                break;
        }
    }

    return 0.0f;
}

int8_t acc_getTemperature()
{		
		acc_enableBDU();
		writeRegister(ACC_ADDR, TEMP_CFG_REG_A, (1<<TEMP_EN1));		//enable temperature data
    int16_t value = read16bRegister(ACC_ADDR, OUT_TEMP_L_A);	//read temperature data
		acc_enableBDU();

    if (configs.powerMode == HighResMode || configs.powerMode == NormalMode) {
        value /= 64; // 12-bit value
        
        return value / 4.0f + 25.0f;
    }
    else if (configs.powerMode == LowPowerMode) {
        value /= 256; // 8-bit value

        return value + 25.0f;
    }

    return 0.0f;
}

//---------------------------------Magnetometer functions------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////



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

double mag_getScaledValue(int16_t value)
{
    return (float)value * 1.5;
}


void mag_enable()
{
    // set odr, mode, systemMode
		writeRegister(MAG_ADDR, CFG_REG_A_M, 0b10010000);   //tempcompensation=1,LP=1,ODR=10hz, continuous mode
		mag_enableLowPass();

    // disable hard-iron calibration
    writeRegister(MAG_ADDR , OFFSET_X_REG_L_M, 0);
    writeRegister(MAG_ADDR , OFFSET_X_REG_H_M, 0);
    writeRegister(MAG_ADDR , OFFSET_Y_REG_L_M, 0);
    writeRegister(MAG_ADDR , OFFSET_Y_REG_H_M, 0);
    writeRegister(MAG_ADDR , OFFSET_Z_REG_L_M, 0);
    writeRegister(MAG_ADDR , OFFSET_Z_REG_H_M, 0);

    // disable offset cancellation
		writeRegister(MAG_ADDR, CFG_REG_B_M, (readRegister(MAG_ADDR, CFG_REG_B_M)	&& ~(1<<OFF_CANC))	);
}


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


//-----------------------------------Utility functions---------------------------------------
/////////////////////////////////////////////////////////////////////////////////////////////

void acc_setRegBits(uint8_t reg, uint8_t bit){
    writeRegister(ACC_ADDR, reg,( readRegister(ACC_ADDR, reg) | (1<<bit))   );
}
void acc_unsetRegBits(uint8_t reg, uint8_t bit){
    writeRegister(ACC_ADDR, reg,( readRegister(ACC_ADDR, reg) &  ~(1<<bit))   );
}



//			-----------I2C------------
//			uses ASF4 I2C functions

uint8_t readRegister(uint8_t address, uint8_t reg){
	return I2C_0_read1ByteRegister( address, reg);
}

uint16_t read16bRegister(uint8_t address, uint8_t reg){	
		bitConvert.uByteLow = I2C_0_read1ByteRegister(address, reg);
		bitConvert.uByteHigh = I2C_0_read1ByteRegister(address, (reg+1));

		return bitConvert.uTwoByte;
}


void writeRegister(uint8_t address, uint8_t reg, uint8_t data){
	I2C_0_write1ByteRegister(address,	reg,	data);
}

void write2BRegister(uint8_t address, uint8_t reg, uint16_t data){
	I2C_0_write2ByteRegister(address, reg, data);
}
