
#include <stdlib.h>
#include <string.h>
#include "LTC2493.h"

#define VERSION_LTC2493_SRC "0.40"

// defauls config is read ch 0 singel ended , 50 ,60 Hz 1x and read
LTC2493::LTC2493(I2CInterface* i2cinterface ,int i2c_address, float referenceVolts, 
			unsigned int configurationinit):
			getVersion( VERSION_LTC2493_HDR,VERSION_LTC2493_SRC, __TIME__, __DATE__), _i2c_interface(i2cinterface){
	_i2c_interface=i2cinterface;
	/** The full i2c device address. */
	_device_address=i2c_address;
	currentconfig=configurationinit;
	refVolt=referenceVolts;
	last_status=writeI2C( currentconfig);
}

int LTC2493::getADCvalue (int& value, int ch , bool new_value ){
	unsigned int ADCValue;
	last_status=getRawADCvalue(ADCValue ,ch) ; // set channel ,wait for conversion 
	if ( last_status ) return last_status; 
	int rangeerror;
	value=getADCINTvalue(ADCValue, rangeerror); 
	return rangeerror; 
}

// convert  the ADC value to a int value 
// @adcvalue  the bit pattern from a read 
// @rangeerror will be set if out of range is detected value will retrun |max + 1lsb|
// @return  int value of the ADC 
int LTC2493::getADCINTvalue(unsigned int adcvalue, int& rangeerror ) {
	int value;
	adcvalue=adcvalue>>6; //only upper  24 bits of the 32 bit read are significant 
	rangeerror=0;
	if ( (unsigned int) 0x3000000 & adcvalue ) { value = (int)0x1000000;rangeerror = 5; return value; }
	if ( (unsigned int) 0x0FF0000 & adcvalue ) { value =-(int)0x1000000;rangeerror =-5; return value; }
	if (0x1000000 & adcvalue){
		adcvalue =  ~adcvalue;
		adcvalue &=  0xFFFFFF;
		adcvalue += 1;
		value = -(int) adcvalue; 
	}
	return value; 

}

int LTC2493::getVoltage (float& volt , int ch){ 
	int  value;
	int rangeerror=getADCvalue ( value, ch, true );
	volt= convert2Voltage(value);
	if (last_status) return last_status;
	return rangeerror; 
}

// set the channel config bits in current config 
void   LTC2493::set_channelconfig( int ch , int diff  ) {
	if (diff !=0 ||  diff !=1 || diff != 2) {  // keep current so check what is set 
		if ( ! (currentconfig & CONFIG_SGL) ) {diff=0 ;} //SGL not set 
		else if (currentconfig & CONFIG_ODD ) {diff=2 ;} else {diff=1;}
	}
	if ( ch==1 || ch==3 ) {currentconfig |= CONFIG_A0; } 
	else { currentconfig &= ~CONFIG_A0;}
	if (diff == 0) { 
		currentconfig &= ~CONFIG_SGL;
		if ( ch > 1 ) {currentconfig |= CONFIG_ODD; }
		else {	currentconfig &= ~CONFIG_ODD;}
	}
	if ( diff == 1 ) { currentconfig |= CONFIG_SGL;currentconfig &= ~CONFIG_ODD;}	// channel is already set 
	if ( diff == 2 ) { currentconfig |= CONFIG_SGL;currentconfig |= CONFIG_ODD;}
}

// set the enable bit 
// @enable set the enable bit  if true ,clear if false 
// @start  set also en2 bit , 

void  LTC2493::setEnable( bool enable ,bool enable2 ) {
	if (enable ) currentconfig |=  CONFIG_EN; else  currentconfig &=  ~CONFIG_EN;
	if (enable2 ) currentconfig |=  CONFIG_EN2; else  currentconfig &=  ~CONFIG_EN2;
}



// returns the ADC int value to voltage 
// @value the 24 bits ADC value + sign
// @return the corresponding voltage 
float LTC2493::convert2Voltage(int value){
	return ((float) value /(float) 16777215) * refVolt;
}

// this function just reads the ADC register ( so also restart a new conversion)(of the CCh) 
// fresh is not yet implemented 
int  LTC2493::getADCvalueCCh(unsigned int& value, bool fresh ){
	last_status=readI2c( value,3) ; // read 24 bits ,  wait if conversion is bussy 
	return last_status;
}

int  LTC2493::getRawADCvalue(unsigned int& value, int ch){
	set_channelconfig(  ch , -1  ) ;// set the channel do not change diff 
	setEnable( true,true ) ;  //enable and start 
	last_status =writeI2C(currentconfig  );
	if ( ! last_status ) last_status=readI2c( value,3) ;
	return last_status;
}
// read the ADC  value wait for conversion ready 
//@value returns the read value 
//@nr_bytes  nr of bytes to be read 
//@return  status of I2C read or other error. 
int LTC2493::readI2c(unsigned int& value ,  int nr_bytes ) { // for the moment no pointer set  - read data 
    char data[3];
    int status = 5 ; // nr_bytes out of range 
    // assume size of int >=  4 byte ) 
    value=0; //make sure all higher bits are 0 
    if((nr_bytes >0 )&& (nr_bytes <= 4)) {
		unsigned int cnt =4;
		while (status && cnt-- ) { // give a number of trails in case converion is still bussy 
        // Read the raw data from the device.
			status = _i2c_interface->read(_device_address, data, nr_bytes, false);
			if ( status == 0) {
				value = (int) data[0];
				for ( int bytecnt=1 ;bytecnt< nr_bytes; bytecnt++) {
					value=value << 8;
					value |= data[bytecnt];                
				}
			} // else result == i2c error code
		
		}//end while status is i2cerror or 0 
    } // else status =  nr_bytes out of range 
    
    return status;
} 

int LTC2493::writeI2C( unsigned  int  dataword ) {
     char data[2];
	if ( dataword < 0) dataword=0;
    data[1] = dataword & 0xFF;
    dataword = dataword >>8;
    data[0] = dataword & 0xFF;
    return _i2c_interface->write(_device_address, data, sizeof(data)/sizeof(*data), false);
}


/*

			return(((tempK - 273.15) * 9.0)  / 5.0 + 32.0);
	

unsigned int LTC2493::readTemperatureDeciK()
{
	
	readVal >>= 6;

	tempDK = (0x00FFFFFF & readVal);
	// Divide by 2 for FS = 1/2 VREF
	// Divide by 1570 (eqn from 2493 datasheet)
	// Divide by 100 to convert millivolts to decivolts
	tempDK = (tempDK * (unsigned long)referenceMillivolts) / 314000;

	// tempDK is now the temperature in deci-kelvin
	return(tempDK);
}
*/ 

        



