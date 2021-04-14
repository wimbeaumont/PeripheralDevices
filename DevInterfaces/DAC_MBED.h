#ifndef __DACMBED__
#define __DACMED__
/*  this interface is to have an integrated DAC of a MBED using 
    the same interface 
    so this will only work for MBED hardware 
    (c) Wim Beaumont  2020 
*/	    
#if defined  __MBED__

#define VERSION_DACMBED_HDR "0.11"
#include "DACInterface.h"
#include "getVersion.h"
#include "DevErrorReporter.h"



// #include  "mbed.h" already done by main 

class DAC_MBED: public DACInterface {
private : 
    	AnalogOut* Dac; 
public : 
    DAC_MBED(PinName pinnr ): getVersion( VERSION_DACMBED_HDR ,VERSION_DACMBED_HDR , __TIME__, __DATE__){   //Create an I2C Master interface
    	static AnalogOut Dac_(pinnr); 
    	Dac= &Dac_;
    	
    };
    
virtual int     setDACvalue(int dacvalue, int ch=0){Dac->write_u16( (unsigned short) dacvalue); return 0;};
virtual int     getDACvalue (int& dacvalue , int ch=0){
			float voltage=Dac->read(); voltage = voltage *(float) 0xFFFF;dacvalue=(int)voltage; return 0;
		};
virtual int     setVoltage (float voltage, int ch=0){ voltage=voltage / 3.3 ;Dac->write (voltage);   return 0;};
virtual int     getVoltage(float &voltage, int ch=0){voltage=3.3*Dac->read() ;return 0;}; // assuming full range =3.3 V 
virtual int     update() {return 0;};  // general update  by example readout all registers in the device to the class storage. 
    
}; 

#endif    
// so if not __MBED__ this is an empty file 
#endif
