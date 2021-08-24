#ifndef __ADCMBED__
#define __ADCMED__
/*  this interface is to have an integrated ADC of a MBED using 
    the same interface as for the I2C devices 
    so this will only work for MBED hardware 
    Each ADC needs it own instantiation. So you can not select a ch 
    (c) Wim Beaumont  2021 
*   Ver 0.1  inital version
* *   Ver 1.0  tested  version 
*/	    
#if defined  __MBED__

#define VERSION_ADCMBED_HDR "1.00"
#include "ADCInterface.h"
#include "getVersion.h"
//#include "DevErrorReporter.h"


class ADC_MBED: public ADCInterface , public DevErrorReporter  {
private : 
    	AnalogIn* ADC; 
public : 
    ADC_MBED(PinName pinnr ):getVersion( VERSION_ADCMBED_HDR ,VERSION_ADCMBED_HDR , __TIME__, __DATE__){   //Create an I2C Master interface
    	static AnalogIn ADC_(pinnr,3.3); 
    	ADC= &ADC_;
    	
    };
    
    
virtual int     getADCvalue (int& adcvalue , int ch=0){// ch is ignorred 
					notsupported=false;short unsigned int value = ADC->read_u16();adcvalue=(int)value;return 0;
				}; 
virtual int     getVoltage (float& voltage, int ch=0){ notsupported=false; voltage=ADC->read_voltage();  return 0;};
virtual int     update() {notsupported=true;return -1;};  // general update  by example readout all registers in the device to the class storage. 

virtual int     startConversion(int ch=0){notsupported=true;return -1;};
virtual int     statusConversion( int& status, int ch=0){status=1; notsupported=true;return -1;};
virtual int     getFullRange( ){notsupported=true;return 256;} 
virtual int		get_reference_voltage(float &refvolt ){notsupported=false;;refvolt=ADC->get_reference_voltage()  ;return 0;}     

    
}; 

#endif    
// so if not __MBED__ this is an empty file 
#endif
