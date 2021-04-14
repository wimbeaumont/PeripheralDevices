#ifndef __ADCNTERFACE__
#define __ADCINTERFACE__


#define VERSION_ADCINTERFACE_HDR "0.13"
#include "getVersion.h"

class ADCInterface: public virtual getVersion{
private: 


public : 
    ADCInterface():getVersion( VERSION_ADCINTERFACE_HDR ,VERSION_ADCINTERFACE_HDR , __TIME__, __DATE__){};   
    
virtual int     getADCvalue (int& dacvalue , int ch=0){dacvalue=0;return 0;};
virtual int     getVoltage (float& voltage, int ch=0){ voltage=0;   return 0;};
virtual int     update() {return 0;};  // general update  by example readout all registers in the device to the class storage. 

virtual int     startConversion(int ch=0){return 0;};
virtual int     statusConversion( int& status, int ch=0){status=1; return 0;};
virtual int     getFullRange( ){return 256;} 
virtual int     get_reference_voltage(float  &refvolt ){refvolt=0;return 0;}     
}; 

#endif
