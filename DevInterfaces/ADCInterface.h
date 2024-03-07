#ifndef __ADCNTERFACE__
#define __ADCINTERFACE__

/*
 *  This is an abstraction for and ADC  I2CInterface class 
 *  So different ADC channels can be accessed in the same way 
 *  Only a limitted (common) functions for an ADC are defined and so not all features of the 
 *  ADC can will be used. 
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *  For more info see 	the README.md in the top of repository 
 *  the class can be used as it is for dummy 
 *  version 0.11  : initial version 
 *  version 0.13  : only coding 
 *  version 0.14  : added DevError reporter
 *  version 0.15  : added getADCvalue with overvlow ,  change getfull range returns unsigned int 
 
 * (C) Wim Beaumont Universiteit Antwerpen 2019 --2024 
 *
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 */




#define VERSION_ADCINTERFACE_HDR "0.15"
#include "getVersion.h"
#include "DevErrorReporter.h" 

class ADCInterface: public  DevErrorReporter, public virtual getVersion{
private: 


public : 
    ADCInterface():getVersion( VERSION_ADCINTERFACE_HDR ,VERSION_ADCINTERFACE_HDR , __TIME__, __DATE__){};   
    //  value is int assumes 32 bits so enough for most ADC's ( < 31 bits) 
    // assume pure ADC value so without status bits , value can be negative 
    virtual int getADCvalue (int& value, int ch=0 , bool new_value=false  ){ value=(unsigned int)  0 ; return 0;};
    virtual int getADCvalue (int& value,int& overflow, int ch=0 , bool new_value=false  ){overflow=0; value=(unsigned int)  0 ; return 0;};
    virtual int getVoltage (float& volt , int ch=0){ volt=(float) 0;   return 0;};
    virtual int     startConversion(int ch=0){return 0;};
    virtual int     statusConversion( int& status, int ch=0){status=0;  return status;};
    virtual unsigned int     getFullRange(void ){return (unsigned int) 256;} 
    // returns the in class stored reference voltage 
    virtual float   get_reference_voltage(void ){return 0;}     
}; 

#endif
