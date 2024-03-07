#ifndef ADC101_xx_H
#define ADC101_xx_H


#include "stdbool.h"

#include "I2CInterface.h" 
#include "ADCInterface.h" 

#define VERSION_ADC101_xx_HDR "0.10"

/** ADC101_xx class.
 *  Used for interfacing with a ADC101_xx Analoge to digital converter of Texas Instruments
 *  To convert between the xxbit ADCvalue and Vin 
 *  the referecene input voltage, Vref is the voltage connected to the Vdd pin of the device. Typically Vdd will be 3.3volts.
 *
 *  Note: There is an accompanying test suite program "ADC101_xx_test" that can be used to test this library.
 *
 *  ver 0.1 : initial verion not connected to the hardware  only basic ADC funnctions , no ALERT or min / max read support
 *                  hs mode not supported 
 * (C) Wim Beaumont Universiteit Antwerpen 2018        
 *  
 */
class ADC101_xx : public ADCInterface {
    
    public:
   
   
    /** Create an ADC101_xx I2C interface
     * 
     * @param i2cinterface pointer to the  I2C interface 
     * @param device_address_bits  the value of the address pins ( if available) 
     * @param Vdd_  the power supply voltage ( == full scale voltage
     * @param  subtype  version of the ADC101  (only 21 is supported initally ) 
     */
    ADC101_xx(I2CInterface* i2cinterface,  int device_address_bits, float Vdd_, int subtype=21);
    
   

    virtual int startConversion(int ch=0) {return 0;}
    virtual int statusConversion( int& status, int ch=0){status=1; return 0;};   
    virtual int getADCvalue(int &value, int ch=0);
    virtual int getVoltage(float &voltage, int ch=0);
    virtual unsigned int     getFullRange( ){return _full_range;} 
    int SetRegPtr( int reg);
    protected:
    /** mbed I2C interface driver. */
    I2CInterface* _i2c_interface;
    /** The full i2c device address. */
    int _device_address;
    float Vdd;
    unsigned int _full_range;
    
    int write(int reg ,  int value , int  nr_bytes , bool setptr0=true );
    int read(int& value ,  int nr_bytes ); // for the moment no pointer set  - read data 

};

#endif
