#include "ADC101_xx.h"
#include "dev_interface_def.h"


#define VERSION_ADC101_xx_SRC "0.13"

// register definition 

#define  CONV_RES     0  //16 bits
#define  CONV_RES_MSK   0x1FFC
#define  ALERT_STAT   1  // 8 bits
#define ALERT_STAT_MSK 3 
#define  CONFIG       2  // 8 bits

#define CYLE_TIME_MSK   0xE0

#define  LOW_LIM      3
#define  HIGH_LIM     4
#define  HYSTERESIS   5
#define  LOWEST_CONV  6
#define  HIGHEST_CONV 7

#define  BASE_ADDR  


ADC101_xx::ADC101_xx(I2CInterface* i2cinterface, int device_address_bits, float Vdd_, int subtype ):
    getVersion( VERSION_ADC101_xx_HDR,VERSION_ADC101_xx_SRC, __TIME__, __DATE__), _i2c_interface(i2cinterface)
{
    Vdd=Vdd_;
    // Assemble the full I2C device address.
    _device_address = 0xA8; // Prime the full device address with the device code.
    _device_address |= (device_address_bits<<1);
    _full_range = 1024; 
}

int ADC101_xx::getADCvalue(int& value, int ch){
    
    int status = read( value , 2); 
    value &= CONV_RES_MSK;
    value = value >>2; 
    return status;
}


int ADC101_xx::getVoltage(float &voltage, int ch){
  int value;
  int status =getADCvalue(value);
  voltage  =  Vdd * (float) value / 1024;
  return status; 
 }   


int ADC101_xx::read(int& value ,  int nr_bytes ) { // for the moment no pointer set  - read data 
    char data[2];
    int status = 3 ; // nr_bytes out of range 
    if(nr_bytes >0 && nr_bytes < 3) {
        // Read the raw data from the device.
        status = _i2c_interface->read(_device_address, data, nr_bytes, false);
        if ( status == 0) {
            value = (int) data[0];
            if ( nr_bytes == 2) {
                value=value << 8;
                value |= data[1];                
            }
        } // else result == i2c error code
    } // else status =  nr_bytes out of range 
    return status;
}  
        
 




int ADC101_xx::write(int reg ,  int value , int  nr_bytes , bool setptr0){   
    char data[3];
    int status ;
    //Assemble our three bytes of data - Refer to ADC101_xx ref manual, section 6.
    data[0] = (int8_t)reg;    
    if ( nr_bytes ==2) {
        data[1] = (int8_t) value;  value= value >>8;        
        data[2] = (int8_t) value;   //LSB
    }
    else { 
        data[1] = (int8_t) value;   
    }
    
    status =  _i2c_interface->write(_device_address, data, nr_bytes, false);
    if ( setptr0 && (reg != 0 ) ) { // make  sure the reg pointer points to 0 
        data[0]=(int8_t)reg;
        status |= _i2c_interface->write(_device_address, data,1, false);
    }    
    return status;
}

  int ADC101_xx::SetRegPtr( int reg) {
      char data[1]={0};
      data[0]= (int8_t) reg;
       return _i2c_interface->write(_device_address, data,1, false);
    }