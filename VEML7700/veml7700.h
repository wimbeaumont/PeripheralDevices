#include "getVersion.h"
#include "dev_interface_def.h"
#include "I2CInterface.h" 

/** 
 * veml770.h  
 * This is a C++ class header to read the veml770 lux sensor
 * via the I2C interface
 *
 * version history  
 * version 0.2  : initial value not tested with the sensor 
 *
 * This file make part of the PeriperalDevice package see repository  
 * https://github.com/wimbeaumont/PeripheralDevices
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2015  2019 
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 *
 **/

#define VERSION_VEML770_HDR "0.20"


class VEML770 : public virtual getVersion {

private :
// registers 
int gain_nr; // array index of the last set gain . 
int IntTnr ; //array index of the last set intergration time
I2CInterface* i2cdev;

// a read command to the device 
u16 read_cmd ( u8 reg);
// a write command to the device	
void write_cmd( u8 reg , u16 value	);


// reg the word that has to change
// value the value that has to be set into reg
// lsb , the position of the lsb in the reg word 
// bsize the size in bits of the value to be set 	
void set_bits_reg  ( u8 reg , u16 value, u16 lsb ,u16 bsize );

// write value to the reg value in the correct place ( calls set_bits_reg ) 
u16 set_bits(u8 regvalue, u16 value,  u16 lsb ,u16 bsize );


public :

VEML770(I2CInterface* i2cinterfaceint , bool init =true );

//  set interrupt enable 	
void set_int_enable( bool int_enable);

void set_gain_bits( u16 gbits ) ;

// set the bits for the ALS config register (0) in one write cycle 
void set_default_als_config ( bool shutdown=true  , bool int_enable=true ,u16 pres=0, u16 integrationtime=0, u16 gain =3  );

void set_power_saving_enable( bool ps_enable); 

void set_power_saving_mode( u16 psmode);

// set the bits for the power mode  config register (0) in one write cycle 
void set_default_powermode ( bool ps_enable=false , u16 psmode=0);


void set_gain( float gain );
// return convert the value readed from the als register, taking in acount the gain and intergration factor in to lux 
float get_lux_als(void) { return 0;}

float get_lux_white(void) { return 0;}
// these could be private but could be useful for debugging 

// get the gain bits depending on the value of gainsel, setgain <= gainsel  
// so 0 -> .125 ,  .126 -> .25 ,  4 -> 2  1.1 -> 2 
u16 get_gain_bit (float  gainsel );
u16 get_als_bits(void);
u16 get_white_ch_bits(void);




}; //end class




