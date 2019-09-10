#include "getVersion.h"
#include "dev_interface_def.h"
#include "I2CInterface.h" 

/** 
 * veml7700.h  
 * This is a C++ class header to read the veml7700 lux sensor
 * via the I2C interface
 *
 * version history  
 * version 0.20  : initial value not tested with the sensor 
 * version 0.21  : just change the name from VEML770 VEML7700
 * version 0.30  : start with error reporting 
 * version 0.42  : more methodes 
 * version 0.50  : added readout in lux 
 * This file make part of the PeriperalDevice package see repository  
 * https://github.com/wimbeaumont/PeripheralDevices
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2015  2019 
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 *
 **/

#define VERSION_VEML7700_HDR "0.51"


class VEML7700 : public virtual getVersion {

private :
// registers 
int gain_nr; // array index of the last set gain . 
int IntTnr ; //array index of the last set integration time
I2CInterface* i2cdev;

// a read command to the device 
u16 read_cmd ( u8 reg);
// a write command to the device    
void write_cmd( u8 reg , u16 value  );


// reg the word that has to change
// value the value that has to be set into reg
// lsb , the position of the lsb in the reg word 
// bsize the size in bits of the value to be set    
void set_bits_reg  ( u8 reg , u16 value, u16 lsb ,u16 bsize );

// write value to the reg value in the correct place ( calls set_bits_reg ) 
u16 set_bits(u16 regvalue, u16 value,  u16 lsb ,u16 bsize );


public :

VEML7700(I2CInterface* i2cinterfaceint , bool init =true );

// returns 0 if no error reported 
//    -10 for  communication error 
int get_status(void)  ;

//  set interrupt enable    
void set_int_enable( bool int_enable);

// set the bits for the ALS config register (0) in one write cycle 
void set_default_als_config ( bool shutdown=false  , bool int_enable=false ,u16 pres=0, u16 integationtime=0, u16 gain =1  );

void shutdown( bool enable) ;// if true set shutdown mode 

void set_power_saving_enable( bool ps_enable); 

void set_power_saving_mode( u16 psmode);

// set the bits for the power mode  config register (0) in one write cycle 
void set_default_powermode ( bool ps_enable=false , u16 psmode=0);

void setHighWarningLevel( int lvl);
void setLowWarningLevel( int lvl);

bool LowThresoldExeed(void) ;
bool HighThresoldExeed(void) ;

void set_gain( float gain );
void set_integrationtime( int time_ms );

// returns convert  the value readed from the als or white  register, taking in acount the gain and integration time  in to lux 
// if als   is true  the ALS register is read, else the white register is read 
// if verify is flase it  doesn't reads the config register assumes the stored gain ant integration time are corrected stored in the program 
float get_lux(bool als=true  ,bool verify_reg =false );



// these could be private but could be useful for debugging 


// write the gain bits to reg 0 
void set_gain_bits( u16 gbits ) ;
// write the integration time  bits to reg 0 
void set_IntT_bits( u16 InTgbits );


// get the gain bits depending on the value of gainsel, setgain <= gainsel  
// so 0 -> .125 ,  .126 -> .25 ,  4 -> 2  1.1 -> 2 
u16 get_gain_in_set_bit (float  gainsel );
u16 get_IntT_in_set_bit( int time_ms);
u16 get_als_bits(void);
u16 get_white_ch_bits(void);
u16 get_reg( u8 reg) { return read_cmd ( reg); }
u16 get_bits(u16 regvalue,   u16 lsb ,u16 bsize );
void set_bits_in_reg ( u8 reg , u16 value, u16 lsb ,u16 bsize ) {  set_bits_reg  ( reg ,  value,  lsb ,bsize ) ;}

float decodeGainBits( u16 gb , int& gainnr) ;
float decodeGainBits( u16 gb);
int  decodeIntTbits( u16 ib, int& InTnr ) ; 
int  decodeIntTbits( u16 ib);
u16  decode_Reg0(  bool& sd ,bool& ie, u16& pers_protect,int& IntT, float& gain  ) ;

}; //end class




