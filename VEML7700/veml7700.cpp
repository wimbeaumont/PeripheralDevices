#include "veml7700.h"
//nclude <stdio.h>
/** 
 * veml7700.cpp  
 * implementation of the veml7700 class , see veml7700.h
 *
 * version history  
 * version 0.20  : initial value not tested with the sensor 
 * version 0.21  : just change the name from VEML770 VEML7700 
                   corrected address
 * version 0.30  : start with error reporting 
 * version 0.34  : correction  
 * version 0.46  :  corrections for correct working , more methods added 
 * version 0.50  : added readout in lux 
 * This file make part of the PeriperalDevice package see repository  
 * https://github.com/wimbeaumont/PeripheralDevices
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2015  2019 
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
 *
 **/

#define VERSION_VEML7700_SRC "0.52"

namespace VEML7700_CONST {
// the VEML7700 support only 1 I2C address 
const int i2caddr = 0x20;

// registers 

const u8 ALS_CONF_0  =0x0 ;
const u8 ALS_WH  =0x1 ;
const u8 ALS_WL  =0x2 ;
const u8 POW_SET  =0x3 ; 
const u8 ALS  =0x4 ;
const u8 WHITE  =0x5 ;
const u8 ALS_INT  =0x6 ;


// bit masks R0 and values 
const u16 ALS_SD_SIZE   =0x1 ; //shutdown  =1 
const u16 ALS_SD_LSB   =0x0 ; 
const u16 ALS_INT_EN_SIZE   =0x1 ; //enable  =1 
const u16 ALS_INT_EN_LSB   =0x1 ; 
const u16 ALS_PERS_SIZE   =0x2 ; 
const u16 ALS_PERS_LSB   =4 ; 
const u16 ALS_GAIN_SIZE   =0x2 ; 
const u16 ALS_GAIN_LSB   = 11;
const u16 ALS_IT_SIZE   =0x4; 
const u16 ALS_IT_LSB   =6; 
const u16 RESERVED_BIT_MSK_ALS_CONF_0 =  0xE40C; // these bits should be 0  so use inverse with AND 

const int NrGains =4;

// make array of struct for the next 2 ? 
u16 GainSets[NrGains] = { 0x2, 0x3 , 0x0 , 0x1 };
float gains[NrGains] = {.125, .25 , 1, 2  };

const int NrIntT =6;
u16 IntTSets[NrIntT]={ 0b1100,0b1000,0b0000,0b0001,0b0010,0b0011 };
int IntTs[NrIntT] = { 25 ,50,100,200,400, 800 };
   
const float luxref=.0576  ; // lux per bit for gain = 1 and IT =100 ms 
const int RefIntT= 100;
// min resolution gain =2  IT =800  , 0.0036 (als ch) 
// R1 and R2 are 16 words  value 
// bit mask R3 and values 
const u16 RESERVED_BIT_MSK_POW_SET =  0xFFF8; // these bits should be 0  so use inverse with AND 
const u16 PSM_EN_SIZE  =0x1 ; 
const u16 PSM_EN_LSB   =0x0 ; 
const u16 PSM_SIZE     =0x6 ; 
const u16 PSM_LSB      =0x1 ; 

// R4 and R5 are 16 words  value 
// bit mask R6 and values   status 

const u16 INT_TH_HIGH_MSK   =0x4000 ; 
const u16 INT_TH_LOW_MSK   =0x8000 ; 
}

using namespace VEML7700_CONST; 



VEML7700::VEML7700(I2CInterface*  i2c , bool init   )
  :getVersion( VERSION_VEML7700_HDR,VERSION_VEML7700_SRC, __TIME__, __DATE__),
   i2cdev(i2c) {
    i2cdev=i2c;
    gain_nr=0;
    IntTnr =0;
    if( init) {
        set_default_als_config();
        set_default_powermode();
        // don't care about the threshold settings are interrupt  not used by default
    }
    
} 

int VEML7700::get_status(void) {
        return i2cdev-> getLastComError() ;
    
}


void VEML7700::setHighWarningLevel( int lvl){ 
    u16 llvl=(u16) lvl & 0xFF;
    write_cmd(ALS_WH, llvl);
}

void VEML7700::setLowWarningLevel( int lvl){
    u16 llvl=(u16) lvl & 0xFF;
    write_cmd(ALS_WL, llvl);
}

bool VEML7700::HighThresoldExeed(void) {
     return (INT_TH_HIGH_MSK  & read_cmd(ALS_INT )) ? true:false; 
}

bool VEML7700::LowThresoldExeed(void){
     return (INT_TH_LOW_MSK  & read_cmd(ALS_INT )) ? true:false; 
}


void VEML7700::set_bits_reg  ( u8 reg , u16 value, u16 lsb ,u16 bsize ) {
    u16  regvalue =read_cmd(reg );  
    regvalue = set_bits(regvalue,  value, lsb, bsize) ;
    if (reg == ALS_CONF_0) {regvalue &= ~RESERVED_BIT_MSK_ALS_CONF_0; } 
    if (reg == POW_SET) {regvalue &= ~RESERVED_BIT_MSK_POW_SET; } 
    write_cmd( reg, regvalue ) ;
} 

u16 VEML7700::set_bits(u16 regvalue, u16 value,  u16 lsb ,u16 bsize ) {
    u16 mask = 1; 
    mask = mask << bsize ; mask = mask -1; // so bsize=3 give now mask 0  0111
    mask = mask << lsb; // put the bits in the correct place 
    regvalue  &= ~mask; // set bits to 0
    value = value << lsb; // set the value bits in the correct place. 
    value &= mask ; //only set these bits 
    return  value | regvalue;
}   

void VEML7700::shutdown( bool enable) {
    u16 ie= enable  ? 0 :1 ;
    set_bits_reg(  ALS_CONF_0,ie, ALS_SD_LSB,ALS_SD_SIZE );
}
    
void VEML7700::set_int_enable( bool int_enable) {
    u16 ie= int_enable  ? 0 :1 ;
    set_bits_reg(  ALS_CONF_0,ie, ALS_INT_EN_LSB,ALS_INT_EN_SIZE );
}

void VEML7700::set_gain_bits( u16 gbits ) {
    set_bits_reg ( ALS_CONF_0 , gbits , ALS_GAIN_LSB,ALS_GAIN_SIZE );
}

void VEML7700::set_IntT_bits( u16 InTgbits ) {
    set_bits_reg ( ALS_CONF_0 , InTgbits , ALS_IT_LSB,ALS_IT_SIZE );
}

    
void VEML7700::set_default_als_config ( bool shutdown , bool int_enable ,u16 pres, u16 integrationtime, u16 gain ) {
    u16 setvalue = shutdown? 1:0;  //set shutdown 
    u16 uinterrupt= int_enable ?1:0;
    setvalue=set_bits( setvalue , uinterrupt ,  ALS_INT_EN_LSB,ALS_INT_EN_SIZE);
    setvalue=set_bits( setvalue , pres , ALS_PERS_LSB,ALS_PERS_SIZE);
    setvalue=set_bits( setvalue , integrationtime , ALS_IT_LSB,ALS_IT_SIZE);
    setvalue=set_bits( setvalue , gain  , ALS_GAIN_LSB,ALS_GAIN_SIZE);
    setvalue &= ~RESERVED_BIT_MSK_ALS_CONF_0;
    write_cmd(  ALS_CONF_0, setvalue ) ;// it will et all the bits so no need to read first 
}

void VEML7700::set_power_saving_enable( bool ps_enable) {
    u16 pse=ps_enable ? 0 :1  ;
    set_bits_reg( POW_SET ,pse, PSM_EN_LSB, PSM_EN_SIZE );
}

void VEML7700::set_power_saving_mode( u16 psmode)       {
    set_bits_reg( POW_SET ,psmode, PSM_LSB, PSM_SIZE );
}

void VEML7700::set_default_powermode ( bool ps_enable , u16 psmode) {
    u16 setvalue = ps_enable ? 0:1;
    setvalue = set_bits( setvalue, psmode,PSM_LSB, PSM_SIZE );
    setvalue &= ~ ~RESERVED_BIT_MSK_POW_SET;
    write_cmd(  POW_SET , setvalue ) ;
}   

// error handling via DevErrorReporter 
u16 VEML7700::read_cmd ( u8 reg) {
/* seems this doesn work  returns always zero    
    printf("call read_cmd ");
    char readvalue[2]; // max 2 values read 
    // first set the reg pointer . not sure if this works
    readvalue[0] = (char) reg;
     int err= i2cdev->write (i2caddr,readvalue, 1,false) ; // Write reg to an I2C slave. 3 words with stop
     if (err) { printf("VELM7700 %d i2c err %d  in %d \n\r",err, __LINE__); }
     //now read 
    err = i2cdev->read( i2caddr, readvalue , 2, false) ;// read from the i2c dev with stop
    if (err) {printf("VELM7700 %d i2c err %d  in %d \n\r",err, __LINE__); }
    printf(" %x %x \n\r", (int) readvalue[0], (int) readvalue[1]);
    u16 rv= readvalue[1]; rv=rv<<8;
     rv|= readvalue[0];
    return rv; 
*/ 
    char readvalue[2];
    i2cdev-> read_reg( i2caddr , readvalue , 2, (int) reg, 1);
     u16 rv= readvalue[1]; rv=rv<<8;
     rv|= readvalue[0];
    return rv; 
    
}



// error handling via DevErrorReporter     
void VEML7700::write_cmd( u8 reg , u16 value    ){
    char writevalue[3]; // max 3 values to write 
    writevalue[0] = (char) reg;
    writevalue[1] = (char)( value & 0xFF); //Write LSByte first see data I2C interface 
    value =value >>8;
    writevalue[2] = (char)( value & 0xFF); //MSByte 
    i2cdev->write(i2caddr,writevalue, 3,false) ; // Write to an I2C slave. 3 words with stop    
}


u16 VEML7700::get_IntT_in_set_bit( int time_ms) {
        for ( IntTnr= 0;IntTnr <NrIntT  ;IntTnr++){
            if( time_ms < IntTs[IntTnr] ) break;
         }
         if( IntTnr == NrIntT ) { IntTnr--;}
         return    IntTSets[IntTnr];
}

void VEML7700::set_integrationtime( int time_ms ) {
    u16 gb=get_IntT_in_set_bit(  time_ms);
    set_IntT_bits( gb);
}


void VEML7700::set_gain( float gain ) {
    u16 gb= get_gain_in_set_bit (  gain );
    set_gain_bits( gb);
}


u16 VEML7700::get_gain_in_set_bit (float  gainsel ) {
    for ( gain_nr = 0; gain_nr < NrGains; gain_nr++ ){
        if ( gainsel <= gains[gain_nr] ) { break;}
    }
    if( gain_nr ==  NrGains) { gain_nr--;};
    return GainSets[gain_nr];
}


u16 VEML7700::get_als_bits(void){
    return read_cmd(ALS );
}

u16 VEML7700::get_white_ch_bits(void){
    return read_cmd(WHITE );
}


float VEML7700::get_lux(bool ALSreg , bool verify_reg ) { 
 u16 reg= ALSreg ?  ALS: WHITE;
 u16 res = read_cmd(reg); 
 if ( verify_reg ) {
    reg= ALS_CONF_0;
    reg =read_cmd(reg); 
    (void) decodeIntTbits( reg,IntTnr  ); // just set this correct 
    (void) decodeIntTbits( reg,gain_nr  ); // just set gain_nr correct 
 }
    
 return (float)res *luxref / gains[gain_nr]*RefIntT   /(float) IntTs[IntTnr] ;
}


// below are not necessay but useful fpr debugging 
u16 VEML7700::get_bits(u16 regvalue,   u16 lsb ,u16 bsize ) {
    u16 mask = 1; 
    mask = mask << bsize ; mask = mask -1; // so bsize=3 give now mask 0  0111
    mask = mask << lsb; // put the bits in the correct place 
    regvalue &= mask ; //truncate if to big    
    mask = 1; 
    mask = mask << bsize ; mask = mask -1; // so bsize=3 give now mask 0  0111
    regvalue = regvalue >> lsb; // set the value bits in the correct place. 
    return   regvalue & mask;
}   

float VEML7700::decodeGainBits( u16 gb) {
    int dummy;
    return decodeGainBits( gb, dummy ) ;
}

float VEML7700::decodeGainBits( u16 gb, int& g_nr) {
    // this function doesn't set the gain_nr as it can be used for checking only 
    for ( g_nr = 0; g_nr < NrGains; g_nr++ ){
        if ( gb == GainSets[g_nr] ) { break;}                
    }
    if( g_nr  >=  NrGains) { g_nr = NrGains-1;};
    return gains[g_nr];
}

int VEML7700::decodeIntTbits( u16 ib) {
    int dummy;
    return decodeIntTbits(  ib,dummy );
}

int VEML7700::decodeIntTbits( u16 ib, int &In_nr ) {
        // this function doesn't set the IntTnr as it can be used for checking only 
        for ( In_nr= 0;In_nr < NrIntT  ;In_nr++){
            if( ib == IntTSets[In_nr] ){ break;}            
         }
         if ( In_nr >=  NrIntT) { In_nr= NrIntT-1;  };
         return IntTs[In_nr];
}

u16 VEML7700::decode_Reg0(  bool& sd ,bool& ie, u16& pers_protect,int& IntT, float& gain  ) {
    u16 reg= read_cmd(ALS_CONF_0);
    u16 res= get_bits(reg, ALS_SD_LSB , ALS_SD_SIZE ) ;
    sd = res ? false : true;
    res= get_bits(reg,ALS_INT_EN_LSB , ALS_INT_EN_SIZE ) ;
    ie = res ? false : true;
    pers_protect= get_bits(reg,ALS_PERS_LSB , ALS_PERS_SIZE ) ;
    res= get_bits(reg,ALS_IT_LSB , ALS_IT_SIZE ) ;
    IntT=decodeIntTbits(res);
    res= get_bits(reg,ALS_GAIN_LSB , ALS_GAIN_SIZE ) ; 
    gain=decodeGainBits(res);
    return reg;
}
