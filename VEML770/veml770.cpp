#include "veml770.h"




/** 
 * veml770.cpp  
 * implementation of the veml770 class , see veml770.h
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

#define VERSION_VEML770_SRC "020"

namespace VEML770_CONST {
// the VEML770 support only 1 I2C address 
const int i2caddr = 0x10;

// registers 

const u8 ALS_CONF_0  =0x0 ;
const u8 ALS_WH  =0x1 ;
const u8 ALS_WL  =0x2 ;
const u8 POW_SET  =0x3 ; //reserved
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
const u16 ALS_IT_LSB   =6; //shutdown  =1 
const u16 RESERVED_BIT_MSK_ALS_CONF_0 =  0xE40C; // these bits should be 0  so use inverse with AND 

const int NrGains =4;

// make array of struct for the next 2 ? 
u16 GainSets[NrGains] = { 0x2, 0x3 , 0x0 , 0x1 };
float gains[NrGains] = {.125, .25 , 1, 2  };
const int NrIntT =6;

u16 IntTSets[NrIntT]={ 0b1100,0b1000,0b0000,0b0001,0b0010,0b0011 };
int IntTs[NrIntT] = { 25 ,50,100,200,400, 800 };

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

using namespace VEML770_CONST;	



VEML770::VEML770(I2CInterface*  i2c , bool init   )
  :getVersion( VERSION_VEML770_HDR,VERSION_VEML770_SRC, __TIME__, __DATE__),
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


void VEML770::set_bits_reg  ( u8 reg , u16 value, u16 lsb ,u16 bsize ) {
	u16  regvalue =read_cmd(reg );	
	regvalue = set_bits(regvalue,  value, lsb, bsize) ;
	if (reg == ALS_CONF_0) {regvalue &= ~RESERVED_BIT_MSK_ALS_CONF_0; } 
	if (reg == POW_SET) {regvalue &= ~RESERVED_BIT_MSK_POW_SET; } 
	write_cmd( reg, regvalue ) ;
} 

u16 VEML770::set_bits(u8 regvalue, u16 value,  u16 lsb ,u16 bsize ) {
	u16 mask = 1; 
	mask = mask << bsize ; mask = mask -1; // so bsize=3 give now mask 0  0111
	value &= mask ; //truncate if to big	
	mask = mask << lsb; // put the bits in the correct place 
	regvalue  &= ~mask; // set bits to 0
	value = value << lsb; // set the value bits in the correct place. 
	return  value | regvalue;
}	
	
void VEML770::set_int_enable( bool int_enable) {
	u16 ie= int_enable  ? 0 :1 ;
	set_bits_reg(  ALS_CONF_0,ie, ALS_INT_EN_LSB,ALS_INT_EN_SIZE );
}

void VEML770::set_gain_bits( u16 gbits ) {
	set_bits_reg ( ALS_CONF_0 , gbits , ALS_GAIN_LSB,ALS_GAIN_SIZE );
}
	
void VEML770::set_default_als_config ( bool shutdown , bool int_enable ,u16 pres, u16 integrationtime, u16 gain ) {
	u16 setvalue = shutdown? 0:1;  //set shutdown 
	u16 uinterrupt= int_enable ?0:1;
	setvalue=set_bits( setvalue , uinterrupt ,  ALS_INT_EN_LSB,ALS_INT_EN_SIZE);
	setvalue=set_bits( setvalue , pres , ALS_PERS_LSB,ALS_PERS_SIZE);
	setvalue=set_bits( setvalue , integrationtime , ALS_IT_LSB,ALS_IT_SIZE);
	setvalue=set_bits( setvalue , gain  , ALS_GAIN_LSB,ALS_GAIN_SIZE);
	setvalue &= ~RESERVED_BIT_MSK_ALS_CONF_0;
	write_cmd(  ALS_CONF_0, setvalue ) ;// it will et all the bits so no need to read first 
}

void VEML770::set_power_saving_enable( bool ps_enable) {
	u16 pse=ps_enable ? 0 :1  ;
	set_bits_reg( POW_SET ,pse, PSM_EN_LSB, PSM_EN_SIZE );
}

void VEML770::set_power_saving_mode( u16 psmode)		{
	set_bits_reg( POW_SET ,psmode, PSM_LSB, PSM_SIZE );
}

void VEML770::set_default_powermode ( bool ps_enable , u16 psmode) {
	u16 setvalue = ps_enable ? 0:1;
	setvalue = set_bits( setvalue, psmode,PSM_LSB, PSM_SIZE );
	setvalue &= ~ ~RESERVED_BIT_MSK_POW_SET;
	write_cmd(  POW_SET , setvalue ) ;
}	



u16 VEML770::get_gain_bit (float  gainsel ) {
	for ( gain_nr = 0; gain_nr < NrGains; gain_nr++ ){
		if ( gainsel <= gains[gain_nr] ) { break;}
	}
	return GainSets[gain_nr];
}


u16 VEML770::read_cmd ( u8 reg) {
	char readvalue[2]; // max 2 values read 
	// first set the reg pointer . not sure if this works
	readvalue[0] = (char) reg;
	 int err= i2cdev->write (i2caddr,readvalue, 1,false) ; // Write reg to an I2C slave. 3 words with stop
	 if (err) { /* throw exception*/ }
	 //now read 
	err = i2cdev->read( i2caddr, readvalue , 2, false) ;// read from the i2c dev with stop
	if (err) { /* throw exception*/ }
	u16 rv= readvalue[1]; rv=rv<<8;
	 rv|= readvalue[0];
	return rv; 
}
	
void VEML770::write_cmd( u8 reg , u16 value	){
	char writevalue[3]; // max 3 values to write 
	writevalue[0] = (char) reg;
	writevalue[1] = (char)( value & 0xFF);
	value =value >>8;
	writevalue[2] = (char)( value & 0xFF);
	int err=i2cdev->write(i2caddr,writevalue, 3,false) ; // Write to an I2C slave. 3 words with stop
	if (err) { /* throw exception*/ }
}


void VEML770::set_gain( float gain ) {
	u16 gb= get_gain_bit (  gain );
	set_gain_bits( gb);
}

u16 VEML770::get_als_bits(void){
	return read_cmd(ALS );
}


u16 VEML770::get_white_ch_bits(void){
	return read_cmd(WHITE );
}

