#include "ltc2633.h"
//#include "mbed.h"
#include <cstdio>


#define VERSION_LTC2633_SRC  "0.10"  

namespace LTC2633_CONST{
    
    const short BITS8 =256;
    const short BITS10 =1024;
    const short BITS12 =4096;
    const int ALLCH=15;

// CMD's 
    const char WRT_INPREG_N=0;
    const char UPDATE_REG_N=1;
    const char WRT_UPDATE_POW_REG_N=2; 
    const char WRT_UPDATE_REG_N=3; 
    const char PWR_DOWN_N=4;
    const char PWR_DOWN_ALL=5;
    const char REF_INT=6;
    const char REF_EXT=7;
  
}; // end namespace 

using namespace LTC2633_CONST ;
LTC2633::LTC2633(I2CInterface* i2cinterface,  int device_address,  float Vext, int  Vreftype,int resolution_in ): 
    getVersion( VERSION_LTC2633_HDR,VERSION_LTC2633_SRC, __TIME__, __DATE__),_i2c_interface(i2cinterface) {
           
    _device_address = (device_address<<1);
    resolution = BITS8; // default
    if ( resolution_in ==10) { resolution = BITS10;}
    if ( resolution_in ==12) { resolution = BITS12;}
    Vrefint=1.25;
    if( Vreftype == 1) Vrefint=2.048;
    vref=InternRef;
    
}



int LTC2633::setVoltage (float voltage, int ch, bool active){   
    short int value = volt2dig( voltage);
    return  setDACvalue( (int) value, ch, active);
        
};



/*
float LTC2633::dig2volt( short int value, GainMode gain,  VrefMode  vref ){
	float vout =(float)value / 4096;
        if ( vref == ExternRef ) {
            vout = Vdd * vout;
        }
        else {
            vout = VREFVOLTAGE * vout; 
            if ( gain == GainX2 ) vout = vout*2;
        }   
        return vout; 
    }
*/ 

short int LTC2633::volt2dig( float voltage ){ 
        float tmp;
        tmp= resolution  * voltage;
        if ( vref == ExternRef ) {
          tmp= tmp/Vrefext ;                                
        }    
        else {
           tmp = tmp /  Vrefint;
        }
        return (short int) ( tmp +.5 );    
}


int    LTC2633::chkch( int ch) {if( !(ch ==0 || ch==1 || ch==ALLCH) )return ch; else  return -111;}

//this assumes ch is valid 
char   LTC2633::cmd_ch_byte( int ch , int cmd ){
	 ch=0xF & ch;7;
	cmd=cmd & 0xF;
	cmd = cmd <<4;
	cmd = cmd | ch;
	return (char) cmd ;
} 

int  LTC2633::setRefIntern (void ) {   
    char cmd=REF_INT;
    int value=0; // don't care
    int ch=0;// don't care
    return writeI2C( cmd, ch , value);    
	
}

int  LTC2633::setRefExtern (void ) {   
    char cmd=REF_EXT;
    int value=0; // don't care
    int ch=0;// don't care
    return writeI2C( cmd, ch , value);    
}

 int LTC2633::setPowerMode(int ch, int pm){
    int value =0;
    int resvalue=0;       
    char cmd= 8;// invalid 
    if ( pm == PowerDownAll ) {
        cmd = PWR_DOWN_ALL;
        ch=ALLCH;// but doesn't matter
        resvalue = writeI2C( cmd, ch , value);        
    }
    else {
         ch = chkch(  ch);
        if(ch < 0) return ch;     
        if ( pm == PowerDown) cmd=PWR_DOWN_N;
        if ( pm == Normal ) cmd=UPDATE_REG_N; // power up don't change setting
    };
    if ( cmd < 7 ) return writeI2C( cmd, ch , value);
    else return -112;   
}


int LTC2633::setDACvalue( int value, int ch , bool activate ){
    ch = chkch(  ch);
    if(ch < 0) return ch;
    if ( resolution == BITS8) { value &= 0xFF; value = value <<8;}
    if ( resolution == BITS10){ value &= 0x3FF; value = value <<6;};
    if ( resolution == BITS12){ value &= 0xFFF ;value = value <<6;}; 
    char cmd = WRT_INPREG_N;
    if ( activate) cmd = UPDATE_REG_N;
    return writeI2C( cmd, ch , value);
 
}

int LTC2633::writeI2C( char  cmd , int ch, int  dataword ) {
     char data[3];	
    if ( dataword < 0) dataword=0;
    data[2] = dataword && 0xFF;
    dataword = dataword >>8;
    data[1] = dataword && 0xFF;
    data[0]= cmd_ch_byte( cmd ,ch);
    return _i2c_interface->write(_device_address, data, sizeof(data)/sizeof(*data), false);
}
