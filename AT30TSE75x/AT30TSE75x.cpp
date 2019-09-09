
#include "AT30TSE75x.h" 

/*
 *  info see  AT30SE75x.h 
 * (C) Wim Beaumont Universiteit Antwerpen 2017

 *   some parts of the code are copied 
 *   from https://developer.mbed.org/users/akhilpanayam/code/AT30TSE75X/file/0e430cef393b/AT30TSE75X.h
 *   ver 0.91  removed mbed.h  should not be there platform depending 
 *   ver 0.96  edit changes also some corrections for compiler warnings
*/


#define VERSION_AT30TSE75x_SRC  "0.96"  

#define RegisterLocDownEnable 0 // set this to one to enable permanent NV register locking , never tested with 1 

#define AT30TSE75X_ADD_TEMP    0x48 /*Temperature Sensor: 0b1001xxx */
#define AT30TSE75X_ADD_EEPROM  0x50 /*EEPROM: 0b1010xxx */
#define AT30TSE75X_FIX_EEPROM  0x62 /*fix EEPROM  0b01100010  ( last 0 = W) */
#define AT30TSE752                      1
#define AT30TSE754                      2
#define AT30TSE758                      3

#define AT30TSE_CONFIG_RES_9_bit    0
#define AT30TSE_CONFIG_RES_10_bit   1
#define AT30TSE_CONFIG_RES_11_bit   2
#define AT30TSE_CONFIG_RES_12_bit   3


#define AT30TSE_TEMPERATURE_REG_SIZE    2

#define CopyNV2VolReg 0xB8
#define CopyVolV2NVReg 0x48

#define TReg 0x0  // 16 bits 
#define ConfigReg 0x1  // 16 bits 
#define TLowReg 0x2  // 16 bits 
#define THighReg 0x3  // 16 bits 
#define ConfigRegNV 0x11 // 16 bits 
#define TLowRegNV 0x12  // 16 bits 
#define THighRegNV 0x13  // 16 bits 

  /*  for no all fixed values           ..5432 1098 7654 3210
        15 normal : 1     ONSHOT        0b1xxx xxxx xxxx xxxx
        14:13  12 bit :11 RESOLUTION    0b1RRx xxxx xxxx xxxx
        12:11  1 fault:00 FaultTQ       0b111F Fxxx xxxx xxxx
        10     0 act low  AlertPol      0b1110 0Axx xxxx xxxx
        9      0 cmp mode               0b1110 00Cx xxxx xxxx
        8      0 temp act  SHUTDOWNM    0b1110 000S xxxx xxxx
        7:1      dont care 0b1110 0000 0000 000x
        0        read only 0b1110 0000 0000 0000 
        =>  0xE0 
    */

// config 
#define ONSHOT_BM       0x8000
#define RESOLUTION_BM   0x6000
#define FaultTQ_BM      0x1800
#define ALERTPOL_BM     0x0400
#define AlarmMode_BM    0x0200
#define SHUTDOWN_BM     0x0100
#define NVREGBUSY_BM    0x0001
#define REGLOCKDOWN_BM  0x0004
#define REGLOCK_BM      0x0002


AT30TSE75x::AT30TSE75x (I2CInterface* i2cinterface,  int device_address_bits,int eepromsize):
    getVersion( VERSION_AT30TSE75x_HDR,VERSION_AT30TSE75x_SRC, __TIME__, __DATE__),_i2c(i2cinterface) {
    Taddr= ((device_address_bits & 0x7) |  AT30TSE75X_ADD_TEMP ) & 0x7F; Taddr=Taddr<<1;    
    Eaddr= ((device_address_bits & 0x7) |  AT30TSE75X_ADD_EEPROM ) & 0x7F;  Eaddr=Eaddr<<1;
    
    Esize=1;
    if( eepromsize ==4 ) {Esize=2; }
	if( eepromsize ==8)  {Esize=3; }    
    read_config(initstatus);
    read_config(initstatus,1);
}   


uint16_t  AT30TSE75x::set_temperature(float temperature ) {
    uint16_t td;
    uint16_t  sign= 0x8000;
    if ( temperature  < 0)  { temperature *= -1; }
    else { sign=0; }
    // this function is only used to calculate limits so we don't care
    // about resolution settings, all is 12 bits
    temperature=temperature *16;
    td=(int)temperature;
    if (sign){ td = ~td; td=td+1;}
    td= td<<4;
    td=td|sign;
    return td;    
        
}
    

float  AT30TSE75x::convert_temperature( uint16_t datain) {
    uint16_t data=datain;
    float temperature;    
    float sign = 1;
 
    /* Check if negative and clear sign bit. */
    if (data & (1 << 15)) {
        sign *= -1;
        data = ~data;
        data +=1;
    }
 
    /* Convert to temperature. */
    switch (Creg.resolution) {
    case AT30TSE_CONFIG_RES_9_bit:
        data = (data >> 7);
        temperature = data * sign * 0.5;
        break;
 
    case AT30TSE_CONFIG_RES_10_bit:
        data = (data >> 6);
        temperature = data * sign * 0.25;
        break;
 
    case AT30TSE_CONFIG_RES_11_bit:
        data = (data >> 5);
        temperature = data * sign * 0.125;
        break;
 
    case AT30TSE_CONFIG_RES_12_bit:
        data = (data >> 4);
        temperature = data * sign * 0.0625;
        break;
 
    default:
		// should never come here so return no exisiting temperature	
		temperature=-300;
        break;
    }
    return temperature;

}
    
float AT30TSE75x::get_temperature(int &error) {
    return convert_temperature( get_temperature_register(error));
}

uint16_t AT30TSE75x::get_temperature_register(int &error){   
    
    uint16_t data;
    int locerr=-200;
    buffer[0] = 0;
    buffer[1] = 0;
    
    locerr=_i2c->read(Taddr,(char *)buffer,AT30TSE_TEMPERATURE_REG_SIZE,false);
    data = (buffer[0] << 8) | buffer[1];
    
    error=locerr;  // pointer !=0 
    return   data;
}     


void AT30TSE75x::set_TLowLimitRegister(int &error ,  float temperture, int Nonvolatile) {
    uint8_t reg = TLowReg ;
    if ( Nonvolatile) reg = TLowRegNV;
    set_LimitRegister(error ,reg ,  temperture,  Nonvolatile);
}
 
void AT30TSE75x::set_THighLimitRegister(int &error ,  float temperture, int Nonvolatile) {
    uint8_t reg = THighReg ;
    if ( Nonvolatile) reg = THighRegNV;
    set_LimitRegister(error ,reg ,  temperture,  Nonvolatile);
}    
void AT30TSE75x::set_LimitRegister(int &error ,uint8_t reg  ,float temperature, int Nonvolatile) {
    int locerr;
    if( Nonvolatile &&  CregNV.RegisterLockDown ) { error=43; return ;}
    if(  CregNV.RegisterLock) { error=44; return; }
    buffer[0]= reg;
    uint16_t td=set_temperature(temperature);
    uint16_t td_low= td & 0xFF;
    buffer[2] = (uint8_t) td_low;
    td = td >> 8; td= td & 0xFF;
    buffer[1] = (uint8_t) td;
    locerr=_i2c->write(Taddr,(char *)buffer,3,false);
    buffer[0]=  TReg;
     locerr= locerr<<2;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // make sure temp read reg is active again 
    error= locerr;
}

float AT30TSE75x::get_THighLimitRegister(int &error ,   int Nonvolatile) {
    uint8_t reg = THighReg ;
    if ( Nonvolatile) reg = THighRegNV;
    return get_LimitRegister(error ,reg ,  Nonvolatile);
}
 
float AT30TSE75x::get_TLowLimitRegister(int &error ,  int Nonvolatile) {
    uint8_t reg = TLowReg ;
    if ( Nonvolatile) reg = TLowRegNV;
    return get_LimitRegister(error ,reg ,  Nonvolatile);
}
    
float  AT30TSE75x::get_LimitRegister(int &error ,uint8_t reg  , int Nonvolatile) {
    int locerr;
    
    buffer[0]= reg;
    locerr=_i2c->write(Taddr,(char *)buffer,1,false); // set reg addres to read
    error = locerr<<2;
    locerr =  _i2c->read(Taddr,(char *)buffer,AT30TSE_TEMPERATURE_REG_SIZE,false);
    uint16_t data = (buffer[0] << 8) | buffer[1];
    buffer[0]=  TReg;
    locerr= locerr<<2;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // make sure temp read reg is active again 
    error= locerr;
    return convert_temperature(  data);
}
    
void AT30TSE75x::CopyNonVolatile2VolatileRegisters( int &error) {
    if ( CregNV.RegisterLock ) { error=43; return ;}
    if ( CregNV.RegisterLockDown ) { error=44; return ;}
    CopyRegisters( error,CopyNV2VolReg);    
}

void AT30TSE75x::CopyVolatile2NoneVolatileRegisters( int &error) { 
    if ( CregNV.RegisterLock ) { error=43; return ;}
    CopyRegisters( error, CopyVolV2NVReg);
}    

void AT30TSE75x::CopyRegisters( int &error , uint8_t reg) {  
    int locerr;
    bool busy=true;
    int lc=3;
    while ( busy && lc--) { 
        (void) read_config(locerr,0);
        if ( locerr ) { error =locerr ; return;}
        busy =Creg.NVregBusy;
        if(busy) _i2c->wait_for_ms(100);
    }
    if ( busy ) { error =22; return ;}
    
    buffer[0]= reg;
    locerr=_i2c->write(Taddr,(char *)buffer,1,false);
    // not sure if this is needed  
    buffer[0]=  TReg;
    locerr= locerr<<2;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // make sure temp read reg is active again 
    error= locerr;
}   


uint16_t AT30TSE75x::read_config(int &error, int Nonvolatile ) {
    uint16_t data;
    uint16_t td;
    int locerr;
    if ( Nonvolatile) buffer[0]=  ConfigRegNV; else buffer[0]=  ConfigReg;
     locerr=_i2c->write(Taddr,(char *)buffer,1,false);
     locerr= locerr<<2;
    locerr|=_i2c->read(Taddr,(char *)buffer,AT30TSE_TEMPERATURE_REG_SIZE,false);
     locerr= locerr<<2;
    data = (buffer[0] << 8) | buffer[1];
    buffer[0]=  TReg;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // set back to read Treg
    error=locerr;  
    if ( locerr == 0) {
        struct configreg *lcreg;
        if(  Nonvolatile) lcreg =&CregNV; else lcreg =&Creg;     
        lcreg->oneshot =bool  ( data  & ONSHOT_BM); // should be 0 when reading
        td = data &  RESOLUTION_BM ; td = td >> 13; lcreg->resolution =td;
        td = data & FaultTQ_BM ; td = td >> 11;  lcreg->faultTolQueue=td;  
        lcreg->alertpol =bool  ( data  & ALERTPOL_BM);
        lcreg->alarmMode =bool  ( data  & AlarmMode_BM);
        lcreg->shutdownMode =bool  ( data  & SHUTDOWN_BM);
        lcreg->RegisterLockDown =bool  ( data  & REGLOCKDOWN_BM);
        lcreg->RegisterLock =bool  ( data  & REGLOCK_BM);
        lcreg->NVregBusy =bool  ( data  & NVREGBUSY_BM);
    }
    return (int)data;
}    


int AT30TSE75x::get_NonevolatileBussy(int &error){
       read_config(error, 0 ) ;
       return (int) Creg.NVregBusy;
    }
    
void AT30TSE75x::set_resolution(int resolution , int &error,  int Nonvolatile,bool write, bool update ){
    int locerr;
    if( resolution ==12 ) resolution =   AT30TSE_CONFIG_RES_12_bit;
    if( resolution ==11 ) resolution =   AT30TSE_CONFIG_RES_11_bit;
    if( resolution ==10 ) resolution =   AT30TSE_CONFIG_RES_10_bit;
    if( resolution ==9 ) resolution =   AT30TSE_CONFIG_RES_9_bit;
    if ( resolution >=0 && resolution <=3 ) {
        if ( update ) read_config(locerr, Nonvolatile ) ;
        error=locerr<<2;
        if(  Nonvolatile) CregNV.resolution=resolution; else Creg.resolution=resolution;
        if(write ) set_config(locerr,  Nonvolatile ) ;
        error |= locerr;
    }
    error=36; // invalid resolution    
 }   

void AT30TSE75x::set_FaultTollerantQueue(int ftq, int &error, int Nonvolatile, bool write, bool update ){
   int locerr;
   if ( ftq >=0 && ftq <=3 ) {
        if ( update ) read_config(locerr, Nonvolatile ) ;
        error=locerr<<2;
        if(  Nonvolatile) CregNV.faultTolQueue=ftq; else Creg.faultTolQueue=ftq;
        if(write ) set_config(locerr,  Nonvolatile ) ;
        error |= locerr;
    }
    error=36; // invalid value
}   


void AT30TSE75x::set_AlertPinPolarity(int pol, int &error,  int Nonvolatile,bool write, bool update ){
        int locerr;
        bool mode=(bool) pol;
        if ( update ) read_config(locerr, Nonvolatile ) ;
        error=locerr<<2;
        if(  Nonvolatile) CregNV.alertpol=mode; else Creg.alertpol=mode;
        if(write ) set_config(locerr,  Nonvolatile ) ;
        error |= locerr;
}

void AT30TSE75x::set_AlarmThermostateMode(int modein, int &error,  int Nonvolatile,bool write, bool update ){
        int locerr;
        bool mode=(bool) modein;
        if ( update ) read_config(locerr, Nonvolatile ) ;
        error=locerr<<2;
        if(  Nonvolatile) CregNV.alarmMode=mode; else Creg.alarmMode=mode;
        if(write ) set_config(locerr,  Nonvolatile ) ;
        error |= locerr;
}


void AT30TSE75x::set_ShutdownMode(int modein, int &error,  int Nonvolatile,bool write, bool update ){
        int locerr;
        bool mode=(bool) modein;
        if ( update ) read_config(locerr, Nonvolatile ) ;
        error=locerr<<2;
        if(  Nonvolatile) CregNV.shutdownMode=mode; else Creg.shutdownMode=mode;
        if(write ) set_config(locerr,  Nonvolatile ) ;
        error |= locerr;
}
void AT30TSE75x::set_FaultTollerantQueue(char nrfaults, int &error, int Nonvolatile, bool write, bool update ){
    int ftq=7;
    if ( nrfaults == '1') ftq=0;
    if ( nrfaults == '2') ftq=1;
    if ( nrfaults == '4') ftq=2;
    if ( nrfaults == '6') ftq=3;
    set_FaultTollerantQueue( ftq,error, Nonvolatile,  write,  update);
}

void AT30TSE75x::activate_oneshot(int &error  ){
    int locerr;
    buffer[0]=  ConfigReg;
    locerr=_i2c->write(Taddr,(char *)buffer,1,false);
    locerr= locerr<<2;
    locerr|=_i2c->read(Taddr,(char *)buffer,AT30TSE_TEMPERATURE_REG_SIZE,false);
    locerr= locerr<<2;
    buffer[0]=  ConfigReg;
    buffer[1] = 0x80  ; //set once shot MSByte 
    buffer[2] = 0 ; // volataille  
    locerr=_i2c->write(Taddr,(char *)buffer,3,false);
    buffer[0]=  TReg;
    locerr= locerr<<2;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // make sure temp read reg is active again 
    error=locerr;
}    

void AT30TSE75x::set_RegisterLock (int &error, int lock   ){
     int locerr;
     (void)read_config(locerr,1 );
     error= locerr<<2;
     CregNV.RegisterLock    =lock;
     set_config(locerr,1 );
     error |= locerr;
}    
    
void AT30TSE75x::set_RegisterLockdown (int &error, int lockpermanent  ){
    int locerr;
    if( RegisterLocDownEnable) {
         if ( error != 123) {error=36; return;}
         (void)read_config(locerr,1 );
          error= locerr<<2;
          CregNV. RegisterLockDown   = lockpermanent;
          set_config(locerr,1 );
         error |= locerr;
    }
    else { error =36 ;}
}    
    


void AT30TSE75x::set_config( int &error, int Nonvolatile ) {
    int locerr;
    struct configreg *lcreg=&Creg;
    if ( Creg.RegisterLockDown && Nonvolatile  ) { locerr = 43; return; }
    if ( Creg.RegisterLock  ) { locerr = 44; return; }
    if ( Nonvolatile) buffer[0]=  ConfigRegNV; else buffer[0]=  ConfigReg; 
    if(  Nonvolatile) lcreg =&CregNV; else lcreg =&Creg; 
    uint16_t ld=0, td;
    if ( lcreg->oneshot) ld= ONSHOT_BM;  // no effect for NV reg
    td= (lcreg->resolution) <<13; ld= ld | td;
    td= (lcreg->faultTolQueue) <<11; ld= ld | td;
    if ( lcreg->alertpol) ld=ld |ALERTPOL_BM;
    if ( lcreg-> alarmMode) ld=ld |AlarmMode_BM; 
    if ( lcreg-> shutdownMode) ld=ld |SHUTDOWN_BM; 
    if ( lcreg->  RegisterLockDown) ld=ld |(REGLOCKDOWN_BM & RegisterLocDownEnable); 
    if ( lcreg->  RegisterLock) ld=ld |(REGLOCK_BM ); 
    if(  Nonvolatile )  ld=ld; else ld= 0xFF00 & ld;
    buffer[2]=ld & 0xFF; ld = ld>>8;
    buffer[1]=ld & 0xFF;    
    locerr=_i2c->write(Taddr,(char *)buffer,3,false);
    buffer[0]=  TReg;
    locerr= locerr<<2;
    locerr|=_i2c->write(Taddr,(char *)buffer,1,false); // make sure temp read reg is active again 
    error= locerr;
}
    
   
    

int AT30TSE75x::read_eeprompage(char *data, uint8_t length, uint8_t word_addr, uint8_t page) {  
    char buff[20];
    buff[0]  = (word_addr & 0x0F) | ((0x0F & page) << 4);
        
    _i2c->write(Eaddr,buff,1,false);
    return _i2c->read(Eaddr,data,length,false);  
}
 
int AT30TSE75x::write_eeprompage(char *data, uint8_t length, uint8_t word_addr, uint8_t page)
{   
    char buff[length+1];
    int buffsizep1=length+1;//buffersize+1
    buff[0] = (word_addr & 0x0F) | ((0x0F & page) << 4);
    for(int i = 1; i < buffsizep1; i++)  {
        buff[i] = *(data++);
    }
    return  _i2c->write(Eaddr,buff,(length + 1),false);
}


int AT30TSE75x::read_eeprombyte(char &data, uint8_t word_addr, uint8_t page){
   char rbuf[2];
   int i2cresult=read_eeprompage(rbuf,1,word_addr,page);
   data=rbuf[0];
   return i2cresult;
}   


int AT30TSE75x::write_eeprombyte(char data,  uint8_t word_addr, uint8_t page){
    char wbuf[2];  wbuf[0]=data;
    return write_eeprompage(wbuf,1,word_addr,page);
    }
    
int AT30TSE75x::protect_eeprom(void) {
       char wbuf[2];  wbuf[0]=0;wbuf[1]=0; // don't care 
     return   _i2c->write(AT30TSE75X_FIX_EEPROM ,wbuf,2,false);
}

int AT30TSE75x::unprotect_eeprom(void) {
       char wbuf[2];  wbuf[0]=0;wbuf[1]=0; // don't care 
      return _i2c->write((0x4 | AT30TSE75X_FIX_EEPROM) ,wbuf,2,false);
}

int AT30TSE75x::get_eeprom_protec(void){
    char wbuf[2]; 
    return _i2c->read(AT30TSE75X_FIX_EEPROM ,wbuf,1,false);
}
    