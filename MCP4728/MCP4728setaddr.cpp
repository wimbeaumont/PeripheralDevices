
#include "mbed.h"

#include "MCP4728setaddr.h"
#include "I2CInterface.h" 
//#include "dev_interface_def.h"



#define MCP4728_baseaddr 0xC0
#define MCP4728_readaddresscmd 0xC
 
 
MPC4728_address_set::MPC4728_address_set(PinName sclcntpin, DigitalOut *LDACpin, I2CInterface* i2cdevice, DigitalOut *Cntoutpin , bool ldac_invert  ): cntin(sclcntpin) {
        i2cd=i2cdevice;
        LDAC=LDACpin;
        Cntout=Cntoutpin;
        *Cntoutpin=0;
        if (ldac_invert )  { ldac1 = 0; ldac0=1;}
        else  { ldac1 = 1; ldac0=0;}
        *LDAC=ldac1;
         i2cd->frequency(40000); // interrupt is slow, so set to 40 kHz 
        
 }        

void MPC4728_address_set::count_down( ){
        sclcnt--;
        *Cntout=1;
        *Cntout=0;
        if ( sclcnt==0){ 
            //disable this interrupt 
            int wait=10;  while ( wait--);
            *LDAC=ldac0;
        }
 }
 


int MPC4728_address_set::readaddress(char& address, char& eepromaddr , char& regaddr ) {
          *LDAC=ldac1;
        start_scl_cnt(18);// 1+9+8 faling edges 
        address=0x0C; // not allowed value
        char cmd = MCP4728_readaddresscmd;
        int i2cresult=i2cd->write(0, &cmd, 1,true ); //1 byte don't send stop  
        if  ( i2cresult ) {  // <> 0 
            //i2cd->abort_transfer  (  );
            i2cd->stop();
            address=0xFF;
            return i2cresult ;
        }
        i2cresult=i2cd->read(MCP4728_baseaddr+4, &cmd, 1,false ); // +4 because it could conflicts with the MCP4725 what is set to 0xC1
        if ( i2cresult) address=0xFE;
        else {
             address=cmd;
             regaddr= cmd ;
             regaddr = regaddr >>1;
             regaddr= regaddr & 0x7;
             eepromaddr = cmd;
             eepromaddr = eepromaddr >> 5;
             eepromaddr = eepromaddr & 0x7 ;
        }
        *LDAC=ldac1;
        return i2cresult ;
}    


  int MPC4728_address_set::setaddress(char currentaddress, char newaddress ){  // both address 
       *LDAC=ldac1;
        char oldaddr=0x7 & currentaddress;
        char newaddr=0x7 & newaddress; 
        newaddr= newaddr<<2;
        oldaddr=oldaddr<<1;
        char address=MCP4728_baseaddr | oldaddr;
        oldaddr=oldaddr<<1;
        const char cmdtype= 0x60;  // 0x011000000   0x011A AA01
        char  data[3] ;
        data[0]= cmdtype | oldaddr | 1; //0x011OOO01
        data[1]= cmdtype | newaddr | 2; //0x011NNN10
        data[2]= cmdtype | newaddr | 3; //0x011NNN11
        printf( "cmd %x %x %x data %x %x %x \n\r", (int)newaddress, (int)newaddr,(int) oldaddr,(int)data[0],(int)data[1],(int)data[2]);
         start_scl_cnt(18);//1+ 9+8 faling edges 
        int i2cresult=i2cd->write(address, data, 3,false ); //3 bytes sent and stop
        *LDAC=ldac1;
        return i2cresult ;
   }

  void MPC4728_address_set::start_scl_cnt(int cnts) {
    sclcnt= cnts; 
    cntin.fall(this, &MPC4728_address_set::count_down);    
  }    
