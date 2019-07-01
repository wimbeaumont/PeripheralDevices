


class I2CInterface;
//class DigitalOut;
//class InterruptIn;

class MPC4728_address_set{
    I2CInterface *i2cd;
    DigitalOut *LDAC;
    DigitalOut *Cntout;
    InterruptIn cntin;
    int sclcnt;
    unsigned char oldaddress;
 void count_down( );
 int ldac1;
 int ldac0;
 
public:
 MPC4728_address_set(PinName sclcntpin, DigitalOut *LDACpin, I2CInterface* i2cdevice, DigitalOut *Cntoutpin , bool ldac_invert=false  );
 int getsclcnt( ){return sclcnt;}
  int readaddress(char& address, char& eepromaddr , char& regaddr );

  int setaddress(char currentaddress, char newaddress );

  void start_scl_cnt(int cnts) ;

};//endclass 
