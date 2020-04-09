#ifndef __LINUXI2C_H  
#define __LINUXI2C_H  


/*

	This is just a wrapper to avoid name confilicts ( for read / write )   
	V 2.0   :  added methode to read register. 
    V 1.0   :  initial 
*/



#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c.h>		//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port



class LinuxI2C {

  int fdev;
  int devstat;

public :
  LinuxI2C(char* filedescr ){
    if ((fdev = open(filedescr, O_RDWR)) < 0) {
      devstat=-1;	
    }
    //printf("open %s with filedescr %d  \n\r", filedescr,fdev );
    devstat=0;	
  };
  
  ~LinuxI2C(void){ close(fdev);}
	
 
  
  int  setaddr(int addr) {
	
     addr=addr>>1;
     int result=ioctl(fdev, I2C_SLAVE, addr);
     //if (result < 0){
		//printf("Failed to acquire bus access and/or talk to slave.\n");
		//printf("fdev %d ,addr : %d , Errno : %d \n\r", fdev, addr, errno);
		//ERROR HANDLING; you can check errno to see what went wrong
		// exit(-2);
      //}
      return result;
    };
int i2c_reg_read( int address, char *result, int length   , int reg  ) {
	
	uint8_t slave_addr = (uint8_t) address;
	slave_addr= slave_addr >>1;
	if(fdev< 0) return -100;
    uint8_t outbuf[1];
    struct i2c_msg msgs[2];
    struct i2c_rdwr_ioctl_data msgset[1];
    msgs[0].addr = slave_addr;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = outbuf;

    msgs[1].addr = slave_addr;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = length;
	msgs[1].buf =(uint8_t*)  result;
    msgset[0].msgs = msgs;
    msgset[0].nmsgs = 2;

    outbuf[0] =(uint8_t) reg;

    if (ioctl(fdev, I2C_RDWR, &msgset) < 0) {
        perror("ioctl(I2C_RDWR) in i2c_read");
        return -1;
    }
    //*result = inbuf[0];
    return 0;
}


  int linux_read( char* data, int length) {
	return read(fdev, data, length); 
  };

  int linux_write( const char* data, int length) {
	return write(fdev, data, length); 
  };
 
  int getDevStatus(void) {return devstat;}
    
};



#endif