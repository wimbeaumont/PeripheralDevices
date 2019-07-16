#ifndef __LINUXI2C_H  
#define __LINUXI2C_H  


/*

	This is just a wrapper to avoid name confilicts ( for read / write ) 

*/



#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port


#include <cerrno> 
 

class LinuxI2C {

  int fdev;
  int devstat;

public :
  LinuxI2C(char* filedescr ){
    if ((fdev = open(filedescr, O_RDWR)) < 0) {
      /* ERROR HANDLING: you can check errno to see what went wrong */
      perror("Failed to open the i2c bus");
      devstat=-1;	
      exit(1);
    }
    printf("open %s with filedescr %d  \n\r", filedescr,fdev );
    devstat=0;	
  };
  
  ~LinuxI2C(void){ close(fdev);}
	
 
  
  int  setaddr(int addr) {
	
     addr=addr>>1;
     int result=ioctl(fdev, I2C_SLAVE, addr);
     if (result < 0){
		printf("Failed to acquire bus access and/or talk to slave.\n");
		printf("fdev %d ,addr : %d , Errno : %d \n\r", fdev, addr, errno);
		//ERROR HANDLING; you can check errno to see what went wrong
		// exit(-2);
      }
      return result;
    };

  int linux_read( char* data, int length) {
	return read(fdev, data, length); 
  };

  int linux_write( const char* data, int length) {
	return write(fdev, data, length); 
  };
 
  int getDevStatus(void) {return devstat;}
    
};



#endif