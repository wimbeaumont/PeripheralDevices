#ifndef __DEVERRORREPORTER__
#define __DEVERRORREPORTER__

//#include "getVersion.h"

#define DEVERRORREPORTER_HDR_VER "2.00"


/*
 *  This is a the Error reporter it stores the last errors etc.  
 *  No getVersion support for the moment. 
 *  This file make part of the PeriperalDevice package see repository  
 *  https://github.com/wimbeaumont/PeripheralDevices
 *  For more info see 	the README.md in the top of repository 
 *
 *  ver  0:10  initial 
 *
 * (C) Wim Beaumont Universiteit Antwerpen 2019
 *
 * License see
 * https://github.com/wimbeaumont/PeripheralDevices/blob/master/LICENSE
*/ 


class DevErrorReporter {

protected :
     const int notsupportederrno = -1999;	
     bool   ack; // last ack status 
     bool   notsupported;
     int    comerr; // reported Deverr 
     bool   devinit; //  if the device is initialized 
     bool setnotsupported(void) { comerr=notsupportederrno; notsupported=true;return comerr;}

     DevErrorReporter(void){
	  ack=false;comerr=0; devinit=false; notsupported=false;	
     }	
public:

// status info 
virtual bool getLastAckStatus(void) { return ack; }
virtual bool getDeviceInitStatus(void) { return devinit; }
virtual int getLastComError(void) {return comerr;}
virtual bool getNotSupported(void) {return notsupported;}


};
#endif 