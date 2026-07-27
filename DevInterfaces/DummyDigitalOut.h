#ifndef __DUMMYDIGITALOUT_H
#define __DUMMYDIGITALOUT_H

#include "DigitalOutInterface.h"

class DummyDigitalOut :public  DigitalOutInterface {
	
	DummyDigitalOut(int pinnr):DigitalOutInterface(pinnr) {};

};

#endif
