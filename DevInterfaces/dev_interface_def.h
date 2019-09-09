#ifndef device_interface_def_H
#define device_interface_def_H
#include <stdint.h>

#define DEV_INTERFACE_DEF_VER "0.4"

/* version history 
    v 0.1  
    v 0.2   20170111 added int16_t  and uint16_t   added DEV_INTERFACE_DEF_VER
    v 0.4   20170111 added int8_t
	v 0.5   20190909 added stdint.h , define the uxx  accordingly 
						removed uintx_t and uint8_t as these are definined in stdint.hardresume
*/

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#endif