#ifndef STD_TYPES_H
#define STD_TYPES_H

#include "Platform_types.h"

#ifndef NULL
#define NULL	0
#endif

/* This type shall be used to request the version of a BSW module using the <Module name>_GetVersionInfo() function */
typedef struct {
   uint16 vendorID;
   uint16 moduleID;
   uint8 sw_major_version;
   uint8 sw_minor_version;
   uint8 sw_patch_version;
} Std_VersionInfoType;

//typedef unsigned int imask_t;

#ifndef MIN
#define MIN(_x,_y) (((_x) < (_y)) ? (_x) : (_y))
#endif
#ifndef MAX
#define MAX(_x,_y) (((_x) > (_y)) ? (_x) : (_y))
#endif

#define STD_GET_VERSION (_major,_minor,_patch) (_major * 10000 + _minor * 100 + _patch)

//#define E_OK				(Std_ReturnType) 0 /* conflict with trampoline */
//#define E_NOT_OK			(Std_ReturnType) 1 /* conflict with trampoline */
#define E_R_OK				(Std_ReturnType) 0
#define E_R_NOT_OK			(Std_ReturnType) 1
#define E_NO_DTC_AVAILABLE		(Std_ReturnType) 2
#define E_SESSION_NOT_ALLOWED		(Std_ReturnType) 4
#define E_PROTOCOL_NOT_ALLOWED	(Std_ReturnType) 5
#define E_REQUEST_NOT_ACCEPTED	(Std_ReturnType) 8
#define E_REQUEST_ENV_NOK		(Std_ReturnType) 9

#ifndef E_PENDING // for WIN32
#define E_PENDING			(Std_ReturnType) 10
#endif

#define E_COMPARE_KEY_FAILED		(Std_ReturnType) 11
#define E_FORCE_RCRRP			(Std_ReturnType) 12

//NOTE:
/* Create Std_VersionInfoType */
// PC-Lint Exception MISRA rule 19.12
//lint -save -esym(960,19.12)
#define STD_GET_VERSION_INFO(_vi,_module) \
	if(_vi != NULL) {\
		((_vi)->vendorID =  _module ## _VENDOR_ID);\
		((_vi)->moduleID = _module ## _MODULE_ID);\
		((_vi)->sw_major_version = _module ## _SW_MAJOR_VERSION);\
		((_vi)->sw_minor_version =  _module ## _SW_MINOR_VERSION);\
		((_vi)->sw_patch_version =  _module ## _SW_PATCH_VERSION);\
		((_vi)->ar_major_version =  _module ## _AR_MAJOR_VERSION);\
		((_vi)->ar_minor_version =  _module ## _AR_MINOR_VERSION);\
		((_vi)->ar_patch_version =  _module ## _AR_PATCH_VERSION);\
	}
	
typedef uint8 Std_ReturnType;

/*
enum {		
   STD_HIGH = 0x01,
   STD_LOW = 0x00
} S_HIGH, S_LOW;

enum {
   STD_ACTIVE = 0x01,
   STD_IDLE = 0x00
} S_ACTIVE,S_IDLE;
*/

/*enum {					
   STD_ON = 0x01,
   STD_OFF = 0x00
} S_ON, S_OFF;*/

//#define STD_ON			0x01
//#define STD_OFF			0x00

//#define STD_ACTIVE			0x01
//#define STD_IDLE			0x00

//#define STD_HIGH			0x01
//#define STD_LOW			0x00

// #define STD_ACTIVE			0x01  
//#define STD_IDLE			0x00 

#define S_ON			0x01  
#define S_OFF			0x00 
/* duplicate */
#define STD_ON			0x01
#define STD_OFF		0x00
#endif
