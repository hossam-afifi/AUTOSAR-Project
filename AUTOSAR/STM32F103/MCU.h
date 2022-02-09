
#ifndef MCU_H_
#define MCU_H_

#include "Modules.h"

#define MCU_SW_MAJOR_VERSION	2
#define MCU_SW_MINOR_VERSION	0
#define MCU_SW_PATCH_VERSION	0

#define MCU_AR_MAJOR_VERSION	2
#define MCU_AR_MINOR_VERSION	2
#define MCU_AR_PATCH_VERSION	2

#include "Cpu.h"
#include "irq_types.h"
#include "Std_Types.h"
#include "Mcu_Cfg.h"

/* Function Definitions */					
/*
#define MCU_GETRESETREASON_SERVICE_ID		0x05
#define Mcu_Init_ID				0x00
#define MCU_GETRESETRAWVALUE_SERVICE_ID	0x06
#define Mcu_InitRamSection_ID			0x01
#define MCU_PERFORMRESET_SERVICE_ID		0x07
#define Mcu_InitClock_ID			0x02
define MCU_SETMODE_SERVICE_ID			0x08
#define Mcu_DistributePllClock_ID		0x03
#define MCU_GETVERSIONINFO_SERVICE_ID		0x09
#define Mcu_GetPllStatus_ID			0x04
#define MCU_INTCVECTORINSTALL_SERVICE_I	0x0a
#define Mcu_GetResetReason_ID			0x05					
*/

#define Mcu_GetResetRawValue_ID		0x06
#define Mcu_PerformReset_ID			0x07
#define Mcu_SetMode_ID				0x08
#define Mcu_GetVersionInfo_ID			0x09
#define Mcu_GetRamState_ID			0x0a										

/* error classification */
#define MCU_E_PARAM_CONFIG			0x0A
#define MCU_E_PARAM_CLOCK			0x0B
#define MCU_E_PARAM_MODE			0x0C
#define MCU_E_PARAM_RAMSECTION		0x0D
#define MCU_E_PLL_NOT_LOCKED			0x0E
#define MCU_E_UNINIT				0x0F
#define MCU_E_PARAM_POINTER			0x10
#define MCU_E_INIT_FAILED			0x11

/* Specific return values */
#define Mcu_GetResetRawValue_nonreset		0x00		/* MCU_GETRESETRAWVALUE_NORESETREG_RV */
#define Mcu_GetResetRawValue_reset		0xffffffff	/* MCU_GETRESETRAWVALUE_UNINIT_RV */

typedef enum { 
   MCU_PLL_LOCKED,
   MCU_PLL_STATUS_UNDEFINED,
   MCU_PLL_UNLOCKED 
} Mcu_PllStatusType;

typedef enum { 
   MCU_POWER_ON_RESET, 
   MCU_WATCHDOG_RESET, 
   MCU_SW_RESET, 
   MCU_RESET_UNDEFINED,
   MCU_CPU_RESET,
   MCU_EXT_RESET,
   MCU_VSW_RESET 
} Mcu_ResetType;
 
 
typedef struct { 
   /* This container defines a reference point in the Mcu Clock tree. 
      It defines the frequency which then can be used by other modules as an input value. 
      Lower multiplicity is 1, as even in the simplest case (only one frequency is used), 
      there is one frequency to be defined
    */	
   uint8 McuClockReferencePointFrequency;
   uint8 Pll1; 	// PLL setting 1
   uint8 Pll2; 	// PLL setting 2
   uint8 Pll3; 	// PLL setting 3
   uint8 Pll1_1; 	// PLL setting 1
   uint8 Pll2_1; 	// PLL setting 2
   uint8 Pll3_1; 	// PLL setting 3
   uint8 Pll4; 	// PLL setting 4
 
} Mcu_ClockSettingConfigType ;
 
typedef struct {
/* Enables/Disables clock failure notification,In case this feature is not supported by HW the setting should be disabled */
   uint8 McuClockSrcFailureNotification; 

   // This parameter shall represent the number of Modes available for the MCU.
   //calculationFormula = Number of configured McuModeSettingConf
   uint8 McuNumberOfMcuModes; 
	
   //This parameter shall represent the number of RAM sectors available for the MCU.
   //calculationFormula = Number of configured McuRamSectorSettingConf
   uint8 McuRamSectors;
	
   //This parameter relates to the MCU specific reset configuration.
   uint8 McuResetSetting;
	
   Mcu_ClockType McuDefaultClockSettings; 
 
   const Mcu_ClockSettingConfigType *McuClockSettingConfig;
 
   const Mcu_RamSectorSettingConfigType *McuRamSectorSettingConfig;
} Mcu_ConfigType;
#endif

