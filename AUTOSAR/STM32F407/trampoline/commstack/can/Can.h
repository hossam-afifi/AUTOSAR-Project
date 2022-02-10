#ifndef CAN_H_
#define CAN_H_

//#include "Modules.h"
#include "std_type.h"
#include "Can_Cfg.h"
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "CanIf_Types.h"

#include "cpu.h"
//#include "CanIf_Cbk.h"

#define CAN_VENDOR_ID		VENDOR_ID_ARCCORE
#define CAN_MODULE_ID		MODULE_ID_CAN

#define CAN_AR_MAJOR_VERSION	3
#define CAN_AR_MINOR_VERSION	1
#define CAN_AR_PATCH_VERSION	5

#define CAN_SW_MAJOR_VERSION	1
#define CAN_SW_MINOR_VERSION	0
#define CAN_SW_PATCH_VERSION	0

/* Development Errors
 */
#define CAN_E_PARAM_POINTER		0x01
#define CAN_E_PARAM_HANDLE		0x02
#define CAN_E_PARAM_DLC		0x03
#define CAN_E_PARAM_CONTROLLER	0x04
#define CAN_E_UNINIT			0x05	//API Service used without initialization
#define CAN_E_TRANSITION		0x06	//Invalid transition for the current mode
#define CAN_E_DATALOST			0x07	//Received CAN message is lost
#define CAN_E_PARAM_BAUDRATE		0x08	//Parameter Baudrate has an invalid value
#define CAN_E_ICOM_CONFIG_INVALID	0x09	//Invalid ICOM Configuration Id	
#define CAN_E_INIT_FAILED		0x0A	//Invalid configuration set selection

/* Function definitions ID'S */

#define Can_Init_ID  				0X00
#define CAN_MAINFUNCTION_WRITE_ID		0X01
#define CAN_INITCONTROLLER_SERVICE_ID  	0x02
#define Can_SetControllerMode_ID		0x03
#define Can_DisableControllerInterrupts_ID	0x04
#define Can_EnableControllerInterrupts_ID	0x05
#define Can_Write_ID				0x06
#define Can_GetVersionInfo_ID			0x07
#define Can_MainFunction_Read_ID		0x08
#define Can_MainFunction_BusOff_ID		0x09
#define Can_MainFunction_Wakeup_ID		0x0a
#define Can_CheckWakeup_ID			0x0b
#define Can_MainFunction_Mode_ID		0x0c
#define Can_ChangeBaudrate_ID			0x0d
#define Can_CheckBaudrate_ID			0x0e
#define Can_SetBaudrate_ID			0x0f

/* TODO: convert to union */
#ifdef CANWIDTH_EXTENDED
typedef uint16 Can_HwHandleType;	//Extended
#else
typedef uint8 Can_HwHandleType;	//Standard
#endif

typedef struct {
   uint32 txSuccessCnt;
   uint32 rxSuccessCnt;
   uint32 txErrorCnt;
   uint32 rxErrorCnt;
   uint32 boffCnt;
   uint32 fifoOverflow;
   uint32 fifoWarning;
} Can_Arc_StatisticsType;

typedef uint32 Can_IdType;

typedef struct Can_PduType {
   PduIdType swPduHandle;	//private data for CanIf,just save and use for callback
   uint8 length;		//Length, max 8 bytes
   Can_IdType id;		//the CAN ID, 29 or 11-bit
   uint8* sdu;			//data ptr
} Can_PduType;

typedef struct {
   Can_IdType CanId;		//Standard/Extended CAN ID of CAN L-PDU
   Can_HwHandleType Hoh;	//ID of the corresponding Hardware Object Range
   uint8 ControllerId;   	//ControllerId provided by CanIf clearly identify the corresponding controller
} Can_HwType;

typedef enum {
   CAN_T_START,	//CAN controller transition value to request state STARTED
   CAN_T_STOP,		//CAN controller transition value to request state STOPPED
   CAN_T_SLEEP,	//CAN controller transition value to request state SLEEP
   CAN_T_WAKEUP	//CAN controller transition value to request state STOPPED from state SLEEP
} Can_StateTransitionType;

typedef enum {
   CAN_OK,	//success
   CAN_NOT_OK,	//error occurred or wakeup event occurred during sleep transition
   CAN_BUSY	//transmit request could not be processed because no transmit object was available
} Can_ReturnType;	

/* Error from  CAN controller */
typedef union Can_Arc_ErrorType {
   volatile uint32_t R;
   struct {
      volatile uint32_t:24;
      volatile uint32_t BIT1ERR:1;
      volatile uint32_t BIT0ERR:1;
      volatile uint32_t ACKERR:1;
      volatile uint32_t CRCERR:1;
      volatile uint32_t FRMERR:1;
      volatile uint32_t STFERR:1;
      volatile uint32_t TXWRN:1;
      volatile uint32_t RXWRN:1;
   } B;
} Can_Arc_ErrorType;
 
 
/* Each controller has 32 hth's, so the division of 32 will give
   the controller
 */

#define GET_CANCONTROLLER(a) (a / HTH_DIVIDER)

#if (CAN_VERSION_INFO_API == STD_ON)
#define Can_GetVersionInfo(_vi) STD_GET_VERSION_INFO(_vi,CAN)
#endif

/* Can APIs */
void Can_Init(const Can_ConfigType* Config);
void Can_DeInit(void);
void Can_EnableControllerInterrupts(uint8 Controller);
void Can_DisableControllerInterrupts(uint8 controller);
void Can_MainFunction_Read(void);
void Can_MainFunction_BusOff(void);
void Can_MainFunction_Wakeup(void);
Can_ReturnType Can_CheckWakeup(uint8 Controller);
void Can_MainFunction_Mode(void);
Can_ReturnType Can_SetControllerMode(uint8 controller, Can_StateTransitionType transition);
Std_ReturnType Can_ChangeBaudrate(uint8 Controller, uint16 Baudrate);
Std_ReturnType Can_SetBaudrate(uint8 Controller, uint16 BaudRateConfigID);
void Can_MainFunction_Write(void);
Can_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType *PduInfo);

/* Helpers */
void Can_InitController(uint8 controller, const Can_ControllerConfigType *config);

/* ISR wrappers */
void Can_RxIsr_wrap(int unit, int fifo);
void Can_TxIsr_wrap(int unit);
void Can_ErrIsr_wrap(int unit);
#endif	

