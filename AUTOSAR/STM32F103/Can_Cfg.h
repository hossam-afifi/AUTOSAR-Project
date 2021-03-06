#ifndef CAN_CFG_H_
#define CAN_CFG_H_

#include "std_type.h"
/* Number of controller configs */
#define CAN_ARC_CTRL_CONFIG_CNT 		1

#define CAN_VERSION_INFO_API 			S_OFF
#define CAN_TIMEOUT_DURATION 			10 /* ms */

#define can_hwhandlingtyperang_Extended     	0xFFFF
#define can_hwhandlingtyperang_Standard	0x0FF

#define INDEX_OF_CAN_CTRL_0 			0
#define INDEX_OF_CAN_CTRL_1 			1
#define INDEX_OF_CAN_CTRL_2 			2
#define INDEX_OF_CAN_CTRL_3 			3
#define INDEX_OF_CAN_CTRL_4 			4

typedef enum {
   CAN_ID_TYPE_EXTENDED,
   CAN_ID_TYPE_MIXED,
   CAN_ID_TYPE_STANDARD
} Can_IdTypeType;

typedef enum {
   CAN_CTRL_0_HTH,
   CAN_CTRL_1_HTH,
   CAN_CTRL_2_HTH,
   CAN_CTRL_3_HTH,
   CAN_CTRL_4_HTH,
   NUM_OF_HTHS 
} Can_Arc_HTHType;

typedef enum {
   CAN_CTRL_0_HRH,
   CAN_CTRL_1_HRH,
   CAN_CTRL_2_HRH,
   CAN_CTRL_3_HRH,
   CAN_CTRL_4_HRH,
   NUM_OF_HRHS
} Can_Arc_HRHType;


typedef enum {
   CAN_ARC_PROCESS_TYPE_INTERRUPT,
   CAN_ARC_PROCESS_TYPE_POLLING
} Can_Arc_ProcessType;

typedef enum {
   CAN_OBJECT_TYPE_RECEIVE,
   CAN_OBJECT_TYPE_TRANSMIT
} Can_ObjectTypeType;

typedef enum {
   CAN_ARC_HANDLE_TYPE_BASIC,
   CAN_ARC_HANDLE_TYPE_FULL
} Can_Arc_HohType;

typedef enum {
	CAN_CTRL_1 = 0,
	CAN_CTRL_2 = 1,
	CAN_CONTROLLER_CNT = 2
} CanControllerIdType;

typedef struct Can_PduType Can_PduType;
typedef uint32 Can_IdType;
typedef uint32 PduIdType;	/* TODO: check proper type */
typedef union Can_Arc_ErrorType Can_Arc_ErrorType;

/* CANIF */
typedef struct {
   void (*CancelTxConfirmation)( const Can_PduType *);
   void (*RxIndication)(uint8, Can_IdType, uint8, const uint8 *);
   void (*ControllerBusOff)(uint8);
   void (*TxConfirmation)(PduIdType);
   void (*ControllerWakeup)(uint8);
   void (*Arc_Error)(uint8, Can_Arc_ErrorType);
} Can_CallbackType;

typedef enum {
   CAN_IDAM_2_32BIT = 0,
   CAN_IDAM_4_16BIT = 1,
   CAN_IDAM_8_8BIT  = 2,
   CAN_IDAM_FILTER_CLOSED = 3,
} Can_IDAMType /* XXX: may be uint32 instead */;

typedef struct {
   uint8 idmr[8];        /* Identifier Mask Register, 1 = ignore corresponding acceptance code register bit*/
   uint8 idar[8];        /* Identifier Acceptance Register */
   Can_IDAMType idam;
} Can_FilterMaskType;

typedef struct S_Can_HwOb {
   /* Specifies the type (Full-CAN or Basic-CAN) of a hardware object.*/
   Can_Arc_HohType CanHandleType;
   
   /* Specifies whether the IdValue is of type - standard identifier - extended 
      identifier - mixed mode ImplementationType: Can_IdType
    */
   Can_IdTypeType CanIdType;

   /* Specifies (together with the filter mask) the identifiers range that passes
      the hardware filter
    */
   uint32 CanIdValue;

   /* Holds the handle ID of HRH or HTH. The value of this parameter is unique
      in a given CAN Driver, and it should start with 0 and continue without any
      gaps. The HRH and HTH Ids are defined under two different name-spaces.
      Example: HRH0-0, HRH1-1, HTH0-2, HTH1-3
    */
   uint16 CanObjectId;

   /* Specifies if the HardwareObject is used as Transmit or as Receive object */
   Can_ObjectTypeType CanObjectType;

   /* Reference to the filter mask that is used for hardware filtering togerther with the CAN_ID_VALUE */
   const Can_FilterMaskType *CanFilterMaskRef;

   /* A "1" in this mask tells the driver that that HW Message Box should be
      occupied by this Hoh. A "1" in bit 31(ppc) occupies Mb 0 in HW
    */
   uint32 Can_MbMask;

   /*boolean*/bool Can_Arc_EOL;	// End Of List. Set to TRUE is this is the last object in the list
} Can_HwObType;


typedef struct {
   /* Enables / disables API Can_MainFunction_BusOff() for
      handling busoff events in polling mode 
    */
   //Can_ProcessType CanBusoffProcessing;
      
   /* Defines if a CAN controller is used in the configuration */
   ///*boolean*/bool    CanControllerActivation;
	
   /* This parameter provides the controller ID which is unique in a
	given CAN Driver. The value for this parameter starts with 0 and
	continue without any gaps
    */
   CanControllerIdType  CanControllerId;
	
   /* Enables / disables API Can_MainFunction_Read() for
      handling PDU reception events in polling mode
    */
   Can_Arc_ProcessType CanRxProcessing;
	
   /* Enables / disables API Can_MainFunction_Write() for
      handling PDU transmission events in polling mode
    */
   Can_Arc_ProcessType CanTxProcessing;
	
   /* Enables / disables API Can_MainFunction_Wakeup() for
      handling wakeup events in polling mode
    */
   Can_Arc_ProcessType CanWakeupProcessing;
   Can_Arc_ProcessType CanBusOffProcessing;
	
   /* CAN driver support for wakeup over CAN Bus */
   ///*boolean*/bool   CanWakeupSupport;
	
   /* Reference to the CPU clock configuration, which is set in the MCU driver
      configuration
    */
   //uint32 CanCpuClockRef;
	
   /* This parameter contains a reference to the Wakeup Source for this
      ontroller as defined in the ECU State Manager. Implementation Type:
      reference to EcuM_WakeupSourceType
    */
   //uint32/* ref to EcuMWakeupSource */ CanWakeupSourceRef;
   
   /* Specifies the baudrate of the controller in kbps */
   uint16   CanControllerBaudRate;
	
   /* Specifies propagation delay in time quantas(1..8) */
   uint16   CanControllerPropSeg;
	
   /* Specifies phase segment 1 in time quantas(1..16) */
   uint16   CanControllerSeg1;

   /* Specifies phase segment 2 in time quantas(1..8) */
   uint16   CanControllerSeg2;
	
   /* Specifies the synchronization jump width(1..4) for the controller in
      time quantas
    */
   //uint16          CanControllerSyncJumpWidth;
	
   /* List of Hoh id's that belong to this controller */
   const Can_HwObType  *Can_Arc_Hoh;
	
   /*boolean*/bool Can_Arc_Loopback;	
} Can_ControllerConfigType;

typedef struct {
   const Can_ControllerConfigType *CanController;
   // Callbacks( Extension )
   const Can_CallbackType *CanCallbacks;	
} Can_ConfigSetType;

typedef struct
{
   /* This is the multiple configuration set container for CAN Driver
      Multiplicity 1..*
    */
   const Can_ConfigSetType *CanConfigSet;
} Can_ConfigType;

extern const Can_ConfigType Can_ConfigData;
#endif 				/*CAN_CFG_H_*/
