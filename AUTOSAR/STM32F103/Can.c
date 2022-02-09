
#include "Can.h"
//#include "stm32f407.h"
//#include "stm32f407_can.h"
//#include "CanIf_Cbk.h"
//#include "std_type.h"
#include <stdlib.h>
#include <string.h>


enum {		
   STD_HIGH = 0x01,
   STD_LOW = 0x00
} S_HIGH, S_LOW;

enum {
   STD_ACTIVE = 0x01,
   STD_IDLE = 0x00
} S_ACTIVE, S_IDLE;

typedef CAN_TypeDef CAN_HW_t;

typedef uint32 imask_t;

#define CAN_STATISTICS  S_ON    

#define GET_CONTROLLER_CONFIG(_controller)	&(can_global.config->CanConfigSet->CanController[(_controller)])

#define GET_PRIVATE_DATA(_controller)    	&CanUnit [_controller]

#define GET_CONTROLLER_CNT() 			(CAN_CONTROLLER_CNT)

#define GET_CALLBACKS() 			(can_global.config->CanConfigSet->CanCallbacks)

#if ( CAN_DEV_ERROR_DETECT == S_ON )
#define VALIDATE(_exp,_api,_err ) \
        if( !(_exp) ) { \
          Det_ReportError(MODULE_ID_CAN,0,_api,_err); \
          return CAN_NOT_OK; \
        }

#define VALIDATE_NO_RV(_exp,_api,_err ) \
        if( !(_exp) ) { \
          Det_ReportError(MODULE_ID_CAN,0,_api,_err); \
          return; \
        }

#define DET_REPORTERROR(_x,_y,_z,_q) Det_ReportError(_x, _y, _z, _q)
#else
#define VALIDATE(_exp,_api,_err )
#define VALIDATE_NO_RV(_exp,_api,_err )
#define DET_REPORTERROR(_x,_y,_z,_q)
#endif



typedef enum {
   CAN_UNINIT,
   CAN_READY
} Can_DriverStateType;

/* Mapping between HRH and Controller HOH */
typedef struct Can_Arc_ObjectHOHMapStruct {
   CanControllerIdType CanControllerRef;	// Reference to controller
   const Can_HwObType* CanHOHRef;	// Reference to HOH.
} Can_Arc_ObjectHOHMapType;

typedef struct {
   Can_DriverStateType initRun;

   /* Our config */
   const Can_ConfigType *config;

   /* One bit for each channel that is configured.
      Used to determine if validity of a channel
      1 - configured
      0 - NOT configured
    */
   uint32 configured;
   
   /* Maps the a channel id to a configured channel id */
   uint8  channelMap[CAN_CONTROLLER_CNT];

   /* This is a map that maps the HTH:s with the controller and Hoh. It is built
      during Can_Init and is used to make things faster during a transmit.
    */
   Can_Arc_ObjectHOHMapType CanHTHMap[NUM_OF_HTHS];
} Can_globalType;

Can_globalType can_global = {
   .initRun = CAN_UNINIT,
};

typedef struct {
   CanIf_ControllerModeType state;   //#include CANIF_Types.h
   uint8 lock_cnt;
#if (CAN_STATISTICS == S_ON)
   Can_Arc_StatisticsType stats;
#endif
//(PduIdType swPduHandle)autosar's document  [private data for CanIf,just save and use for callback]
   PduIdType swPduHandle;
} Can_UnitType;

Can_UnitType CanUnit[CAN_CONTROLLER_CNT] =
{
   {
    .state = CANIF_CS_UNINIT,  //#include CANIF_Types.h
   },
   {
    .state = CANIF_CS_UNINIT,  //#include CANIF_Types.h
   },
};


/*
static CAN_HW_t * GetController(int unit)
{
	CAN_HW_t *res = 0;

	if(unit == CAN_CTRL_0)
	{
		res = (CAN_HW_t *)&Can_HwUnit[unit];
	}
	else if(unit == CAN_CTRL_1)
	{
		res = (CAN_HW_t *)&Can_HwUnit[unit];
	}
	else if(unit == CAN_CTRL_2)
	{
		res = (CAN_HW_t *)&Can_HwUnit[unit];
	}
	else if(unit == CAN_CTRL_3)
	{
		res = (CAN_HW_t *)&Can_HwUnit[unit];
	}
	else if(unit == CAN_CTRL_4)
	{
		res = (CAN_HW_t *)&Can_HwUnit[unit];
	}

	return res;
}
*/

static CAN_HW_t* GetController(int unit)
{
   return ((CAN_HW_t *)(CAN1_BASE + unit*0x400));
}

static const Can_HwObType* Can_FindHoh(Can_HwHandleType Hth, uint32* controller)
{
   const Can_HwObType *hoh;
   const Can_Arc_ObjectHOHMapType *map;
   const Can_ControllerConfigType *can_hwconfig;

   map = &can_global.CanHTHMap[Hth];

   /* Verify that this is the correct map */
   if (map->CanHOHRef->CanObjectId != Hth)
   {
      DET_REPORTERROR(MODULE_ID_CAN, 0, 0x6, CAN_E_PARAM_HANDLE);
   }

   can_hwconfig= GET_CONTROLLER_CONFIG(can_global.channelMap[map->CanControllerRef]);

   hoh = map->CanHOHRef;

   /* Verify that this is the correct Hoh type */
   if (hoh->CanObjectType == CAN_OBJECT_TYPE_TRANSMIT)
   {
      *controller = map->CanControllerRef;
      return hoh;
   }

   DET_REPORTERROR(MODULE_ID_CAN, 0, 0x6, CAN_E_PARAM_HANDLE);

   return NULL;
}

static void Can_RxIsr(int unit, int fifo);
static void Can_TxIsr(int unit);
static void Can_ErrIsr(int unit);

void Can_RxIsr_wrap(int unit, int fifo)
{
  Can_RxIsr(unit, fifo);
}
void Can_TxIsr_wrap(int unit)
{
  Can_TxIsr(unit);
}
void Can_ErrIsr_wrap(int unit)
{
  //Can_ErrIsr(unit);
}


/* configration function can_init */
void Can_Init(const Can_ConfigType *config) {
   Can_UnitType *can_unit;
   const Can_ControllerConfigType *can_hwconfig;
   uint32 control_id;		//ctlrId

   VALIDATE_NO_RV( (can_global.initRun == CAN_UNINIT), 0x0, CAN_E_TRANSITION ); 	//(autosar) CAN_E_TRANSITION =  0x06
   VALIDATE_NO_RV( (config != NULL ), 0x0, CAN_E_PARAM_POINTER ); 			//(autosar) CAN_E_PARAM_POINTER = 0x01

   // Save config
   can_global.config = config;
   can_global.initRun = CAN_READY;

   for (int canconfigid=0;canconfigid < CAN_ARC_CTRL_CONFIG_CNT;canconfigid++) {
      can_hwconfig = GET_CONTROLLER_CONFIG(canconfigid);
      control_id = can_hwconfig->CanControllerId;
      can_global.channelMap[can_hwconfig->CanControllerId] = canconfigid;
      can_global.configured |= (1 << control_id);
      can_unit = GET_PRIVATE_DATA( control_id) ;
      can_unit->state = CANIF_CS_STOPPED;  //CANIF_CS_STOPPED = 1
      can_unit->lock_cnt=0;
      memset(&can_unit->state,0,sizeof(Can_Arc_StatisticsType));  //clear all parmenter in Can_Arc_StatisticsType
      Can_InitController(control_id, can_hwconfig);
	
      const Can_HwObType* hoh;
      
      hoh = can_hwconfig->Can_Arc_Hoh;
      hoh--;
      
      do {
         hoh++;
	 if (hoh->CanObjectType == CAN_OBJECT_TYPE_TRANSMIT) {
	   can_global.CanHTHMap[hoh->CanObjectId].CanHOHRef = hoh;
	   can_global.CanHTHMap[hoh->CanObjectId].CanControllerRef = can_hwconfig->CanControllerId;
	 }
      } while (!hoh->Can_Arc_EOL);
    }
}

void Can_DeInit()
{
   Can_UnitType *canUnit;
   const Can_ControllerConfigType *canHwConfig;
   uint32 ctlrId;

   for (int configId=0; configId < CAN_ARC_CTRL_CONFIG_CNT; configId++) {	  
      canHwConfig = GET_CONTROLLER_CONFIG(configId);
      ctlrId = canHwConfig->CanControllerId;
      canUnit = GET_PRIVATE_DATA(ctlrId);
      canUnit->state = CANIF_CS_UNINIT;
      Can_DisableControllerInterrupts(ctlrId);
      canUnit->lock_cnt = 0;
      /* Clear stats */
      memset(&canUnit->stats, 0, sizeof(Can_Arc_StatisticsType));
   }
   can_global.config = NULL;
   can_global.initRun = CAN_UNINIT;
   return;
}

void Can_InitController(uint8 controller, const Can_ControllerConfigType *config)
{
   CAN_HW_t *canhw;
   uint8_t tq;
   uint8_t tqSync;
   uint8_t tq1;
   uint8_t tq2;
   uint32_t clock;   
   Can_UnitType *canUnit;
   //uint8 cId = controller;
   const Can_ControllerConfigType *canHwConfig;
   const Can_HwObType *hoh;

   VALIDATE_NO_RV( (can_global.initRun == CAN_READY), 0x2, CAN_E_UNINIT );
   VALIDATE_NO_RV( (config != NULL ), 0x2,CAN_E_PARAM_POINTER);
   VALIDATE_NO_RV( (controller < GET_CONTROLLER_CNT()), 0x2, CAN_E_PARAM_CONTROLLER );
   
   canUnit = GET_PRIVATE_DATA(controller);
   VALIDATE_NO_RV( (canUnit->state==CANIF_CS_STOPPED), 0x2, CAN_E_TRANSITION );

   canhw = GetController(controller);
   canHwConfig = GET_CONTROLLER_CONFIG(can_global.channelMap[controller]);

   /* De-init first */
   CAN_DeInit(canhw);

  /* CAN filter init. We set up two filters - one for the master (CAN1) and
   * one for the slave (CAN2)
   *
   * CAN_SlaveStartBank(n) denotes which filter is the first of the slave.
   *
   * The filter registers reside in CAN1 and is shared to CAN2, so we only need
   * to set up this once.
   */

   // We let all frames in and do the filtering in software.
   CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   CAN_FilterInitStructure.CAN_FilterMode      =  CAN_FilterMode_IdMask;
   CAN_FilterInitStructure.CAN_FilterScale      = CAN_FilterScale_32bit;
   CAN_FilterInitStructure.CAN_FilterIdHigh     = 0x0000;  
   CAN_FilterInitStructure.CAN_FilterIdLow      = 0x0000;  
   CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;  
   CAN_FilterInitStructure.CAN_FilterMaskIdLow  = 0x0000;  
   CAN_FilterInitStructure.CAN_FilterFIFOAssignment  =  CAN_FIFO0;  
   CAN_FilterInitStructure.CAN_FilterActivation      =  ENABLE;
   /* Init filter 0 (CAN1/master) */
   CAN_FilterInitStructure.CAN_FilterNumber  = 0;  
   CAN_FilterInit(&CAN_FilterInitStructure);
   /* Init filter 1 (CAN2/slave) */
   CAN_FilterInitStructure.CAN_FilterNumber  =  1;  
   CAN_FilterInit(&CAN_FilterInitStructure);
   /* Set which filter to use for CAN2 */
   CAN_SlaveStartBank(1);

   /* acceptance filters */
   hoh = canHwConfig->Can_Arc_Hoh;
   --hoh;
   do {
      ++hoh;
      if (hoh->CanObjectType == CAN_OBJECT_TYPE_RECEIVE)
      {
    	 /* TODO: Hw filtering */
      }
   } while(!hoh->Can_Arc_EOL);

   /* Clock calucation
      -------------------------------------------------------------------
      1 TQ = Sclk period( also called SCK )
      Ftq = Fcanclk / ( PRESDIV + 1 ) = Sclk
      ( Fcanclk can come from crystal or from the peripheral dividers )
      -->
      TQ = 1/Ftq = (PRESDIV+1)/Fcanclk --> PRESDIV = (TQ * Fcanclk - 1 )
      TQ is between 8 and 25
    */
   //clock = McuE_GetSystemClock()/2;  // clock -- #include time.h
   clock = SystemCoreClock/2;
   tqSync = config->CanControllerPropSeg + 1;
   tq1    = config->CanControllerSeg1 + 1;  
   tq2    = config->CanControllerSeg2 + 1;  
   tq     = tqSync + tq1 + tq2;
   
   CAN_InitTypeDef CAN_InitStructure;  
   CAN_StructInit(&CAN_InitStructure);
   /* CAN cell init */
   CAN_InitStructure.CAN_TTCM  = DISABLE;
   CAN_InitStructure.CAN_ABOM  = ENABLE;
   CAN_InitStructure.CAN_AWUM  = ENABLE;
   CAN_InitStructure.CAN_NART  = DISABLE;
   CAN_InitStructure.CAN_RFLM  = DISABLE;
   CAN_InitStructure.CAN_TXFP  = DISABLE;
   
   if (config->Can_Arc_Loopback) {
      CAN_InitStructure.CAN_Mode=CAN_Mode_Silent_LoopBack/*CAN_Mode_LoopBack*/;  
   } else {
      CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
   }

   CAN_InitStructure.CAN_SJW = config->CanControllerPropSeg;
   CAN_InitStructure.CAN_BS1 = config->CanControllerSeg1;
   CAN_InitStructure.CAN_BS2 = config->CanControllerSeg2;
   CAN_InitStructure.CAN_Prescaler = clock/(config->CanControllerBaudRate*1000*tq);

   if (CANINITOK != CAN_Init(canhw,&CAN_InitStructure)) {
      return;
   }
   
   canUnit->state = CANIF_CS_STOPPED;
   Can_EnableControllerInterrupts(controller);
   
   /* Disable Debug Freeze */
   CAN_DBGFreeze(canhw, DISABLE);
   return;
}


#define INSTALL_HANDLERS(_can_name,_sce,_rx,_tx) \
  do { \
    ISR_INSTALL_ISR2( "Can_Err", _can_name ## _ErrIsr, _sce, 2, 0 ); \
	ISR_INSTALL_ISR2( "Can_Rx",  _can_name ## _RxIsr,  _rx,  2, 0 ); \
	ISR_INSTALL_ISR2( "Can_Tx",  _can_name ## _TxIsr,  _tx,  2, 0 ); \
  } while(0);


static void Can_RxIsr(int unit, int fifo) {
   const Can_HwObType *hoh;
   CanRxMsg RxMessage;
   CAN_HW_t *canhw = GetController(unit);	
   const Can_ControllerConfigType *canHwConfig = GET_CONTROLLER_CONFIG(can_global.channelMap[unit]);	
   Can_UnitType *canUnit = GET_PRIVATE_DATA(unit);
   
   RxMessage.StdId = 0x00;
   RxMessage.ExtId = 0X00;
   RxMessage.DLC = 0;
   RxMessage.IDE = 0;
   RxMessage.FMI = 0;
   RxMessage.Data[0] = 0X00;
   RxMessage.Data[1] = 0X00;
   
   CAN_Receive(canhw, fifo, &RxMessage);  //THIS FUNCTION DEFINE IN STMCAN.
	
   hoh = canHwConfig->Can_Arc_Hoh;
   --hoh;
   do {
      ++hoh;
      if (hoh->CanObjectType == CAN_OBJECT_TYPE_RECEIVE) {
         Can_IdType id =0;
         		
	 if (RxMessage.StdId != CAN_ID_STD) {
	    id = RxMessage.ExtId;
	    id |=0x80000000;
	 } else {
	    id = RxMessage.StdId;
	 }
	 
	 if (GET_CALLBACKS()->RxIndication != NULL) {   //RxIndication config in struct Can_CallbackType
	    GET_CALLBACKS()->RxIndication(hoh->CanObjectId, id, RxMessage.DLC, (uint8 *)&RxMessage.Data[0]);
	 }
	 
	 canUnit->stats.rxSuccessCnt++;
      }
   } while (!hoh->Can_Arc_EOL);
}

static void Can_TxIsr(int controller) {
   const Can_HwObType* hoh;
   CAN_HW_t *can_hw = GetController(controller);
   const Can_ControllerConfigType *can_hwconfig = GET_CONTROLLER_CONFIG(can_global.channelMap[controller]);
   Can_UnitType *can_unit = GET_PRIVATE_DATA(controller);

   hoh = can_hwconfig->Can_Arc_Hoh;	
   --hoh;	
   do {
      ++hoh;
      if (hoh->CanObjectType == CAN_OBJECT_TYPE_TRANSMIT) {
      
         if (GET_CALLBACKS()->TxConfirmation != NULL) {			
	     GET_CALLBACKS()->TxConfirmation(can_unit->swPduHandle);
	 }
	 
         can_unit->swPduHandle = 0;			
         CAN_ClearITPendingBit(can_hw, CAN_IT_RQCP0);
         CAN_ClearITPendingBit(can_hw, CAN_IT_RQCP1);
         CAN_ClearITPendingBit(can_hw, CAN_IT_RQCP2);
      }
   } while (!hoh->Can_Arc_EOL);
}

Can_ReturnType Can_Write(Can_HwHandleType Hth, const Can_PduType* PduInfo) {
   Can_UnitType *can_unit;
   CAN_HW_t *can_hw;
   const Can_ControllerConfigType *can_hwconfig;
   const Can_HwObType* hoh;	
   uint32 can_controller;
   Can_ReturnType RV = CAN_OK;   //checks if the hardware transmit object
   
   VALIDATE((can_global.initRun == CAN_READY), 0x6, CAN_E_UNINIT);
   VALIDATE((PduInfo != NULL), 0x6, CAN_E_PARAM_POINTER);
   VALIDATE((PduInfo->length <= 8), 0x6, CAN_E_PARAM_DLC);
   VALIDATE((Hth < can_hwhandlingtyperang_Extended), 0x6, CAN_E_PARAM_HANDLE);
  
   imask_t state;
   hoh = Can_FindHoh(Hth, &can_controller);
   if (hoh == NULL)
      return CAN_NOT_OK;
		
   can_unit = GET_PRIVATE_DATA(can_controller);	
   can_hw = GetController(can_controller);
   
   Irq_Save(state);
   
   CanTxMsg TxMessage ;	
   TxMessage.RTR = CAN_RTR_DATA;	
   TxMessage.DLC = PduInfo->length;	
   memcpy(TxMessage.Data, PduInfo->sdu, PduInfo->length);
   	
   if (hoh->CanIdType == CAN_ID_TYPE_EXTENDED) {	
      TxMessage.IDE = CAN_ID_EXT;	
      TxMessage.ExtId = PduInfo->id;
   } else {
      TxMessage.IDE = CAN_ID_STD;
      TxMessage.StdId = PduInfo->id;
   }

   if (CAN_Transmit(can_hw , &TxMessage) != CAN_NO_MB) {
      can_hwconfig = GET_CONTROLLER_CONFIG(can_global.channelMap[can_controller]);
      
      if (can_hwconfig->CanTxProcessing == CAN_ARC_PROCESS_TYPE_INTERRUPT ) {
         CAN_ITConfig(can_hw,CAN_IT_TME,ENABLE);
      }
	
      can_unit->stats.txSuccessCnt++;
      can_unit->swPduHandle = PduInfo->swPduHandle;
   } else {
      RV = CAN_BUSY;
   }

   Irq_Restore(state);
   return RV ;
}

Can_ReturnType Can_SetControllerMode(uint8 controller, Can_StateTransitionType transition) {
   uint32 state ;     // imask_t state;	 
   CAN_HW_t *canhw;	 
   Can_ReturnType RV = CAN_OK;

   VALIDATE( (controller < GET_CONTROLLER_CNT()), 0x03, CAN_E_PARAM_CONTROLLER ); 	// check for can_ctrl RETURN NOT_OK	 
   Can_UnitType *canUnit = GET_PRIVATE_DATA(controller);	 
   VALIDATE( (canUnit->state != CANIF_CS_UNINIT), 0x03, CAN_E_UNINIT ); 		//check canif is not can_uninit
	 
   canhw =GetController(controller);	 
   switch (transition) {
      case CAN_T_START:
         canUnit->state = CANIF_CS_STARTED;
         
	 Irq_Save(state);
	 
	 if (canUnit->lock_cnt == 0) 
	    Can_EnableControllerInterrupts(controller);

         Irq_Restore(state);
      break;
      case CAN_T_STOP:
         VALIDATE(canUnit->state == CANIF_CS_STOPPED, 0x03, CAN_E_TRANSITION);
         CAN_Sleep(canhw);	// function determined canx in sleep mode or not (this function defination in stm32f407 can.c)
         canUnit->state = CANIF_CS_SLEEP;
      break;
      case CAN_T_WAKEUP:
         VALIDATE(canUnit->state == CANIF_CS_SLEEP, 0x03, CAN_E_TRANSITION);
         CAN_WakeUp(canhw);
         canUnit->state = CANIF_CS_STOPPED;
         break;
   }
   return RV;
}

void Can_DisableControllerInterrupts(uint8 controller) {
   uint32 state ;     // imask_t state;
   CAN_HW_t *canhw;
   Can_UnitType *canUnit = GET_PRIVATE_DATA(controller);
   
   VALIDATE_NO_RV((controller < GET_CONTROLLER_CNT()), 0x04, CAN_E_PARAM_CONTROLLER);
   VALIDATE_NO_RV((canUnit->state != CANIF_CS_UNINIT), 0x04, CAN_E_UNINIT);
   
   Irq_Save(state);
   
   if (canUnit->lock_cnt > 0)  {
      canUnit->lock_cnt++;
      
      Irq_Restore(state);
      
      return;
   }
   
   canUnit->lock_cnt++;
	
   Irq_Restore(state);
   canhw = GetController(controller);
   /* turn off transmit interrupt mailboix */
   CAN_ITConfig(canhw,CAN_IT_TME,DISABLE);	//CAN_IT_TME config in stm32f407 can.h
   /* turn off err interrupt, bus off, wake up and fifo message */
   CAN_ITConfig(canhw, CAN_IT_FMP0 | CAN_IT_BOF | CAN_IT_ERR | CAN_IT_WKU, DISABLE);	// DISABLE=0 config in stm32f40xx.h
}

void Can_EnableControllerInterrupts(uint8 controller) {
   imask_t state;
   Can_UnitType *canUnit;
   CAN_HW_t *canHw;
   const Can_ControllerConfigType *canHwConfig;

   VALIDATE_NO_RV((controller < GET_CONTROLLER_CNT()), 0x5, CAN_E_PARAM_CONTROLLER);
   canUnit = GET_PRIVATE_DATA(controller);
   VALIDATE_NO_RV( (canUnit->state != CANIF_CS_UNINIT), 0x5, CAN_E_UNINIT );

   Irq_Save(state);
  
   if (canUnit->lock_cnt > 1) {
      /* IRQ should still be disabled so just decrement counter */
       canUnit->lock_cnt--;

       Irq_Restore(state);

       return;
   } else if (canUnit->lock_cnt == 1) {
      canUnit->lock_cnt = 0;
   }

   Irq_Restore(state);
   canHw = GetController(controller);
   canHwConfig = GET_CONTROLLER_CONFIG(can_global.channelMap[controller]);

   if (canHwConfig->CanRxProcessing == CAN_ARC_PROCESS_TYPE_INTERRUPT) {
      /* Turn on the rx interrupt */
      CAN_ITConfig(canHw, CAN_IT_FMP0 /*| CAN_IT_FF0 | CAN_IT_FOV0*/ | CAN_IT_FMP1 /*| CAN_IT_FF1 | CAN_IT_FOV1*/, ENABLE);
   }

   if (canHwConfig->CanTxProcessing == CAN_ARC_PROCESS_TYPE_INTERRUPT) {
      /* Turn on the tx interrupt mailboxes */
      CAN_ITConfig(canHw, CAN_IT_TME, ENABLE);
   }

   /* BusOff here represents all errors and warnings */
   if (canHwConfig->CanBusOffProcessing == CAN_ARC_PROCESS_TYPE_INTERRUPT) {
      /* Turn on the bus off/tx warning/rx warning and error and rx  */
      CAN_ITConfig(canHw, CAN_IT_BOF | CAN_IT_ERR | CAN_IT_WKU, ENABLE);
   }
}

