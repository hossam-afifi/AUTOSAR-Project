#include "tp.h"
#include "tpl_os.h"
#include "blink.h"

void rx_activate(void)
{
  /* scheduale shot alarm for canrx task */
//  SetRelAlarm(canrx_trigger, 0, 0);
}

void tx_activate(void)
{
  /* scheduale shot alarm for canrx task */
//  SetRelAlarm(canrx_trigger, 0, 0);
}

#define APP_Task_cantx_START_SEC_CODE
#include "tpl_memmap.h"
FUNC(int, OS_APPL_CODE) main(void)
{
  /* 
   * NVIC specific intialization 
   */
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable the CAN1 Tx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the CAN1 Rx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* 
   * simple board intialization 
   */
  initBoard();
  
  /*
   * Communication Stack intialization
   */
  /* Down Up initialization */
  const Comm_ConfigType *CommConfigPtr = &CommConfig;
  CanControllerIdType ControllerId = CommConfigPtr->CanConfigPtr->CanConfigSet->CanController->CanControllerId;
  Can_Init(CommConfigPtr->CanConfigPtr);
  Can_SetControllerMode(ControllerId, CAN_T_START);
  CanIf_Init(CommConfigPtr->CanifConfigPtr);
  CanIf_SetControllerMode(ControllerId, CANIF_CS_STARTED);
  CanTp_Init();
  
  /* 
   * Now start the OS 
   */
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

DeclareAlarm(blink_alarm);

TASK(cantx)
{
  ledToggle(RED);
  CanTp_MainFunction();
  TerminateTask();
}

#define APP_Task_cantx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_canrx_START_SEC_CODE
#include "tpl_memmap.h"

TASK(canrx)
{
  ledToggle(BLUE);
  CanTp_MainFunction();
  TerminateTask();
}

#define APP_Task_canrx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_cantx_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_cantx)
{
  DisableAllInterrupts();
  for (int i = 0; i < 10; i++)
     ledToggle(RED);
  EnableAllInterrupts();
}
#define APP_ISR_isr_cantx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_canrx_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_canrx)
{
  DisableAllInterrupts();
  for (int i = 0; i < 10; i++)
     ledToggle(BLUE);
  EnableAllInterrupts();
}
#define APP_ISR_isr_canrx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define OS_START_SEC_CODE
#include "tpl_memmap.h"
/*
 *  * This is necessary for ST libraries
 *   */
FUNC(void, OS_CODE) assert_failed(uint8_t* file, uint32_t line)
{
}

FUNC(void, OS_CODE) PreTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == cantx) {
    ledOn(ORANGE);
  } else {
    ledOn(ORANGE);
    ledOff(ORANGE);
    ledOn(ORANGE);
  }
}

FUNC(void, OS_CODE) PostTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == cantx) {
    ledOff(ORANGE);
  } else {
    ledOff(ORANGE);
    ledOn(ORANGE);
    ledOff(ORANGE);
  }
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

