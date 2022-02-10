#include "tp.h"
#include "tpl_os.h"
#include "blink.h"

/*
 * Tx and Rx buffers
 */
#define OS_START_SEC_CODE
#include "tpl_memmap.h"

PduInfoType* txbufptr;
PduInfoType* rxbufptr;
int txlen;
int rxlen;

uint8 txbuf1[] = { 10, 20, 30, 40, 50, 60, 70, 80 };
uint8 txbuf2[] = { 90, 100, 110, 120, 130, 140, 150, 160 };
uint8 txbuf3[] = { 170, 180, 190, 200, 210, 220, 230, 240 };
uint8 txbuf4[] = { 250, 260, 270, 280, 290, 300, 310, 320 };

PduInfoType txbuf[] = { 
{
  .SduDataPtr = txbuf1,
  .SduLength  = /*8*/7
},
{
  .SduDataPtr = txbuf2,
  .SduLength  = /*8*/7
},
{
  .SduDataPtr = txbuf3,
  .SduLength  = /*8*/7
},
{
  .SduDataPtr = txbuf4,
  .SduLength  = /*8*/7
}
};

uint8 rxbuf1[8];
uint8 rxbuf2[8];
uint8 rxbuf3[8];
uint8 rxbuf4[8];

PduInfoType rxbuf[] = {
{
  .SduDataPtr = rxbuf1,
  .SduLength  = 8
},
{
  .SduDataPtr = rxbuf2,
  .SduLength  = 8
},
{
  .SduDataPtr = rxbuf3,
  .SduLength  = 8
},
{
  .SduDataPtr = rxbuf4,
  .SduLength  = 8
}
};
 
void rx_activate(void)
{
  /* scheduale shot alarm for canrx task */
//  SetRelAlarm(canrx_trigger, 0, 0);
  if ( (txbuf[rxlen].SduDataPtr[0] == rxbuf[rxlen].SduDataPtr[0]) ||
       (txbuf[rxlen].SduDataPtr[1] == rxbuf[rxlen].SduDataPtr[1]) ||
       (txbuf[rxlen].SduDataPtr[2] == rxbuf[rxlen].SduDataPtr[2]) ||
       (txbuf[rxlen].SduDataPtr[3] == rxbuf[rxlen].SduDataPtr[3]) ||
       (txbuf[rxlen].SduDataPtr[4] == rxbuf[rxlen].SduDataPtr[4]) ||
       (txbuf[rxlen].SduDataPtr[5] == rxbuf[rxlen].SduDataPtr[5]) ||
       (txbuf[rxlen].SduDataPtr[6] == rxbuf[rxlen].SduDataPtr[6]) ||
       (txbuf[rxlen].SduDataPtr[7] == rxbuf[rxlen].SduDataPtr[7]) ) {
    for (int i = 0; i < 2; i++) {
       //ledToggle(BLUE);
       //ledToggle(RED);
    }
  }
}

void tx_activate(void)
{
  /* scheduale shot alarm for canrx task */
//  SetRelAlarm(canrx_trigger, 0, 0);
  for (int i = 0; i < 10; i++) {
     //ledToggle(BLUE);
  }
}

BufReq_ReturnType cantp_provide_rxbuffer(PduIdType CanTpRxPduId, PduLengthType TpSduLength, PduInfoType** PduInfoPtr)
{
  if (rxlen > sizeof(rxbuf)/sizeof(PduInfoType)) {
     /* fold back */
     rxbufptr = rxbuf;
     rxlen = 0;
  }
  //PduInfoPtr = &rxbufptr;
  *PduInfoPtr = rxbufptr;
  //rxbufptr += TpSduLength;
  //rxlen += TpSduLength;
  /* CANTP provides its internal buf holder and size which is NULL and zero-length */
  rxbufptr += 1;
  rxlen += 1;
  return BUFREQ_OK;
}

BufReq_ReturnType cantp_provide_txbuffer(PduIdType CanTpTxPduId, PduInfoType** PduInfoPtr, uint16 Length)
{
  if (txlen > sizeof(txbuf)/sizeof(PduInfoType)) {
     /* fold back */
     txbufptr = txbuf;
     txlen = 0;
  }
  *PduInfoPtr = txbufptr;
  //txbufptr += Length;
  //txlen += Length;
  /* CANTP provides its internal buf holder and size which is NULL and zero-length */
  txbufptr += 1;
  txlen += 1;
  return BUFREQ_OK;
}

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
   * Setup clocks for CAN
   */
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
   /*RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);*/
   RCC_APB1PeriphClockLPModeCmd(RCC_APB1Periph_CAN1 /*| RCC_APB1Periph_CAN2*/, ENABLE);
   RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1 /*| RCC_APB1Periph_CAN2*/, ENABLE);
   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);
   
   /* 
    * setup GPIO for CAN
    */
#define CAN_GPIO_PORT		GPIOB
#define CAN_RX_PIN 		GPIO_Pin_8
#define CAN_TX_PIN 		GPIO_Pin_9
//#define GPIO_PinSource0 	0
#define CAN_RX_SOURCE 		GPIO_PinSource0
//#define GPIO_PinSource1 	1
#define CAN_TX_SOURCE 		GPIO_PinSource1
//#define GPIO_AF_CAN1		9
#define CAN_AF_PORT		GPIO_AF_CAN1
   GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
   GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);
   GPIO_InitTypeDef GPIO_InitStructure;
   GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
   GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);
  
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
   * Intialize Tx and Rx buffers
   */
  txbufptr = &txbuf[0];
  rxbufptr = &rxbuf[0];
  txlen = rxlen = 0;
  
  /*
   * Enable interrupts
   */
  ENABLE_IRQ();
  ENABLE_FIQ();
  
  /* 
   * Now start the OS 
   */
  StartOS(OSDEFAULTAPPMODE);
  return 0;
}

#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_Task_cantxrx_START_SEC_CODE
#include "tpl_memmap.h"

TASK(cantxrx)
{
  static int txidx = 0;
  //ledToggle(RED);
  /* Scheduale a trnsmission */
  if (txidx > 3) {
     /* fold back */
     txidx = 0;
  }
  CanTp_Transmit(0, &txbuf[txidx++]);
  /* Run TP MainLoop */
  CanTp_MainFunction();
  TerminateTask();
}

#define APP_Task_cantxrx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_cantx_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_cantx)
{
  DisableAllInterrupts();
  
  //for (int i = 0; i < 1; i++) {
     //ledToggle(BLUE);
     //ledToggle(RED);
     ledOn(BLUE);
     ledOff(RED);
     for (int j = 0; j < 1000; j++) {}
  //   for (int j = 0; j < 100; j++); /* simple wait just to observe blinking only. This is completely undesirable in ISR context. XXX: remove */
  //}
  /* Call ISR wrappers */
  Can_TxIsr_wrap(0);
  
  EnableAllInterrupts();
}
#define APP_ISR_isr_cantx_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_canrx0_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_canrx0)
{
  DisableAllInterrupts();
  
  //for (int i = 0; i < 3; i++) {
     //ledToggle(RED);
     ledOn(RED);
     ledOff(BLUE);
     for (int j = 0; j < 1000; j++) {}
  //   for (int j = 0; j < 100; j++); /* simple wait just to observe blinking only. This is completely undesirable in ISR context. XXX: remove */
  //}
  /* Call ISR wrappers */
  Can_RxIsr_wrap(0, CAN_FIFO0);
  
  EnableAllInterrupts();
}
#define APP_ISR_isr_canrx0_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_canrx1_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_canrx1)
{
  DisableAllInterrupts();
  
  //for (int i = 0; i < 2; i++) {
     //ledToggle(RED);
     ledOn(RED);
     ledOff(BLUE);
     for (int j = 0; j < 1000; j++) {}
  //   for (int j = 0; j < 100; j++); /* simple wait just to observe blinking only. This is completely undesirable in ISR context. XXX: remove */
  //}
  /* Call ISR wrappers */
  Can_RxIsr_wrap(0, CAN_FIFO1);
  
  EnableAllInterrupts();
}
#define APP_ISR_isr_canrx1_STOP_SEC_CODE
#include "tpl_memmap.h"

#define APP_ISR_isr_cansce_START_SEC_CODE
#include "tpl_memmap.h"

ISR(isr_cansce)
{
  DisableAllInterrupts();
  
  for (int i = 0; i < 2; i++)
     ledToggle(ORANGE);
  /* Call ISR wrappers */
  Can_ErrIsr_wrap(0);
  
  EnableAllInterrupts();
}
#define APP_ISR_isr_cansce_STOP_SEC_CODE
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
  if (task_id == cantxrx) {
    ledOn(ORANGE);
  }
}

FUNC(void, OS_CODE) PostTaskHook()
{
  TaskType task_id = 0;
  GetTaskID(&task_id);
  if (task_id == cantxrx) {
    ledOff(ORANGE);
  }
}
#define OS_STOP_SEC_CODE
#include "tpl_memmap.h"

