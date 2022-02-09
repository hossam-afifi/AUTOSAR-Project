#include <FreeRTOS.h>
#include <task.h>

#include<FreeRTOSConfig.h>

#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <queue.h>
#include "stm32f10x.h"
#include "can_app.h"

#define mainQUEUE_RECEIVE_TASK_PRIORITY     ( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_SEND_TASK_PRIORITY        ( tskIDLE_PRIORITY + 1 )
#define mainQUEUE_LENGTH                    ( 1 )
#define mainQUEUE_SEND_FREQUENCY_MS         ( 200 / portTICK_PERIOD_MS )
/* The queue used by both tasks. */
static QueueHandle_t yQueue = NULL;

#define CAN1_TX_IRQn  						19
#define CAN1_RX0_IRQn 						20
#define CAN1_RX1_IRQn 						21

#define RCC_APB2Periph_GPIO_CAN1    RCC_APB2Periph_GPIOB/*RCC_APB2Periph_GPIOA*/
#define GPIO_Remapping_CAN1         GPIO_Remap1_CAN1
#define GPIO_CAN1                   GPIOB/*GPIOA*/
#define GPIO_Pin_CAN1_RX            GPIO_Pin_8/*GPIO_Pin_11*/
#define GPIO_Pin_CAN1_TX            GPIO_Pin_9/*GPIO_Pin_12*/
#define GPIO_CAN                    GPIO_CAN1
#define GPIO_Remapping_CAN          GPIO_Remapping_CAN1
#define GPIO_Pin_CAN_RX             GPIO_Pin_CAN1_RX
#define GPIO_Pin_CAN_TX             GPIO_Pin_CAN1_TX

static void cantxrx( void *pvParameters );

/*
 * Tx and Rx buffers
 */

PduInfoType* txbufptr;
PduInfoType* rxbufptr;
int txlen;
int rxlen;

uint8 txbuf1[] = { 1, 2, 3, 4, 5, 6, 7, 8 };
uint8 txbuf2[] = { 9, 10, 11, 12, 13, 14, 15, 16 };
uint8 txbuf3[] = { 17, 18, 19, 20, 21, 22, 23, 24 };
uint8 txbuf4[] = { 25, 26, 27, 28, 29, 30, 31, 32 };

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
			 GPIOC->BSRR = (1 << (13 + 16));
			 GPIOC->BSRR |= (1 << 13);
    }
  }
}

void tx_activate(void)
{
  /* scheduale shot alarm for canrx task */
//  SetRelAlarm(canrx_trigger, 0, 0);
  for (int i = 0; i < 10; i++)
			 GPIOC->BSRR = (1 << (13 + 16));
			 GPIOC->BSRR |= (1 << 13);
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

void main_can(void)
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
  /* Enable the CAN1 Rx0 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* Enable the CAN1 Rx1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* 
   * simple board intialization 
   */
  //initBoard();
  
  /*
   * Setup clocks for CAN
   */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
#if 1	 
	/*
	 * GPIO setup for CAN 
	 */
	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_CAN1, ENABLE);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Configure CAN pin: RX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_RX;
  GPIO_InitStructure.GPIO_Mode = /*GPIO_Mode_IN_FLOATING*/GPIO_Mode_IPU;
  GPIO_Init(GPIO_CAN, &GPIO_InitStructure);
  
  /* Configure CAN pin: TX */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_CAN_TX;
  GPIO_InitStructure.GPIO_Mode = /*GPIO_Mode_AF_OD*/GPIO_Mode_AF_PP/*GPIO_Mode_Out_OD*//*GPIO_Mode_AF_PP*/;
  GPIO_InitStructure.GPIO_Speed = /*GPIO_Speed_2MHz*/GPIO_Speed_50MHz;
  GPIO_Init(GPIO_CAN, &GPIO_InitStructure);
  
  GPIO_PinRemapConfig(GPIO_Remapping_CAN , ENABLE);
#endif
  
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
	__enable_irq();
	__enable_fiq();
	
	/* Create the queue. */
  yQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint32_t ) );

  if( yQueue != NULL )
  {
     /* Start the two tasks as described in the comments at the top of this
      file. */
     xTaskCreate( cantxrx,                         /* The function that implements the task. */
                  "TXRX",                          /* The text name assigned to the task - for debug only as it is not used by the kernel. */
                  configMINIMAL_STACK_SIZE,        /* The size of the stack to allocate to the task. */
                  NULL,                            /* The parameter passed to the task - not used in this case. */
                  mainQUEUE_RECEIVE_TASK_PRIORITY, /* The priority assigned to the task. */
                  NULL );                          /* The task handle is not required, so NULL is passed. */

     /* Start the tasks and timer running. */
     vTaskStartScheduler();
    }

    /* If all is well, the scheduler will now be running, and the following
    line will never be reached.  If the following line does execute, then
    there was insufficient FreeRTOS heap memory available for the Idle and/or
    timer tasks to be created.  See the memory management section on the
    FreeRTOS web site for more details on the FreeRTOS heap
    http://www.freertos.org/a00111.html. */
    for( ;; );
}

static void cantxrx( void *pvParameters )
{
 for(;;) {
  static int txidx = 0;
  /* Scheduale a trnsmission */
  if (txidx > 3) {
     /* fold back */
     txidx = 0;
  }

  CanTp_Transmit(0, &txbuf[txidx++]);
  /* Run TP MainLoop */
  CanTp_MainFunction();
  //TerminateTask();
	//vTaskSuspendAll();
}
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
  taskDISABLE_INTERRUPTS();
  
  for (int i = 0; i < 2; i++) {
		GPIOC->BSRR = (1 << (13 + 16));
		/* 
		 * simple wait just to observe blinking only. 
		 * This is completely undesirable in ISR context. 
		 * XXX: remove 
		 */
     for (int j = 0; j < 100; j++);
		 GPIOC->BSRR |= (1 << 13);
		 for (int j = 0; j < 100; j++);
  }
  /* Call ISR wrappers */
  Can_TxIsr_wrap(0);
  
  taskENABLE_INTERRUPTS();
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  taskDISABLE_INTERRUPTS();
  
  for (int i = 0; i < 2; i++) {
		GPIOC->BSRR = (1 << (13 + 16));
		/* 
		 * simple wait just to observe blinking only. 
		 * This is completely undesirable in ISR context. 
		 * XXX: remove 
		 */
     for (int j = 0; j < 100; j++);
		 GPIOC->BSRR |= (1 << 13);
		 for (int j = 0; j < 100; j++);
  }
  /* Call ISR wrappers */
  Can_RxIsr_wrap(0, CAN_FIFO0);
  
  taskENABLE_INTERRUPTS();
}

void CAN1_RX1_IRQHandler(void)
{
  taskDISABLE_INTERRUPTS();
  
  for (int i = 0; i < 2; i++) {
			 GPIOC->BSRR = (1 << (13 + 16));
		/* 
		 * simple wait just to observe blinking only. 
		 * This is completely undesirable in ISR context. 
		 * XXX: remove 
		 */
     for (int j = 0; j < 100; j++);
		 GPIOC->BSRR |= (1 << 13);
		 for (int j = 0; j < 100; j++);
  }
  /* Call ISR wrappers */
  Can_RxIsr_wrap(0, CAN_FIFO1);
  
  taskENABLE_INTERRUPTS();
}

void CAN1_SCE_IRQHandler(void)
{
  taskDISABLE_INTERRUPTS();
  
  for (int i = 0; i < 2; i++)
			 GPIOC->BSRR = (1 << (13 + 16));
			 GPIOC->BSRR |= (1 << 13);
  /* Call ISR wrappers */
  Can_ErrIsr_wrap(0);
  
  taskENABLE_INTERRUPTS();
}

