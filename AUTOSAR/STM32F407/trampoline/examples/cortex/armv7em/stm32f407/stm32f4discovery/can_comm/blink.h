/*
 * blinky headers
 */
 
#ifndef __BLINK_H__
#define __BLINK_H__

#define USE_CANIF
#define USE_CANTP

#include "Can.h"
#include "CanIf.h"
#include "CanTp.h"

#define USE_LOOPBACK

/*
 * Helper routines
 */
void rx_activate(void);
void tx_activate(void);
BufReq_ReturnType cantp_provide_rxbuffer(PduIdType CanTpRxPduId, PduLengthType TpSduLength, PduInfoType** PduInfoPtr);
BufReq_ReturnType cantp_provide_txbuffer(PduIdType CanTpTxPduId, PduInfoType** PduInfoPtr, uint16 Length);

/*
 * Config structs
 */
typedef struct Comm_ConfigType
{
  const Can_ConfigType* CanConfigPtr;
  const CanIf_ConfigType* CanifConfigPtr;
  const CanTp_ConfigType* CanTpConfigPtr;
} Comm_ConfigType;

/*
Can_ConfigType CanConfig;

CanIf_ConfigType CanifConfig;
*/
extern const Comm_ConfigType CommConfig;


/*
 * CAN Callback functions wrapping CANIF exported callbacks
 */
#if CANIF_TRANSMIT_CANCELLATION == STD_ON
void cancel_tx_confirmation(const Can_PduType *can_tx_pduid);
#endif
void rx_indication(uint8 hrh, Can_IdType id, uint8 can_dlc, const uint8 *can_sdu_ptr);
void controllerbus_off(uint8 controller);
void tx_confirmation(PduIdType can_tx_pduid);
void controller_wakeup(uint8 controller);
void arc_error(uint8 controller, Can_Arc_ErrorType error);

/*
 * CANIF Callback functions, Just hooks
 * TODO: Implement
 */
void canif_busoff_notification(uint8 controller);
void canif_wakeup_notification(void);
void canif_wakeup_validnotification(void);
void canif_error_notificaton(uint8 controller, Can_Arc_ErrorType error);
void canif_user_txconfirmation(PduIdType pdu_id); 

/*
 * CANTP Callback functions
 */
void pdur_cantp_rxindication(PduIdType CanTpRxPduId, NotifResultType Result);
BufReq_ReturnType pdur_cantp_provide_rxbuffer(PduIdType CanTpRxPduId, PduLengthType TpSduLength, PduInfoType** PduInfoPtr);
void pdur_cantp_txconfirmation(PduIdType CanTpTxPduId, NotifResultType Result);
BufReq_ReturnType pdur_cantp_provide_txbuffer(PduIdType CanTpTxPduId, PduInfoType** PduInfoPtr, uint16 Length);
 
#endif
