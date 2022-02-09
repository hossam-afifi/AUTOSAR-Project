/*
 * blinky helpers
 */

#include "can_app.h"
#include "CanIf_Cbk.h"
#include "CanTp_Cbk.h"
#include "CanTp_Cfg.h"

/*
 * CAN configs
 */
const Can_FilterMaskType CanFilterMaskRef = {
  .idmr = { 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0xFF, 0xFF, 0xFF
            },
  .idar = { 0, 0, 0, 0,
            0, 0, 0, 0
            },
  .idam = 0
};

const Can_HwObType CanHoh[] = {
#if 0
{
  .CanHandleType    = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIdType        = CAN_ID_TYPE_STANDARD,
  .CanIdValue       = 0,
  .CanObjectId      = 0,
  .CanObjectType    = CAN_OBJECT_TYPE_TRANSMIT,
  .CanFilterMaskRef = &CanFilterMaskRef,
  .Can_MbMask       = 0,
  .Can_Arc_EOL      = false
},
#endif
{
  .CanHandleType    = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIdType        = CAN_ID_TYPE_STANDARD,
  .CanIdValue       = 0,
  .CanObjectId      = 0,
  .CanObjectType    = CAN_OBJECT_TYPE_RECEIVE,
  .CanFilterMaskRef = &CanFilterMaskRef,
  .Can_MbMask       = 0,
  .Can_Arc_EOL      = false
},
{
  .Can_Arc_EOL      = true
}
};

const Can_ControllerConfigType CanCtrlConfig = {
  .CanControllerId       = CAN_CTRL_1,                     /* ID 1 */
  .CanRxProcessing       = CAN_ARC_PROCESS_TYPE_INTERRUPT, /* Interrupt Mode */
  .CanTxProcessing       = CAN_ARC_PROCESS_TYPE_INTERRUPT, /* Interrupt Mode */
  .CanWakeupProcessing   = CAN_ARC_PROCESS_TYPE_INTERRUPT, /* Interrupt Mode */
  .CanBusOffProcessing   = CAN_ARC_PROCESS_TYPE_INTERRUPT, /* Interrupt Mode */
  .CanControllerBaudRate = 125,			     /* 125kbps */
  .CanControllerPropSeg  = 0,
  .CanControllerSeg1     = 12,
  .CanControllerSeg2     = 1,
  .Can_Arc_Hoh           = CanHoh,
  .Can_Arc_Loopback      = false/*true*/			     /* Operate CAN Module in LoopBack for self-testing purpose only */
};

const Can_CallbackType CanCbks = {
#if CANIF_TRANSMIT_CANCELLATION == STD_ON
  .CancelTxConfirmation = cancel_tx_confirmation,
#endif
  .RxIndication         = rx_indication,
  .ControllerBusOff     = controllerbus_off,
  .TxConfirmation       = tx_confirmation,
  .ControllerWakeup     = controller_wakeup,
  .Arc_Error            = arc_error
};

const Can_ConfigSetType CanConfigSet = {
  .CanController = &CanCtrlConfig,
  .CanCallbacks  = &CanCbks
};

Can_ConfigType CanConfig = {
  .CanConfigSet = &CanConfigSet
};

/*
 * CANIF configs
 */
const CanIf_ControllerConfigType CanIfCtlrConfig = {
  .CanIfControllerIdRef   = CANIF_Controller_A,
  .CanIfInitControllerRef = (const Can_ControllerConfigType*) &CanCtrlConfig
};

const CanIf_DispatchConfigType CanIfDispatchConfig = {
  .CanIfBusOffNotification      = canif_busoff_notification,
  .CanIfWakeUpNotification      = canif_wakeup_notification,
  .CanIfWakeupValidNotification = canif_wakeup_validnotification,
  .CanIfErrorNotificaton        = canif_error_notificaton
};

const CanIf_HrhConfigType CanIfHrhConfig1 = {
  .CanIfHrhType               = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfSoftwareFilterHrh     = true,
  .CanIfCanControllerHrhIdRef = CANIF_Controller_A,
  .CanIfHrhIdSymRef           = CAN_CTRL_0_HRH,
  .CanIfHrhRangeConfig        = NULL,
  .CanIf_Arc_EOL              = true
};

const CanIf_HrhConfigType CanIfHrhConfig2 = {
  .CanIfHrhType               = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfSoftwareFilterHrh     = true,
  .CanIfCanControllerHrhIdRef = CANIF_CHANNEL_CNT,
  .CanIfHrhIdSymRef           = CAN_CTRL_1_HRH,
  .CanIfHrhRangeConfig        = NULL,
  .CanIf_Arc_EOL              = true
};

const CanIf_HthConfigType CanIfHthConfig1 = {
  .CanIfHthType            = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfCanControllerIdRef = CANIF_Controller_A,
  .CanIfHthIdSymRef        = CAN_CTRL_0_HTH,
  .CanIf_Arc_EOL           = true
};

const CanIf_HthConfigType CanIfHthConfig2 = {
  .CanIfHthType            = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfCanControllerIdRef = CANIF_CHANNEL_CNT,
  .CanIfHthIdSymRef        = CAN_CTRL_1_HTH,
  .CanIf_Arc_EOL           = true
};

const CanIf_InitHohConfigType CanIfHohConfig[] = {
{
  .CanIfHrhConfig = &CanIfHrhConfig1,
  .CanIfHthConfig = &CanIfHthConfig1,
  .CanIf_Arc_EOL  = false
},
{
  .CanIfHrhConfig = &CanIfHrhConfig2,
  .CanIfHthConfig = &CanIfHthConfig2,
  .CanIf_Arc_EOL  = true
}
};

const CanIf_HrhConfigType CanIfCanRxPduHrhRef = {
  .CanIfHrhType               = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfSoftwareFilterHrh     = true,
  .CanIfCanControllerHrhIdRef = CANIF_Controller_A,
  .CanIfHrhIdSymRef           = 0,
  .CanIfHrhRangeConfig        = NULL,
  .CanIf_Arc_EOL              = true
};

const CanIf_RxPduConfigType CanIfRxPduConfig = {
  .CanIfCanRxPduId            = 1, /* corresponds to index of Receive configs in CanTpNSduList array */
  .CanIfCanRxPduCanId         = 256,
  .CanIfCanRxPduDlc           = 8,
#if CANIF_CANPDUID_READDATA_API == STD_ON
  .CanIfReadRxPduData         = false,
#endif
#if CANIF_READRXPDU_NOTIF_STATUS_API == STD_ON
  .CanIfReadRxPduNotifyStatus = false,
#endif
  .CanIfRxPduIdCanIdType      = CANIF_CAN_ID_TYPE_11,
  .CanIfRxUserType            = CANIF_USER_TYPE_CAN_TP,
  .CanIfUserRxIndication      = NULL,                  /* Only registered whenevr the CANIF is directly application driven */
  .CanIfCanRxPduHrhRef        = &CanIfCanRxPduHrhRef,
  .PduIdRef                   = NULL,
  .CanIfSoftwareFilterType    = CANIF_SOFTFILTER_TYPE_MASK,
  .CanIfCanRxPduCanIdMask     = 0xFF
};

const CanIf_HthConfigType CanIfCanTxPduHthRef = {
  .CanIfHthType            = CAN_ARC_HANDLE_TYPE_BASIC,
  .CanIfCanControllerIdRef = CANIF_Controller_A,
  .CanIfHthIdSymRef        = 0,
  .CanIf_Arc_EOL           = true
};

#if CANIF_ARC_RUNTIME_PDU_CONFIGURATION == STD_OFF
const
#endif
CanIf_TxPduConfigType CanIfTxPduConfig = {
  .CanIfTxPduId               = 0,
  .CanIfCanTxPduIdCanId       = 512,
  .CanIfCanTxPduIdDlc         = 8,
  .CanIfCanTxPduType          = CANIF_PDU_TYPE_STATIC,
#if CANIF_READTXPDU_NOTIFY_STATUS_API == STD_ON
  .CanIfReadTxPduNotifyStatus = false,
#endif
  .CanIfTxPduIdCanIdType      = CANIF_CAN_ID_TYPE_11,
  .CanIfUserTxConfirmation    = canif_user_txconfirmation,
  .CanIfCanTxPduHthRef        = &CanIfCanTxPduHthRef,
  .PduIdRef                   = NULL
};

#define CanIfTxPduConfigData_SZ   sizeof(CanIfTxPduConfig)
#define CanIf_TxPduConfigData_SZ  sizeof(CanIf_TxPduConfigType)
#define CanIfRxPduConfigData_SZ   sizeof(CanIfRxPduConfig)
#define CanIf_RxPduConfigData_SZ  sizeof(CanIf_RxPduConfigType)

const CanIf_InitConfigType InitConfig = {
  .CanIfConfigSet                  = 0,
  .CanIfNumberOfCanRxPduIds        = CanIfRxPduConfigData_SZ/CanIf_RxPduConfigData_SZ,
  .CanIfNumberOfCanTXPduIds        = CanIfTxPduConfigData_SZ/CanIf_TxPduConfigData_SZ,
  .CanIfNumberOfDynamicCanTXPduIds = 0,
  .CanIfHohConfigPtr               = CanIfHohConfig,
  .CanIfRxPduConfigPtr             = &CanIfRxPduConfig,
  .CanIfTxPduConfigPtr             = &CanIfTxPduConfig
};

CanIf_ConfigType CanifConfig = {
  .ControllerConfig            = &CanIfCtlrConfig,
  .DispatchConfig              = &CanIfDispatchConfig,
  .InitConfig                  = &InitConfig,
  .TransceiverConfig           = NULL,
  .Arc_ChannelToControllerMap  = CAN_CTRL_1,
  .Arc_ChannelDefaultConfIndex = 0
};

/*
 * CANTP configs
 */

const CanTp_GeneralType CanTpGeneral = {
  .main_function_period = 10
};

const CanTp_NSduType CanTpNSduConfigList[] = {
#if 0
{
  .direction    = IS015765_TRANSMIT,
  .listItemType = CANTP_NOT_LAST_ENTRY,
  .configData.CanTpTxNSdu   = {
    .CanIf_PduId               = 0,
    .PduR_PduId                = 0,
    .CanTp_FcPduId             = 0,
    .CanTpAddressingMode       = CANTP_STANDARD,
    .reserved0                 = 0,
    .CanTpNas                  = 100,
    .CanTpNbs                  = 100,
    .CanTpNcs                  = 100,
    .CanTpTxChannel            = 2,             /* 4 runtime entries, upper 2 are for Tx (Nsdu Index) */
    .CanTpTxDI                 = 8,
    .CanTpTxPaddingActivation  = CANTP_ON,      /* Enable Padding */
    .CanTpTxTaType             = CANTP_PHYSICAL,
    .reserved1                 = 0,
    .reserved2                 = 0,
    .CanTpNSa                  = {
       0
    },
    .CanTpNTa                  = {
       0
    },
    .ll_dl                     = 8,
    .PduR_CanTpTxConfirmation  = pdur_cantp_txconfirmation,
    .PduR_CanTpProvideTxBuffer = pdur_cantp_provide_txbuffer
  }
},
#endif
{
  .direction    = ISO15765_RECEIVE,
  .listItemType = CANTP_END_OF_LIST,
  .configData.CanTpRxNSdu   = {
    .CanTp_FcPduId             = 0,
    .CanIf_FcPduId             = 0,
    .PduR_PduId                = 0,
    .CanTpAddressingFormant    = CANTP_STANDARD,
    .CanTpBs                   = 8,
    .CanTpNar                  = 100,
    .CanTpNbr                  = 100,
    .CanTpNcr                  = 100,
    .CanTpRxChannel            = 0,             /* 4 runtime entries, upper 2 are for Tx (Nsdu Index) */
    .CanTpRxDI                 = 8,
    .CanTpRxPaddingActivation  = CANTP_ON,      /* Enable Padding */
    .CanTpRxTaType             = CANTP_PHYSICAL,
    .CanTpWftMax               = 10,
    .CanTpSTmin                = 1,
    .CanTpNSa                  = {
       0
    },
    .CanTpNTa                  = {
       0
    },
    .ll_dl                     = 8,
    .PduR_CanTpRxIndication    = pdur_cantp_rxindication,
    .PduR_CanTpProvideRxBuffer = pdur_cantp_provide_rxbuffer
  }
}
};

const CanTp_RxIdType CanTpRxIdList[] = {
{
  .CanTpAddressingMode   = CANTP_STANDARD,
  .CanTpNSduIndex        = 0,
  .CanTpReferringTxIndex = 0
},
{
  .CanTpAddressingMode   = CANTP_STANDARD,
  .CanTpNSduIndex        = 1,
  .CanTpReferringTxIndex = 1
}
};

const CanTp_ConfigType CanTpConfig = {
  .CanTpGeneral  = &CanTpGeneral,
  .CanTpNSduList = CanTpNSduConfigList,
  .CanTpRxIdList = CanTpRxIdList
};

/*
 * COMM configs
 */
const Comm_ConfigType CommConfig = {
  .CanConfigPtr   = (const Can_ConfigType*) &CanConfig,
  .CanifConfigPtr = (const CanIf_ConfigType*) &CanifConfig,
  .CanTpConfigPtr = (const CanTp_ConfigType*) &CanTpConfig
};

/*
 * Callback functions
 */
#if CANIF_TRANSMIT_CANCELLATION == STD_ON
void cancel_tx_confirmation(const Can_PduType *can_tx_pduid)
{
  CanIf_CancelTxConfirmation(can_tx_pduid);
}
#endif

void rx_indication(uint8 hrh, Can_IdType id, uint8 can_dlc, const uint8 *can_sdu_ptr)
{
  CanIf_RxIndication(hrh, id, can_dlc, can_sdu_ptr);
}

void controllerbus_off(uint8 controller)
{
  CanIf_ControllerBusOff(controller);
}

void tx_confirmation(PduIdType can_tx_pduid)
{
  CanIf_TxConfirmation(can_tx_pduid);
}

void controller_wakeup(uint8 controller)
{
  CanIf_SetWakeupEvent(controller);
}

void arc_error(uint8 controller, Can_Arc_ErrorType error)
{
  CanIf_Arc_Error(controller, error);
}

void canif_busoff_notification(uint8 controller)
{
}
void canif_wakeup_notification(void)
{
}
void canif_wakeup_validnotification(void)
{
}
void canif_error_notificaton(uint8 controller, Can_Arc_ErrorType error)
{
}

void canif_user_txconfirmation(PduIdType pdu_id)
{
  /*
   * Acknowledge data transmission over CAN bus to upper layers
   * Behaviour is implementation defined:
   *  1. Activate the Reception task for checking correct data
   *     received in LoopBack mode only
   *  2. Activate the Transmission task again after checking for
   *     more data, sleep down the CAN controller if no data
   *     available, or just terminate the task
   *  3. Route back to lower layers to dequeues buffered data
   */
#ifdef USE_LOOPBACK
  //rx_activate();
#endif
  CanTp_TxConfirmation(pdu_id);
}

void pdur_cantp_rxindication(PduIdType CanTpRxPduId, NotifResultType Result)
{
  /*
   * Acknowledge data receiption over CAN bus to upper layers
   * Behaviour is implementation defined:
   *  1. Activate the Reception task for checking correct data
   *     received in LoopBack mode only
   *  2. Activate the Transmission task again after checking for
   *     more data, sleep down the CAN controller if no data
   *     available, or just terminate the task
   *  3. Route back to lower layers to dequeues buffered data
   */
#ifdef USE_LOOPBACK
  rx_activate();
#endif
}

BufReq_ReturnType pdur_cantp_provide_rxbuffer(PduIdType CanTpRxPduId, PduLengthType TpSduLength, PduInfoType** PduInfoPtr)
{
  return cantp_provide_rxbuffer(CanTpRxPduId, TpSduLength, PduInfoPtr);
}

void pdur_cantp_txconfirmation(PduIdType CanTpTxPduId, NotifResultType Result)
{
  /*
   * Acknowledge data transmission over CAN bus to upper layers
   * Behaviour is implementation defined:
   *  1. Activate the Reception task for checking correct data
   *     received in LoopBack mode only
   *  2. Activate the Transmission task again after checking for
   *     more data, sleep down the CAN controller if no data
   *     available, or just terminate the task
   *  3. Route back to lower layers to dequeues buffered data
   */
#ifdef USE_LOOPBACK
  tx_activate();
#endif
}

BufReq_ReturnType pdur_cantp_provide_txbuffer(PduIdType CanTpTxPduId, PduInfoType** PduInfoPtr, uint16 Length)
{
  return cantp_provide_txbuffer(CanTpTxPduId, PduInfoPtr, Length);
}
