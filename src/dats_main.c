/*************************************************************************************************/
/*!
 *  \file   dats_main.c
 *
 *  \brief  Data transmitter sample application.
 *
 *          $Date: 2017-03-09 12:18:38 -0600 (Thu, 09 Mar 2017) $
 *          $Revision: 11460 $
 *
 *  Copyright (c) 2012-2017 ARM Ltd., all rights reserved.
 *  ARM Ltd. confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact ARM Ltd. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include <ble_freertos_dats.h>
#include "svc_ch.h"
#include "svc_core.h"
#include "svc_wp.h"
#include "calc128.h"
#include "util/bstream.h"
#include "accelerometer_functions.h"

extern float acceleration_mg[3];
/**************************************************************************************************
  Macros
**************************************************************************************************/
/*! Enumeration of client characteristic configuration descriptors */
enum
{
  DATS_GATT_SC_CCC_IDX,           /*! GATT service, service changed characteristic */
  DATS_WP_DAT_CCC_IDX,            /*! ARM Ltd. proprietary service, data transfer characteristic */
  DATS_NUM_CCC_IDX
};

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t datsAdvCfg =
{
  {30000,     0,     0},                  /*! Advertising durations in ms */
  {   96,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t datsSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t datsSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  DM_KEY_DIST_IRK,                        /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK | DM_KEY_DIST_IRK,      /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! SMP security parameter configuration */
static const smpCfg_t datsSmpCfg =
{
  3000,                                   /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0,                                      /*! Device authentication requirements */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t datsUpdateCfg =
{
  2000,                                   /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  150,                                    /*! Minimum connection interval in 1.25ms units */
  160,                                    /*! Maximum connection interval in 1.25ms units */
  4,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! local IRK */
static uint8_t localIrk[] =
{
  0x95, 0xC8, 0xEE, 0x6F, 0xC5, 0x0D, 0xEF, 0x93, 0x35, 0x4E, 0x7C, 0x57, 0x08, 0xE2, 0xA3, 0x85
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t datsAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,

  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_ARM)             /*! company ID */
};

/*! scan data, discoverable mode */
static const uint8_t datsScanDataDisc[] =
{
  /*! device name */
  8,                                      /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'D',
  'a',
  't',
  'a',
  ' ',
  'T',
  'X'
};

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t datsCccSet[DATS_NUM_CCC_IDX] =
{
  /* cccd handle          value range               security level */
  {GATT_SC_CH_CCC_HDL,    ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* DATS_GATT_SC_CCC_IDX */
  {WP_DAT_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* DATS_WP_DAT_CCC_IDX */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
static struct
{
  wsfHandlerId_t    handlerId;        /* WSF handler ID */
} datsCb;

/* LESC OOB configuration */
static dmSecLescOobCfg_t *datsOobCfg;

/**
 * @brief : translate  float IEEE754 to 4 bytes
 * @param d : buffer to store the bytes
 * @param offset : starting point in the buffer
 * @param val : float value to store
 *
 * return the next place to store in buffer
 */
uint8_t byte_to_float(uint8_t *d, uint8_t offset, float val)
{
    uint8_t i;
    union {
        uint8_t array[4];
        float value;
    } conv;

    conv.value = val;

    for (i = 0; i < 4; i++){
        d[offset+i] = conv.array[3-i] ; //or d[offset+i] = conv.array[i] ;  depending on endianness
    }
    return(offset + i);
}

/*************************************************************************************************/
/*!
 *  \fn     datsSendData
 *
 *  \brief  Send notification containing data.
 *
 *  \param  connId      DM connection ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsSendData(dmConnId_t connId)
{
    APP_TRACE_INFO0("datsSendData");

  /**
   * THIS is the place to include your code to read the accellerometer
   * or sent the latest version of the data
   *
   * Below just an examples
   */

  // max SendPacket size is MTU (default 23 - 3 = 20)
  // if longer datsAttCback will fail with code 0x77
  // if you need to sent more sent multiple
  uint8_t SendPacket[12];
  uint8_t cnt = 0;

  getAccelerometerReadings();

  // now turn the float to bytes
  cnt = byte_to_float(SendPacket, cnt, acceleration_mg[0]);
  cnt = byte_to_float(SendPacket, cnt, acceleration_mg[1]);
  cnt = byte_to_float(SendPacket, cnt, acceleration_mg[2]);

  if (AttsCccEnabled(connId, DATS_WP_DAT_CCC_IDX))
  {
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(SendPacket), SendPacket);
  }

  return;

  // another example is using a string
  uint8_t str[] = "hello.....from dats";
  if (AttsCccEnabled(connId, DATS_WP_DAT_CCC_IDX))
  {
    /* send notification */
    AttsHandleValueNtf(connId, WP_DAT_HDL, sizeof(str), str);
  }
}

static void datsSetup(dmEvt_t *pMsg);

/*************************************************************************************************/
/*!
 *  \fn     datsDmCback
 *
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsDmCback(dmEvt_t *pDmEvt)
{
  uint16_t  len;

  if (pDmEvt->hdr.event == DM_SEC_ECC_KEY_IND)
  {
    DmSecSetEccKey(&pDmEvt->eccMsg.data.key);
    datsSetup(NULL);

    if (datsSecCfg.oob)
    {
      uint8_t oobLocalRandom[SMP_RAND_LEN];
      SecRand(oobLocalRandom, SMP_RAND_LEN);
      DmSecCalcOobReq(oobLocalRandom, pDmEvt->eccMsg.data.key.pubKey_x);
    }
  }
  else if (pDmEvt->hdr.event == DM_SEC_CALC_OOB_IND)
  {
    if (datsOobCfg == NULL)
    {
      datsOobCfg = WsfBufAlloc(sizeof(dmSecLescOobCfg_t));
    }

    if (datsOobCfg)
    {
      Calc128Cpy(datsOobCfg->localConfirm, pDmEvt->oobCalcInd.confirm);
      Calc128Cpy(datsOobCfg->localRandom, pDmEvt->oobCalcInd.random);
    }
  }
  else
  {
    dmEvt_t   *pMsg;

    len = DmSizeOfEvt(pDmEvt);

    if ((pMsg = WsfMsgAlloc(len)) != NULL)
    {
      memcpy(pMsg, pDmEvt, len);
      WsfMsgSend(datsCb.handlerId, pMsg);
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     datsAttCback
 *
 *  \brief  Application ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsAttCback(attEvt_t *pEvt)
{
  APP_TRACE_INFO0("datsATTCback");

  if (pEvt->hdr.status != ATT_SUCCESS) {
      APP_TRACE_INFO1("Failed to send data, error 0x%x", pEvt->hdr.status);
  }

  // this is good place if you want to create a
  // wait a little, do a new measurement, store the values and
  // call DatsSendData()
}

/*************************************************************************************************/
/*!
 *  \fn     datsCccCback
 *
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsCccCback(attsCccEvt_t *pEvt)
{
  APP_TRACE_INFO0("datsCccCback");
  appDbHdl_t    dbHdl;

  /* if CCC not set from initialization and there's a device record */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
  {
    /* store value in device database */
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     datsWpWriteCback
 *
 *  \brief  ATTS write callback for proprietary data service.
 *
 *  \return ATT status.
 */
/*************************************************************************************************/
uint8_t datsWpWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation,
                          uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
  /* print received data */
  APP_TRACE_INFO1("datsWpWriteCback: %c",(const char*) pValue);

  // depending on what you receive from the client you can decide to stay in polling mode
  // or go to streaming mode on this server

  /* send back some data */
  datsSendData(connId);

  return ATT_SUCCESS;
}

/*************************************************************************************************/
/*!
 *  \fn     datsSetup
 *
 *  \brief  Set up advertising and other procedures that need to be performed after
 *          device reset.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(datsAdvDataDisc), (uint8_t *) datsAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(datsScanDataDisc), (uint8_t *) datsScanDataDisc);

  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \fn     datsProcMsg
 *
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;

  switch(pMsg->hdr.event)
  {
    case ATT_MTU_UPDATE_IND:
      APP_TRACE_INFO1("Negotiated MTU %d", ((attEvt_t *)pMsg)->mtu);
      break;

    case DM_RESET_CMPL_IND:
      DmSecGenerateEccKeyReq();
      uiEvent = APP_UI_RESET_CMPL;
      break;

    case DM_ADV_START_IND:
      uiEvent = APP_UI_ADV_START;
      break;

    case DM_ADV_STOP_IND:
      uiEvent = APP_UI_ADV_STOP;
      break;

    case DM_CONN_OPEN_IND:
      uiEvent = APP_UI_CONN_OPEN;
      break;

    case DM_CONN_CLOSE_IND:
      uiEvent = APP_UI_CONN_CLOSE;
      break;

    case DM_SEC_PAIR_CMPL_IND:
      uiEvent = APP_UI_SEC_PAIR_CMPL;
      break;

    case DM_SEC_PAIR_FAIL_IND:
      uiEvent = APP_UI_SEC_PAIR_FAIL;
      break;

    case DM_SEC_ENCRYPT_IND:
      uiEvent = APP_UI_SEC_ENCRYPT;
      break;

    case DM_SEC_ENCRYPT_FAIL_IND:
      uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
      break;

    case DM_SEC_AUTH_REQ_IND:

      if (pMsg->authReq.oob)
      {
        dmConnId_t connId = (dmConnId_t) pMsg->hdr.param;

        /* TODO: Perform OOB Exchange with the peer. */


        /* TODO: Fill datsOobCfg peerConfirm and peerRandom with value passed out of band */

        if (datsOobCfg != NULL)
        {
          DmSecSetOob(connId, datsOobCfg);
        }

        DmSecAuthRsp(connId, 0, NULL);
      }
      else
      {
        AppHandlePasskey(&pMsg->authReq);
      }
      break;

    case DM_SEC_COMPARE_IND:
      AppHandleNumericComparison(&pMsg->cnfInd);
      break;

    case DM_VENDOR_SPEC_CMD_CMPL_IND:
      {
        #if defined(AM_PART_APOLLO) || defined(AM_PART_APOLLO2)

          uint8_t *param_ptr = &pMsg->vendorSpecCmdCmpl.param[0];

          switch (pMsg->vendorSpecCmdCmpl.opcode)
          {
            case 0xFC20: //read at address
            {
              uint32_t read_value;

              BSTREAM_TO_UINT32(read_value, param_ptr);

              APP_TRACE_INFO3("VSC 0x%0x complete status %x param %x",
                pMsg->vendorSpecCmdCmpl.opcode,
                pMsg->hdr.status,
                read_value);
            }

            break;
            default:
                APP_TRACE_INFO2("VSC 0x%0x complete status %x",
                    pMsg->vendorSpecCmdCmpl.opcode,
                    pMsg->hdr.status);
            break;
          }

        #endif
      }
      break;

      default:
      break;
  }

  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     DatsHandlerInit
 *
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DatsHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("DatsHandlerInit");

  /* store handler ID */
  datsCb.handlerId = handlerId;

  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &datsSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &datsAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &datsSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &datsUpdateCfg;
  pSmpCfg = (smpCfg_t *) &datsSmpCfg;

  /* Initialize application framework */
  AppSlaveInit();

  /* Set IRK for the local device */
  DmSecSetLocalIrk(localIrk);
}

/*************************************************************************************************/
/*!
 *  \fn     datsBtnCback
 *
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
// NOT USED, but you could use this if you want to include some buttons..
static void datsBtnCback(uint8_t btn)
{
  APP_TRACE_INFO0("datsBtnCback");
  if (AppConnIsOpen() != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      default:
        break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     datsWsfBufDiagnostics
 *
 *  \brief  Callback for WSF buffer diagnostic messages.
 *
 *  \param  pInfo     Diagnostics message
 *
 *  \return None.
 */
/*************************************************************************************************/
static void datsWsfBufDiagnostics(WsfBufDiag_t *pInfo)
{
  if (pInfo->type == WSF_BUF_ALLOC_FAILED)
  {
    APP_TRACE_INFO2("Dats got WSF Buffer Allocation Failure - Task: %d Len: %d",
                     pInfo->param.alloc.taskId, pInfo->param.alloc.len);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     DatsHandler
 *
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DatsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Dats got evt %d", pMsg->event);

    if (pMsg->event >= DM_CBACK_START && pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);

      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }

    /* perform profile and user interface-related operations */
    datsProcMsg((dmEvt_t *) pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     DatsStart
 *
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void DatsStart(void)
{
  APP_TRACE_INFO0("DatsStart");

  /* Register for stack callbacks */
  DmRegister(datsDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, datsDmCback);
  AttRegister(datsAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(DATS_NUM_CCC_IDX, (attsCccSet_t *) datsCccSet, datsCccCback);

  /* Initialize attribute server database */
  SvcCoreAddGroup();
  SvcWpCbackRegister(NULL, datsWpWriteCback);
  SvcWpAddGroup();

  /* Register for app framework button callbacks */
  AppUiBtnRegister(datsBtnCback);

  WsfBufDiagRegister(datsWsfBufDiagnostics);

  /* Reset the device */
  DmDevReset();
}
