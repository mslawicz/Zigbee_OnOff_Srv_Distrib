
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/app_zigbee.c
  * @author  MCD Application Team
  * @brief   Zigbee Application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019-2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_common.h"
#include "app_entry.h"
#include "dbg_trace.h"
#include "app_zigbee.h"
#include "zigbee_interface.h"
#include "shci.h"
#include "stm_logging.h"
#include "app_conf.h"
#include "stm32wbxx_core_interface_def.h"
#include "zigbee_types.h"
#include "stm32_seq.h"

/* Private includes -----------------------------------------------------------*/
#include <assert.h>
#include "zcl/zcl.h"
#include "zcl/general/zcl.identify.h"
#include "zcl/general/zcl.groups.h"
#include "zcl/general/zcl.scenes.h"
#include "zcl/general/zcl.onoff.h"
#include "zcl/general/zcl.color.h"
#include "zcl/general/zcl.level.h"

/* USER CODE BEGIN Includes */
#include "zcl/general/zcl.basic.h"
#include "app_RGB_LED.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define APP_ZIGBEE_STARTUP_FAIL_DELAY               500U
#define CHANNEL                                     11

#define SW1_ENDPOINT                                17

/* Scenes (endpoint 1) specific defines ------------------------------------------------*/
#define ZCL_SCENES_MAX_SCENES_1                      20
/* USER CODE BEGIN Scenes (endpoint 1) defines */
/* USER CODE END Scenes (endpoint 1) defines */

/* USER CODE BEGIN PD */
#define SW1_GROUP_ADDR                              0x0001
#define ZCL_LEVEL_ATTR_ONOFF_TRANS_TIME_DEFAULT	10

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define ATTR_COLOR_TEMP_BEGIN		200
/* USER CODE END PM */

/* External definition -------------------------------------------------------*/
enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config);

/* USER CODE BEGIN ED */
/* USER CODE END ED */

/* Private function prototypes -----------------------------------------------*/
static void APP_ZIGBEE_StackLayersInit(void);
static void APP_ZIGBEE_ConfigEndpoints(void);
static void APP_ZIGBEE_NwkForm(void);

static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode);
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void);

static void Wait_Getting_Ack_From_M0(void);
static void Receive_Ack_From_M0(void);
static void Receive_Notification_From_M0(void);

static void APP_ZIGBEE_ProcessNotifyM0ToM4(void);
static void APP_ZIGBEE_ProcessRequestM0ToM4(void);

/* USER CODE BEGIN PFP */
static void APP_ZIGBEE_ConfigGroupAddr(void);
static void APP_ZIGBEE_JoinReq(struct ZigBeeT* zb, void* arg);
static void RGB_timer_callback(struct ZigBeeT* zb, void* arg);
/* USER CODE END PFP */

/* Private variables ---------------------------------------------------------*/
static TL_CmdPacket_t   *p_ZIGBEE_otcmdbuffer;
static TL_EvtPacket_t   *p_ZIGBEE_notif_M0_to_M4;
static TL_EvtPacket_t   *p_ZIGBEE_request_M0_to_M4;
static __IO uint32_t    CptReceiveNotifyFromM0 = 0;
static __IO uint32_t    CptReceiveRequestFromM0 = 0;

PLACE_IN_SECTION("MB_MEM1") ALIGN(4) static TL_ZIGBEE_Config_t ZigbeeConfigBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static TL_CmdPacket_t ZigbeeOtCmdBuffer;
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRspEvtBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];
PLACE_IN_SECTION("MB_MEM2") ALIGN(4) static uint8_t ZigbeeNotifRequestBuffer[sizeof(TL_PacketHeader_t) + TL_EVT_HDR_SIZE + 255U];

struct zigbee_app_info
{
  bool has_init;
  struct ZigBeeT *zb;
  enum ZbStartType startupControl;
  enum ZbStatusCodeT join_status;
  uint32_t join_delay;
  bool init_after_join;

  struct ZbZclClusterT *identify_server_1;
  struct ZbZclClusterT *groups_server_1;
  struct ZbZclClusterT *scenes_server_1;
  struct ZbZclClusterT *onOff_server_1;
  struct ZbZclClusterT *colorControl_server_1;
  struct ZbZclClusterT *levelControl_server_1;
};
static struct zigbee_app_info zigbee_app_info;

/* OnOff server 1 custom callbacks */
static enum ZclStatusCodeT onOff_server_1_off(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT onOff_server_1_on(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT onOff_server_1_toggle(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg);

static struct ZbZclOnOffServerCallbacksT OnOffServerCallbacks_1 =
{
  .off = onOff_server_1_off,
  .on = onOff_server_1_on,
  .toggle = onOff_server_1_toggle,
};

/* ColorControl server 1 custom callbacks */
static enum ZclStatusCodeT colorControl_server_1_move_to_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveToColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_move_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_step_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientStepColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_move_to_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveToColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_color_loop_set(struct ZbZclClusterT *cluster, struct ZbZclColorClientColorLoopSetReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_stop_move_step(struct ZbZclClusterT *cluster, struct ZbZclColorClientStopMoveStepReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_move_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT colorControl_server_1_step_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientStepColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);

static struct ZbZclColorServerCallbacksT ColorServerCallbacks_1 =
{
  .move_to_color_xy = colorControl_server_1_move_to_color_xy,
  .move_color_xy = colorControl_server_1_move_color_xy,
  .step_color_xy = colorControl_server_1_step_color_xy,
  .move_to_color_temp = colorControl_server_1_move_to_color_temp,
  .color_loop_set = colorControl_server_1_color_loop_set,
  .stop_move_step = colorControl_server_1_stop_move_step,
  .move_color_temp = colorControl_server_1_move_color_temp,
  .step_color_temp = colorControl_server_1_step_color_temp,
};

/* LevelControl server 1 custom callbacks */
static enum ZclStatusCodeT levelControl_server_1_move_to_level(struct ZbZclClusterT *cluster, struct ZbZclLevelClientMoveToLevelReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT levelControl_server_1_move(struct ZbZclClusterT *cluster, struct ZbZclLevelClientMoveReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT levelControl_server_1_step(struct ZbZclClusterT *cluster, struct ZbZclLevelClientStepReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);
static enum ZclStatusCodeT levelControl_server_1_stop(struct ZbZclClusterT *cluster, struct ZbZclLevelClientStopReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg);

static struct ZbZclLevelServerCallbacksT LevelServerCallbacks_1 =
{
  .move_to_level = levelControl_server_1_move_to_level,
  .move = levelControl_server_1_move,
  .step = levelControl_server_1_step,
  .stop = levelControl_server_1_stop,
};

/* USER CODE BEGIN PV */
struct ZbTimerT* joinReqTimer;	//timer for sending join requests
struct ZbTimerT* appTimer;	//timer for RGB application loop
/* USER CODE END PV */
/* Functions Definition ------------------------------------------------------*/

/* OnOff server off 1 command callback */
static enum ZclStatusCodeT onOff_server_1_off(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 0 OnOff server 1 off 1 */
  uint8_t endpoint;

  endpoint = ZbZclClusterGetEndpoint(cluster);
  if (endpoint == SW1_ENDPOINT) 
  {
    APP_DBG("onOff_server_1_off");
    BSP_LED_Off(LED_RED);
    (void)ZbZclAttrIntegerWrite(cluster, ZCL_ONOFF_ATTR_ONOFF, 0);
    if(RGB_params.OnOff != 0)
    {
    	RGB_params.OnOff = 0;
    	ZbTimerStop(appTimer);	//no automatic actions in off state
    	turn_off_LEDs();		//turn off all LEDs
    }
  }
  else 
  {
    /* Unknown endpoint */
    return ZCL_STATUS_FAILURE;
  }
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 0 OnOff server 1 off 1 */
}

/* OnOff server on 1 command callback */
static enum ZclStatusCodeT onOff_server_1_on(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 1 OnOff server 1 on 1 */
  uint8_t endpoint;

  endpoint = ZbZclClusterGetEndpoint(cluster);
  if (endpoint == SW1_ENDPOINT) 
  {
    APP_DBG("onOff_server_1_on");
    BSP_LED_On(LED_RED);
    (void)ZbZclAttrIntegerWrite(cluster, ZCL_ONOFF_ATTR_ONOFF, 1);
    if(RGB_params.OnOff == 0)
    {
    	RGB_params.OnOff = 1;
    	RGB_LED_action(appTimer);
    }
  }
  else 
  {
    /* Unknown endpoint */
    return ZCL_STATUS_FAILURE;
  }
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 1 OnOff server 1 on 1 */
}

/* OnOff server toggle 1 command callback */
static enum ZclStatusCodeT onOff_server_1_toggle(struct ZbZclClusterT *cluster, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 2 OnOff server 1 toggle 1 */
  uint8_t attrVal;

  APP_DBG("onOff_server_1_toggle");
  if (ZbZclAttrRead(cluster, ZCL_ONOFF_ATTR_ONOFF, NULL,
            &attrVal, sizeof(attrVal), false) != ZCL_STATUS_SUCCESS) 
  {
    return ZCL_STATUS_FAILURE;
  }
  
  if (attrVal != 0) 
  {
    return onOff_server_1_off(cluster, srcInfo, arg);
  }
  else
  {
    return onOff_server_1_on(cluster, srcInfo, arg);
  }
  /* USER CODE END 2 OnOff server 1 toggle 1 */
}

/* ColorControl server move_to_color_xy 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_move_to_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveToColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 3 ColorControl server 1 move_to_color_xy 1 */
	APP_DBG("colorControl_server_1_move_to_color_xy (x=%d y=%d)", req->color_x, req->color_y);

	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_CURRENT_X, req->color_x);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_CURRENT_Y, req->color_y);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_REMAINING_TIME, req->transition_time);

	convert_xy_to_RGB(req->color_x, req->color_y, &RGB_params.color);
	/* execute action if the device is on */
	if(RGB_params.OnOff != 0)
	{
		RGB_params.mode = Mode_Static;
		RGB_LED_action(appTimer);
	}

  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 3 ColorControl server 1 move_to_color_xy 1 */
}

/* ColorControl server move_color_xy 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_move_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 4 ColorControl server 1 move_color_xy 1 */
	APP_DBG("colorControl_server_1_move_color_xy");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 4 ColorControl server 1 move_color_xy 1 */
}

/* ColorControl server step_color_xy 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_step_color_xy(struct ZbZclClusterT *cluster, struct ZbZclColorClientStepColorXYReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 5 ColorControl server 1 step_color_xy 1 */
	APP_DBG("colorControl_server_1_step_color_xy");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 5 ColorControl server 1 step_color_xy 1 */
}

/* ColorControl server move_to_color_temp 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_move_to_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveToColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 6 ColorControl server 1 move_to_color_temp 1 */
	APP_DBG("colorControl_server_1_move_to_color_temp (temp=%d)", req->color_temp);
	if((req->color_temp >= ATTR_COLOR_TEMP_BEGIN) && (req->color_temp < ATTR_COLOR_TEMP_BEGIN + No_Of_Modes))
	{
		RGB_params.mode = (req->color_temp - ATTR_COLOR_TEMP_BEGIN);
		RGB_LED_action(appTimer);
	}
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_COLOR_TEMP_MIREDS,req->color_temp);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_REMAINING_TIME, req->transition_time);
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 6 ColorControl server 1 move_to_color_temp 1 */
}

/* ColorControl server color_loop_set 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_color_loop_set(struct ZbZclClusterT *cluster, struct ZbZclColorClientColorLoopSetReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 7 ColorControl server 1 color_loop_set 1 */
	APP_DBG("colorControl_server_1_color_loop_set (flags=%d action=%d dir=%d hue=%d)", req->update_flags, req->action, req->direction, req->start_hue);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_COLOR_LOOP_ACTIVE, (req->action == 0) ? 0 : 1);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_COLOR_LOOP_DIR, req->direction);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_COLOR_LOOP_TIME, req->transition_time);
	(void)ZbZclAttrIntegerWrite(cluster, ZCL_COLOR_ATTR_REMAINING_TIME, 0);
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 7 ColorControl server 1 color_loop_set 1 */
}

/* ColorControl server stop_move_step 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_stop_move_step(struct ZbZclClusterT *cluster, struct ZbZclColorClientStopMoveStepReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 8 ColorControl server 1 stop_move_step 1 */
	APP_DBG("colorControl_server_1_stop_move_step");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 8 ColorControl server 1 stop_move_step 1 */
}

/* ColorControl server move_color_temp 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_move_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientMoveColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 9 ColorControl server 1 move_color_temp 1 */
	  APP_DBG("colorControl_server_1_move_color_temp");
	  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 9 LevelControl server 1 move_to_level 1 */
}

/* ColorControl server step_color_temp 1 command callback */
static enum ZclStatusCodeT colorControl_server_1_step_color_temp(struct ZbZclClusterT *cluster, struct ZbZclColorClientStepColorTempReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 10 ColorControl server 1 step_color_temp 1 */
	APP_DBG("colorControl_server_1_step_color_temp");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 10 ColorControl server 1 step_color_temp 1 */
}

/* LevelControl server move_to_level 1 command callback */
static enum ZclStatusCodeT levelControl_server_1_move_to_level(struct ZbZclClusterT *cluster, struct ZbZclLevelClientMoveToLevelReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 11 LevelControl server 1 move_to_level 1 */
	  uint8_t attrVal;
	  uint8_t endpoint;
	  uint8_t onOffVal;

	  if (ZbZclAttrRead(cluster, ZCL_LEVEL_ATTR_CURRLEVEL, NULL, &attrVal, sizeof(attrVal), false) != ZCL_STATUS_SUCCESS)
	  {
	    return ZCL_STATUS_FAILURE;
	  }

	  endpoint = ZbZclClusterGetEndpoint(cluster);
	  if (endpoint == SW1_ENDPOINT)
	  {
		RGB_params.level = req->level;
		APP_DBG("levelControl_server_1_move_to_level (level=%d, time=%d)", RGB_params.level, req->transition_time);
		(void)ZbZclAttrIntegerWrite(cluster, ZCL_LEVEL_ATTR_CURRLEVEL, RGB_params.level);

		// send data to LEDs if the device is on
		if(RGB_params.OnOff != 0)
		{
			RGB_LED_action(appTimer);
		}

		enum ZclDataTypeT* typePtr = NULL;
		enum ZclStatusCodeT* statusPtr = NULL;;
		onOffVal = ZbZclAttrIntegerRead(zigbee_app_info.onOff_server_1, ZCL_ONOFF_ATTR_ONOFF, typePtr, statusPtr);
		if((req->with_onoff) && (*statusPtr == ZCL_STATUS_SUCCESS))
		{
			//on/off action must be executed
			if((RGB_params.level <= 1) && (onOffVal != 0))
			{
				APP_DBG("levelControl_server_1_move_to_level calling onOff_server_1_off ");
				onOff_server_1_off(zigbee_app_info.onOff_server_1, srcInfo, arg);
			}
			if((RGB_params.level > 1) && (onOffVal == 0))
			{
				APP_DBG("levelControl_server_1_move_to_level calling onOff_server_1_on ");
				onOff_server_1_on(zigbee_app_info.onOff_server_1, srcInfo, arg);
			}
		}
	  }
	  else
	  {
	    /* Unknown endpoint */
	    return ZCL_STATUS_FAILURE;
	  }
	  return ZCL_STATUS_SUCCESS;
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 11 LevelControl server 1 move_to_level 1 */
}

/* LevelControl server move 1 command callback */
static enum ZclStatusCodeT levelControl_server_1_move(struct ZbZclClusterT *cluster, struct ZbZclLevelClientMoveReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 12 LevelControl server 1 move 1 */
	APP_DBG("levelControl_server_1_move");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 12 LevelControl server 1 move 1 */
}

/* LevelControl server step 1 command callback */
static enum ZclStatusCodeT levelControl_server_1_step(struct ZbZclClusterT *cluster, struct ZbZclLevelClientStepReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 13 LevelControl server 1 step 1 */
	APP_DBG("levelControl_server_1_step");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 13 LevelControl server 1 step 1 */
}

/* LevelControl server stop 1 command callback */
static enum ZclStatusCodeT levelControl_server_1_stop(struct ZbZclClusterT *cluster, struct ZbZclLevelClientStopReqT *req, struct ZbZclAddrInfoT *srcInfo, void *arg)
{
  /* USER CODE BEGIN 14 LevelControl server 1 stop 1 */
	APP_DBG("levelControl_server_1_stop");
  return ZCL_STATUS_SUCCESS;
  /* USER CODE END 14 LevelControl server 1 stop 1 */
}

/**
 * @brief  Zigbee application initialization
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_Init(void)
{
  SHCI_CmdStatus_t ZigbeeInitStatus;

  APP_DBG("APP_ZIGBEE_Init");

  /* Check the compatibility with the Coprocessor Wireless Firmware loaded */
  APP_ZIGBEE_CheckWirelessFirmwareInfo();

  /* Register cmdbuffer */
  APP_ZIGBEE_RegisterCmdBuffer(&ZigbeeOtCmdBuffer);

  /* Init config buffer and call TL_ZIGBEE_Init */
  APP_ZIGBEE_TL_INIT();

  /* Register task */
  /* Create the different tasks */
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessNotifyM0ToM4);
  UTIL_SEQ_RegTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, UTIL_SEQ_RFU, APP_ZIGBEE_ProcessRequestM0ToM4);

  /* Task associated with network creation process */
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, UTIL_SEQ_RFU, APP_ZIGBEE_NwkForm);

  /* USER CODE BEGIN APP_ZIGBEE_INIT */
  /* USER CODE END APP_ZIGBEE_INIT */

  /* Start the Zigbee on the CPU2 side */
  ZigbeeInitStatus = SHCI_C2_ZIGBEE_Init();
  /* Prevent unused argument(s) compilation warning */
  UNUSED(ZigbeeInitStatus);

  /* Initialize Zigbee stack layers */
  APP_ZIGBEE_StackLayersInit();

}

/**
 * @brief  Initialize Zigbee stack layers
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_StackLayersInit(void)
{
  APP_DBG("APP_ZIGBEE_StackLayersInit");

  zigbee_app_info.zb = ZbInit(0U, NULL, NULL);
  assert(zigbee_app_info.zb != NULL);

  /* Create the endpoint and cluster(s) */
  APP_ZIGBEE_ConfigEndpoints();

  /* USER CODE BEGIN APP_ZIGBEE_StackLayersInit */
  BSP_LED_Off(LED_RED);
  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_BLUE);
  /* USER CODE END APP_ZIGBEE_StackLayersInit */

  /* Configure the joining parameters */
  zigbee_app_info.join_status = (enum ZbStatusCodeT) 0x01; /* init to error status */
  zigbee_app_info.join_delay = HAL_GetTick(); /* now */
  zigbee_app_info.startupControl = ZbStartTypeJoin;

  /* Initialization Complete */
  zigbee_app_info.has_init = true;

  /* run the task */
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
}

/**
 * @brief  Configure Zigbee application endpoints
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigEndpoints(void)
{
  struct ZbApsmeAddEndpointReqT req;
  struct ZbApsmeAddEndpointConfT conf;

  memset(&req, 0, sizeof(req));

  /* Endpoint: SW1_ENDPOINT */
  req.profileId = ZCL_PROFILE_HOME_AUTOMATION;
  req.deviceId = ZCL_DEVICE_COLOR_DIMMABLE_LIGHT;
  req.endpoint = SW1_ENDPOINT;
  ZbZclAddEndpoint(zigbee_app_info.zb, &req, &conf);
  assert(conf.status == ZB_STATUS_SUCCESS);

  /* Identify server */
  zigbee_app_info.identify_server_1 = ZbZclIdentifyServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, NULL);
  assert(zigbee_app_info.identify_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.identify_server_1);
  /* Groups server */
  zigbee_app_info.groups_server_1 = ZbZclGroupsServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT);
  assert(zigbee_app_info.groups_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.groups_server_1);
  /* Scenes server */
  zigbee_app_info.scenes_server_1 = ZbZclScenesServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, ZCL_SCENES_MAX_SCENES_1);
  assert(zigbee_app_info.scenes_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.scenes_server_1);
  /* OnOff server */
  zigbee_app_info.onOff_server_1 = ZbZclOnOffServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, &OnOffServerCallbacks_1, NULL);
  assert(zigbee_app_info.onOff_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.onOff_server_1);
  /* ColorControl server */
  struct ZbColorClusterConfig colorServerConfig_1 = {
    .callbacks = ColorServerCallbacks_1,
    /* Please complete the other attributes according to your application:
     *          .capabilities           //uint8_t (e.g. ZCL_COLOR_CAP_HS)
     *          .enhanced_supported     //bool
     */
    /* USER CODE BEGIN Color Server Config (endpoint1) */
	.capabilities = ZCL_COLOR_CAP_XY |
						ZCL_COLOR_CAP_COLOR_TEMP |
						ZCL_COLOR_CAP_COLOR_LOOP
    /* USER CODE END Color Server Config (endpoint1) */
  };
  zigbee_app_info.colorControl_server_1 = ZbZclColorServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, zigbee_app_info.onOff_server_1, NULL, 0, &colorServerConfig_1, NULL);
  assert(zigbee_app_info.colorControl_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.colorControl_server_1);
  /* LevelControl server */
  zigbee_app_info.levelControl_server_1 = ZbZclLevelServerAlloc(zigbee_app_info.zb, SW1_ENDPOINT, zigbee_app_info.onOff_server_1, &LevelServerCallbacks_1, NULL);
  assert(zigbee_app_info.levelControl_server_1 != NULL);
  ZbZclClusterEndpointRegister(zigbee_app_info.levelControl_server_1);

  /* USER CODE BEGIN CONFIG_ENDPOINT */
  APP_DBG("adding cluster attributes");

  static const struct ZbZclAttrT identifyControl_attr_list[] =		/* MS add optional attributes of identify cluster */
  {
	{
	ZCL_IDENTIFY_ATTR_TIME, ZCL_DATATYPE_UNSIGNED_16BIT,
	ZCL_ATTR_FLAG_WRITABLE, 0, NULL, {0, 0}, {0, 0}
	}
  };
  ZbZclAttrAppendList( zigbee_app_info.identify_server_1, identifyControl_attr_list, ZCL_ATTR_LIST_LEN(identifyControl_attr_list));

  static const struct ZbZclAttrT groupsControl_attr_list[] =		/* MS add optional attributes of groups cluster */
  {
	{
	ZCL_GROUPS_ATTR_NAME_SUPPORT, ZCL_DATATYPE_BITMAP_8BIT,
	ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	}
  };
  ZbZclAttrAppendList( zigbee_app_info.groups_server_1, groupsControl_attr_list, ZCL_ATTR_LIST_LEN(groupsControl_attr_list));
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.groups_server_1, ZCL_GROUPS_ATTR_NAME_SUPPORT, 0);

  static const struct ZbZclAttrT scenesControl_attr_list[] =		/* MS add optional attributes of scenes cluster */
  {
	{
	ZCL_SCENES_ATTR_SCENE_COUNT, ZCL_DATATYPE_UNSIGNED_8BIT,
	ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_SCENES_ATTR_CURRENT_SCENE, ZCL_DATATYPE_UNSIGNED_8BIT,
	ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_SCENES_ATTR_CURRENT_GROUP, ZCL_DATATYPE_UNSIGNED_16BIT,
	ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_SCENES_ATTR_SCENE_VALID, ZCL_DATATYPE_BOOLEAN,
	ZCL_ATTR_FLAG_REPORTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_SCENES_ATTR_NAME_SUPPORT, ZCL_DATATYPE_BITMAP_8BIT,
	ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	}
  };
  ZbZclAttrAppendList( zigbee_app_info.scenes_server_1, scenesControl_attr_list, ZCL_ATTR_LIST_LEN(scenesControl_attr_list));
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.scenes_server_1, ZCL_SCENES_ATTR_SCENE_COUNT, 0);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.scenes_server_1, ZCL_SCENES_ATTR_CURRENT_SCENE, 0);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.scenes_server_1, ZCL_SCENES_ATTR_CURRENT_GROUP, 0);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.scenes_server_1, ZCL_SCENES_ATTR_SCENE_VALID, 0);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.scenes_server_1, ZCL_SCENES_ATTR_NAME_SUPPORT, 0);

  static const struct ZbZclAttrT colorControl_attr_list[] =		/* MS add optional attributes of color control cluster */
  {
	{
			ZCL_COLOR_ATTR_REMAINING_TIME, ZCL_DATATYPE_UNSIGNED_16BIT,
		ZCL_ATTR_FLAG_REPORTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_CURRENT_X, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_CURRENT_Y, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_TEMP_MIREDS, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
		ZCL_COLOR_ATTR_COLOR_MODE, ZCL_DATATYPE_ENUMERATION_8BIT,
		ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
		ZCL_COLOR_ATTR_ENH_COLOR_MODE, ZCL_DATATYPE_ENUMERATION_8BIT,
		ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_LOOP_ACTIVE, ZCL_DATATYPE_UNSIGNED_8BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_LOOP_DIR, ZCL_DATATYPE_UNSIGNED_8BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_LOOP_TIME, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_TEMP_MIN, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_COLOR_TEMP_MAX, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
			ZCL_COLOR_ATTR_STARTUP_COLOR_TEMP, ZCL_DATATYPE_UNSIGNED_16BIT,
			ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_REPORTABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
  };
  ZbZclAttrAppendList( zigbee_app_info.colorControl_server_1, colorControl_attr_list, ZCL_ATTR_LIST_LEN(colorControl_attr_list));
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_COLOR_MODE, ZCL_COLOR_MODE_XY);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_ENH_COLOR_MODE, ZCL_COLOR_ENH_MODE_CURR_XY);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_COLOR_TEMP_MIN, ATTR_COLOR_TEMP_BEGIN);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_COLOR_TEMP_MAX, ATTR_COLOR_TEMP_BEGIN + No_Of_Modes - 1);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_STARTUP_COLOR_TEMP, ATTR_COLOR_TEMP_BEGIN);
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.colorControl_server_1, ZCL_COLOR_ATTR_COLOR_TEMP_MIREDS,ATTR_COLOR_TEMP_BEGIN);

  static const struct ZbZclAttrT levelControl_attr_list[] =		/* MS add optional attributes of level control cluster */
  {
	{
	ZCL_LEVEL_ATTR_ONLEVEL, ZCL_DATATYPE_UNSIGNED_8BIT,
	ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_LEVEL_ATTR_ONOFF_TRANS_TIME, ZCL_DATATYPE_UNSIGNED_16BIT,
	ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	},
	{
	ZCL_LEVEL_ATTR_STARTUP_CURRLEVEL, ZCL_DATATYPE_UNSIGNED_8BIT,
	ZCL_ATTR_FLAG_WRITABLE | ZCL_ATTR_FLAG_PERSISTABLE, 0, NULL, {0, 0}, {0, 0}
	}
  };
  ZbZclAttrAppendList( zigbee_app_info.levelControl_server_1, levelControl_attr_list, ZCL_ATTR_LIST_LEN(levelControl_attr_list));
  (void)ZbZclAttrIntegerWrite( zigbee_app_info.levelControl_server_1, ZCL_LEVEL_ATTR_ONOFF_TRANS_TIME, ZCL_LEVEL_ATTR_ONOFF_TRANS_TIME_DEFAULT);

  (void)ZbZclAttrIntegerWrite(zigbee_app_info.onOff_server_1, ZCL_ONOFF_ATTR_ONOFF, RGB_params.OnOff);

  joinReqTimer = ZbTimerAlloc(zigbee_app_info.zb, APP_ZIGBEE_JoinReq, NULL);
  ZbTimerReset(joinReqTimer, 10 *1000);

  appTimer = ZbTimerAlloc(zigbee_app_info.zb, RGB_timer_callback, NULL);

  /* USER CODE END CONFIG_ENDPOINT */
}

/**
 * @brief  Handle Zigbee network forming and joining
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_NwkForm(void)
{
  if ((zigbee_app_info.join_status != ZB_STATUS_SUCCESS) && (HAL_GetTick() >= zigbee_app_info.join_delay))
  {
    struct ZbStartupT config;
    enum ZbStatusCodeT status;

    /* Configure Zigbee Logging */
    ZbSetLogging(zigbee_app_info.zb, ZB_LOG_MASK_LEVEL_5, NULL);

    /* Attempt to join a zigbee network */
    ZbStartupConfigGetProDefaults(&config);

    /* Set the distributed network */
    APP_DBG("Network config : APP_STARTUP_DISTRIBUTED");
    config.startupControl = zigbee_app_info.startupControl;

    /* Set the TC address to be distributed. */
    config.security.trustCenterAddress = ZB_DISTRIBUTED_TC_ADDR;

    /* Using the Uncertified Distributed Global Key (d0:d1:d2:d3:d4:d5:d6:d7:d8:d9:da:db:dc:dd:de:df) */
    memcpy(config.security.distributedGlobalKey, sec_key_distrib_uncert, ZB_SEC_KEYSIZE);

    config.channelList.count = 1;
    config.channelList.list[0].page = 0;
    config.channelList.list[0].channelMask = 1 << CHANNEL; /*Channel in use */

    /* Using ZbStartupWait (blocking) */
    status = ZbStartupWait(zigbee_app_info.zb, &config);

    APP_DBG("ZbStartup Callback (status = 0x%02x)", status);
    zigbee_app_info.join_status = status;

    if (status == ZB_STATUS_SUCCESS)
    {
      zigbee_app_info.join_delay = 0U;
      zigbee_app_info.init_after_join = true;
      APP_DBG("Startup done !\n");
      /* USER CODE BEGIN 15 */
      BSP_LED_On(LED_BLUE);
      /* USER CODE END 15 */
    }
    else
    {
      zigbee_app_info.startupControl = ZbStartTypeForm;
      APP_DBG("Startup failed, attempting again after a short delay (%d ms)", APP_ZIGBEE_STARTUP_FAIL_DELAY);
      zigbee_app_info.join_delay = HAL_GetTick() + APP_ZIGBEE_STARTUP_FAIL_DELAY;
      /* USER CODE BEGIN 16 */

      /* USER CODE END 16 */
    }
  }

  /* If Network forming/joining was not successful reschedule the current task to retry the process */
  if (zigbee_app_info.join_status != ZB_STATUS_SUCCESS)
  {
    UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_NETWORK_FORM, CFG_SCH_PRIO_0);
  }
  /* USER CODE BEGIN NW_FORM */
  else
  {
    zigbee_app_info.init_after_join = false;

    /* Assign ourselves to the group addresses */
    APP_ZIGBEE_ConfigGroupAddr();

    /* Since we're using group addressing (broadcast), shorten the broadcast timeout */
    uint32_t bcast_timeout = 3;
    ZbNwkSet(zigbee_app_info.zb, ZB_NWK_NIB_ID_NetworkBroadcastDeliveryTime, &bcast_timeout, sizeof(bcast_timeout));
  }
  /* USER CODE END NW_FORM */
}

/*************************************************************
 * ZbStartupWait Blocking Call
 *************************************************************/
struct ZbStartupWaitInfo
{
  bool active;
  enum ZbStatusCodeT status;
};

static void ZbStartupWaitCb(enum ZbStatusCodeT status, void *cb_arg)
{
  struct ZbStartupWaitInfo *info = cb_arg;

  info->status = status;
  info->active = false;
  UTIL_SEQ_SetEvt(EVENT_ZIGBEE_STARTUP_ENDED);
}

enum ZbStatusCodeT ZbStartupWait(struct ZigBeeT *zb, struct ZbStartupT *config)
{
  struct ZbStartupWaitInfo *info;
  enum ZbStatusCodeT status;

  info = malloc(sizeof(struct ZbStartupWaitInfo));
  if (info == NULL)
  {
    return ZB_STATUS_ALLOC_FAIL;
  }
  memset(info, 0, sizeof(struct ZbStartupWaitInfo));

  info->active = true;
  status = ZbStartup(zb, config, ZbStartupWaitCb, info);
  if (status != ZB_STATUS_SUCCESS)
  {
    free(info);
    return status;
  }

  UTIL_SEQ_WaitEvt(EVENT_ZIGBEE_STARTUP_ENDED);
  status = info->status;
  free(info);
  return status;
}

/**
 * @brief  Trace the error or the warning reported.
 * @param  ErrId :
 * @param  ErrCode
 * @retval None
 */
void APP_ZIGBEE_Error(uint32_t ErrId, uint32_t ErrCode)
{
  switch (ErrId)
  {
    default:
      APP_ZIGBEE_TraceError("ERROR Unknown ", 0);
      break;
  }
}

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/**
 * @brief  Warn the user that an error has occurred.
 *
 * @param  pMess  : Message associated to the error.
 * @param  ErrCode: Error code associated to the module (Zigbee or other module if any)
 * @retval None
 */
static void APP_ZIGBEE_TraceError(const char *pMess, uint32_t ErrCode)
{
  APP_DBG("**** Fatal error = %s (Err = %d)", pMess, ErrCode);
  /* USER CODE BEGIN TRACE_ERROR */
  while (1U == 1U) 
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(500U);
    BSP_LED_Toggle(LED2);
    HAL_Delay(500U);
    BSP_LED_Toggle(LED3);
    HAL_Delay(500U);
  }
  /* USER CODE END TRACE_ERROR */

}

/**
 * @brief Check if the Coprocessor Wireless Firmware loaded supports Zigbee
 *        and display associated information
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_CheckWirelessFirmwareInfo(void)
{
  WirelessFwInfo_t wireless_info_instance;
  WirelessFwInfo_t *p_wireless_info = &wireless_info_instance;

  if (SHCI_GetWirelessFwInfo(p_wireless_info) != SHCI_Success)
  {
    APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
  }
  else
  {
    APP_DBG("**********************************************************");
    APP_DBG("WIRELESS COPROCESSOR FW:");
    /* Print version */
    APP_DBG("VERSION ID = %d.%d.%d", p_wireless_info->VersionMajor, p_wireless_info->VersionMinor, p_wireless_info->VersionSub);

    switch (p_wireless_info->StackType)
    {
      case INFO_STACK_TYPE_ZIGBEE_FFD:
        APP_DBG("FW Type : FFD Zigbee stack");
        break;

      case INFO_STACK_TYPE_ZIGBEE_RFD:
        APP_DBG("FW Type : RFD Zigbee stack");
        break;

      default:
        /* No Zigbee device supported ! */
        APP_ZIGBEE_Error((uint32_t)ERR_ZIGBEE_CHECK_WIRELESS, (uint32_t)ERR_INTERFACE_FATAL);
        break;
    }

    /* print the application name */
    char *__PathProject__ = (strstr(__FILE__, "Zigbee") ? strstr(__FILE__, "Zigbee") + 7 : __FILE__);
    char *pdel = NULL;
    if((strchr(__FILE__, '/')) == NULL)
    {
      pdel = strchr(__PathProject__, '\\');
    }
    else
    {
      pdel = strchr(__PathProject__, '/');
    }

    int index = (int)(pdel - __PathProject__);
    APP_DBG("Application flashed: %*.*s", index, index, __PathProject__);

    /* print channel */
    APP_DBG("Channel used: %d", CHANNEL);
    /* print Link Key */
    APP_DBG("Link Key: %.16s", sec_key_ha);
    /* print Link Key value hex */
    char Z09_LL_string[ZB_SEC_KEYSIZE*3+1];
    Z09_LL_string[0] = 0;
    for (int str_index = 0; str_index < ZB_SEC_KEYSIZE; str_index++)
    {
      sprintf(&Z09_LL_string[str_index*3], "%02x ", sec_key_ha[str_index]);
    }

    APP_DBG("Link Key value: %s", Z09_LL_string);
    /* print clusters allocated */
    APP_DBG("Clusters allocated are:");
    APP_DBG("identify Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("groups Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("scenes Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("onOff Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("colorControl Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("levelControl Server on Endpoint %d", SW1_ENDPOINT);
    APP_DBG("**********************************************************");
  }
}

/*************************************************************
 *
 * WRAP FUNCTIONS
 *
 *************************************************************/

void APP_ZIGBEE_RegisterCmdBuffer(TL_CmdPacket_t *p_buffer)
{
  p_ZIGBEE_otcmdbuffer = p_buffer;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_OTCmdRspPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)((TL_EvtPacket_t *)p_ZIGBEE_otcmdbuffer)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_NotificationPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_notif_M0_to_M4)->evtserial.evt.payload;
}

Zigbee_Cmd_Request_t * ZIGBEE_Get_M0RequestPayloadBuffer(void)
{
  return (Zigbee_Cmd_Request_t *)(p_ZIGBEE_request_M0_to_M4)->evtserial.evt.payload;
}

/**
 * @brief  This function is used to transfer the commands from the M4 to the M0.
 *
 * @param   None
 * @return  None
 */
void ZIGBEE_CmdTransfer(void)
{
  Zigbee_Cmd_Request_t *cmd_req = (Zigbee_Cmd_Request_t *)p_ZIGBEE_otcmdbuffer->cmdserial.cmd.payload;

  /* Zigbee OT command cmdcode range 0x280 .. 0x3DF = 352 */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.cmdcode = 0x280U;
  /* Size = otCmdBuffer->Size (Number of OT cmd arguments : 1 arg = 32bits so multiply by 4 to get size in bytes)
   * + ID (4 bytes) + Size (4 bytes) */
  p_ZIGBEE_otcmdbuffer->cmdserial.cmd.plen = 8U + (cmd_req->Size * 4U);

  TL_ZIGBEE_SendM4RequestToM0();

  /* Wait completion of cmd */
  Wait_Getting_Ack_From_M0();
}

/**
 * @brief  This function is called when the M0+ acknowledge the fact that it has received a Cmd
 *
 *
 * @param   Otbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_CmdEvtReceived(TL_EvtPacket_t *Otbuffer)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Otbuffer);

  Receive_Ack_From_M0();
}

/**
 * @brief  This function is called when notification from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_NotReceived(TL_EvtPacket_t *Notbuffer)
{
  p_ZIGBEE_notif_M0_to_M4 = Notbuffer;

  Receive_Notification_From_M0();
}

/**
 * @brief  This function is called before sending any ot command to the M0
 *         core. The purpose of this function is to be able to check if
 *         there are no notifications coming from the M0 core which are
 *         pending before sending a new ot command.
 * @param  None
 * @retval None
 */
void Pre_ZigbeeCmdProcessing(void)
{
  UTIL_SEQ_WaitEvt(EVENT_SYNCHRO_BYPASS_IDLE);
}

/**
 * @brief  This function waits for getting an acknowledgment from the M0.
 *
 * @param  None
 * @retval None
 */
static void Wait_Getting_Ack_From_M0(void)
{
  UTIL_SEQ_WaitEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive an acknowledgment from the M0+ core.
 *         Each command send by the M4 to the M0 are acknowledged.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Ack_From_M0(void)
{
  UTIL_SEQ_SetEvt(EVENT_ACK_FROM_M0_EVT);
}

/**
 * @brief  Receive a notification from the M0+ through the IPCC.
 *         This function is called under interrupt.
 * @param  None
 * @retval None
 */
static void Receive_Notification_From_M0(void)
{
  CptReceiveNotifyFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_NOTIFY_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief  This function is called when a request from M0+ is received.
 *
 * @param   Notbuffer : a pointer to TL_EvtPacket_t
 * @return  None
 */
void TL_ZIGBEE_M0RequestReceived(TL_EvtPacket_t *Reqbuffer)
{
  p_ZIGBEE_request_M0_to_M4 = Reqbuffer;

  CptReceiveRequestFromM0++;
  UTIL_SEQ_SetTask(1U << (uint32_t)CFG_TASK_REQUEST_FROM_M0_TO_M4, CFG_SCH_PRIO_0);
}

/**
 * @brief Perform initialization of TL for Zigbee.
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_TL_INIT(void)
{
  ZigbeeConfigBuffer.p_ZigbeeOtCmdRspBuffer = (uint8_t *)&ZigbeeOtCmdBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotAckBuffer = (uint8_t *)ZigbeeNotifRspEvtBuffer;
  ZigbeeConfigBuffer.p_ZigbeeNotifRequestBuffer = (uint8_t *)ZigbeeNotifRequestBuffer;
  TL_ZIGBEE_Init(&ZigbeeConfigBuffer);
}

/**
 * @brief Process the messages coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessNotifyM0ToM4(void)
{
  if (CptReceiveNotifyFromM0 != 0)
  {
    /* Reset counter */
    CptReceiveNotifyFromM0 = 0;
    Zigbee_CallBackProcessing();
  }
}

/**
 * @brief Process the requests coming from the M0.
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ProcessRequestM0ToM4(void)
{
  if (CptReceiveRequestFromM0 != 0)
  {
    CptReceiveRequestFromM0 = 0;
    Zigbee_M0RequestProcessing();
  }
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/**
 * @brief  Set group addressing mode
 * @param  None
 * @retval None
 */
static void APP_ZIGBEE_ConfigGroupAddr(void)
{
  struct ZbApsmeAddGroupReqT req;
  struct ZbApsmeAddGroupConfT conf;

  memset(&req, 0, sizeof(req));
  req.endpt = SW1_ENDPOINT;
  req.groupAddr = SW1_GROUP_ADDR;
  ZbApsmeAddGroupReq(zigbee_app_info.zb, &req, &conf);

}

/* ZbZdoPermitJoinReq message has to be sent to allow permit join for an extra amount of time */
static void APP_ZIGBEE_JoinReq(struct ZigBeeT* zb, void* arg)
{
	struct ZbZdoPermitJoinReqT req;
	memset(&req, 0, sizeof(req));

	req.destAddr=0xFFFC;
	req.tcSignificance = true;
	req.duration = 0xFE;
	enum ZbStatusCodeT status = ZbZdoPermitJoinReq(zb,&req,NULL,NULL);
	APP_DBG("ZbZdoPermitJoinReq call (status = 0x%02x)", status);

	(void)ZbTimerReset(joinReqTimer, 60 * 1000);
}

static void RGB_timer_callback(struct ZigBeeT* zb, void* arg)
{
	RGB_LED_action(appTimer);
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
