/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "hcan.h"
#include "usart.h"


/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines *///用户代码开始


/**
 * @brief 初始化通信相关参数, 初始化CAN硬件配置：
 *  波特率为500K
 *  重发配置为自动重发
 *  bus-off恢复配置为自动恢复
 * @param  can_frame_filter_cfg 过滤配置表：包含CAN ID和MASK
 * @param  filter_num      过滤配置表的大小
 * @return int8_t 
         0：初始化成功
        -1：初始化失败
 */
extern int8_t hcan_driver_hw_init(hcan_frame_filter_t *can_filter, uint8_t filter_num);



/**
 * @brief 周期性报文发送， 用于多帧报文发送
 * @param  frame    CAN帧
 * @param  fifo     发送CAN帧使用的发送邮箱！！！
 *                  多帧发送需要使用专用发送邮箱，即此邮箱仅用作多帧发送，
 *                  对于支持两个多帧发送的设备，发送邮箱需要与发送缓存绑定，
 *                  以防止由于总线负载过大导致帧发送乱序。
 * @return int8_t 
        0：发送成功
        1：发送失败
 */
extern int8_t hcan_driver_send_multi_frame(uint32_t id, uint8_t *data, int fifo);




/**
 * @brief 快速报文发送， 用于单帧报文发送
 *          单帧发送使用除多帧发送的邮箱外的其他邮箱，不能与多帧邮箱混用。
 * @param  frame            CAN帧
 * @return int8_t 
        0: 发送成功
        1: 发送失败
 */
extern int8_t hcan_driver_send_single_frame(uint32_t id, uint8_t *data, uint8_t len);



/**
 * @brief 查询总线状态，仅查询bus-off状态
 * @return int8_t 
        0：总线正常
        -1：总线bus-off
 */
extern int8_t hcan_driver_query_bus_status(void);




/**
 * @brief 复位故障的CAN总线收发器
 * @return int8_t 
        0:成功
        -1:失败
 */
extern int8_t hcan_driver_fault_reset(void);







/* USER CODE END Private defines *///用户代码结束






void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
