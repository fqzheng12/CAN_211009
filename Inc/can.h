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

/* USER CODE BEGIN Private defines *///�û����뿪ʼ


/**
 * @brief ��ʼ��ͨ����ز���, ��ʼ��CANӲ�����ã�
 *  ������Ϊ500K
 *  �ط�����Ϊ�Զ��ط�
 *  bus-off�ָ�����Ϊ�Զ��ָ�
 * @param  can_frame_filter_cfg �������ñ�����CAN ID��MASK
 * @param  filter_num      �������ñ�Ĵ�С
 * @return int8_t 
         0����ʼ���ɹ�
        -1����ʼ��ʧ��
 */
extern int8_t hcan_driver_hw_init(hcan_frame_filter_t *can_filter, uint8_t filter_num);



/**
 * @brief �����Ա��ķ��ͣ� ���ڶ�֡���ķ���
 * @param  frame    CAN֡
 * @param  fifo     ����CAN֡ʹ�õķ������䣡����
 *                  ��֡������Ҫʹ��ר�÷������䣬���������������֡���ͣ�
 *                  ����֧��������֡���͵��豸������������Ҫ�뷢�ͻ���󶨣�
 *                  �Է�ֹ�������߸��ع�����֡��������
 * @return int8_t 
        0�����ͳɹ�
        1������ʧ��
 */
extern int8_t hcan_driver_send_multi_frame(uint32_t id, uint8_t *data, int fifo);




/**
 * @brief ���ٱ��ķ��ͣ� ���ڵ�֡���ķ���
 *          ��֡����ʹ�ó���֡���͵���������������䣬�������֡������á�
 * @param  frame            CAN֡
 * @return int8_t 
        0: ���ͳɹ�
        1: ����ʧ��
 */
extern int8_t hcan_driver_send_single_frame(uint32_t id, uint8_t *data, uint8_t len);



/**
 * @brief ��ѯ����״̬������ѯbus-off״̬
 * @return int8_t 
        0����������
        -1������bus-off
 */
extern int8_t hcan_driver_query_bus_status(void);




/**
 * @brief ��λ���ϵ�CAN�����շ���
 * @return int8_t 
        0:�ɹ�
        -1:ʧ��
 */
extern int8_t hcan_driver_fault_reset(void);







/* USER CODE END Private defines *///�û��������






void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
