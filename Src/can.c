/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
//
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
	hcan_frame_filter_t hcan1_fliter_1;
	hcan_frame_filter_t hcan1_fliter_2;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
	hcan1_fliter_1.can_id=0x02<<6;//只有控制器id的02可以收
	hcan1_fliter_1.mask=0xFC0;
	hcan_driver_hw_init(&hcan1_fliter_1,0);
	hcan1_fliter_2.can_id=0x3F<<6;//广播消息也要收，过滤之间是或的关系
	hcan1_fliter_2.mask=0xFC0;
	hcan_init(&hcan1_fliter_2,1);//这个地方初始化了单多帧收发和链表
  HAL_CAN_Start(&hcan1);//启动CAN
	//激活fifo0的数据接收

  if(HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK)
{
    Error_Handler();
}
  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
    
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
   



/**
 * @brief 初始化通信相关参数, 初始化CAN硬件配置：//次函数在改驱动内只用来设置过滤
 *  波特率为500K
 *  重发配置为自动重发
 *  bus-off恢复配置为自动恢复
 * @param  can_frame_filter_cfg 过滤配置表：包含CAN ID和MASK
 * @param  filter_num      过滤配置表的大小
 * @return int8_t 
         0：初始化成功
        -1：初始化失败
 */

 int8_t hcan_driver_hw_init(hcan_frame_filter_t *can_filter,uint8_t filter_num)
{
	CAN_FilterTypeDef CAN_FilterType;
// 过滤器的id,STM32F072RBTx提供了14个过滤器所以id可以配置成0~13
  CAN_FilterType.FilterBank=filter_num;

// 设置位宽为32位
  CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;

// 设置为掩码模式
  CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;

// 设置前两个字节的STDID[10:3]、STDID[2:0]、EXID[17:13]
  CAN_FilterType.FilterIdHigh=((can_filter->can_id<<3) >>16) &0xffff;
  
// 设置后两个字节的EXID[12:5]、EXID[4:0]、IDE、RTR、预留的一个0
  CAN_FilterType.FilterIdLow=(can_filter->can_id<<3) | CAN_ID_EXT;
  
// 设置掩码前两个字节,左移3位再或CAN_ID_EXT是因为最后的三位并不是ID，而是IDE、RTR和预留的0
  CAN_FilterType.FilterMaskIdHigh=((can_filter->mask<<3|CAN_ID_EXT)>>16)&0xffff;

// 设置掩码后两个字节,左移3位再或CAN_ID_EXT是因为最后的三位并不是ID，而是IDE、RTR和预留的0
  CAN_FilterType.FilterMaskIdLow=(can_filter->mask<<3|CAN_ID_EXT)&0xffff;

// 将消息放到FIFO0这个队列里
  CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;

// 激活过滤器
  CAN_FilterType.FilterActivation=ENABLE;

// 设置过滤器
  if(HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterType)!=HAL_OK)
  {
    Error_Handler();
  }

	return 0;
}



//中断接收的回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
//    printf("HAL_CAN_RxFifo0MsgPendingCallback\r\n");
    CAN_RxHeaderTypeDef CAN_RxHeader;
    HAL_StatusTypeDef HAL_Retval;
    uint8_t Rx_Data[8];
    uint8_t Data_Len = 0;
    uint32_t ID = 0;
    uint8_t i;
    HAL_Retval = HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CAN_RxHeader,Rx_Data);
    if(HAL_Retval == HAL_OK)
    {
        Data_Len = CAN_RxHeader.DLC;
        if(CAN_RxHeader.IDE)
        {
            ID = CAN_RxHeader.ExtId;
        }
        else
        {
            ID = CAN_RxHeader.StdId;
        }
//        printf("id:%x\r\n",ID);
//        printf("Data_Len:%d\r\n",Data_Len);
//        for(i=0;i<Data_Len;i++)
//        {
//            printf("Rx_Data[%d]=%x\r\n",i,Rx_Data[i]);  
//        }
//		hcan_driver_send_single_frame(ID,Rx_Data,Data_Len);
		hcan_hw_frame_recv(ID,Rx_Data,Data_Len);
    }
}



/**
 * @brief 快速报文发送， 用于单帧报文发送
 *          单帧发送使用除多帧发送的邮箱外的其他邮箱，不能与多帧邮箱混用。
 * @param  frame            CAN帧
 * @return int8_t 
        0: 发送成功
        1: 发送失败
 */
 int8_t hcan_driver_send_single_frame(uint32_t id, uint8_t *data, uint8_t len)
{	  
	CAN_TxHeaderTypeDef TxHeader;
	 
	TxHeader.ExtId = id;
	TxHeader.DLC = len;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	  if((hcan1.Instance->TSR & CAN_TSR_TME0) != 0U)//如果邮箱0空的，用来发单帧
		{
			if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX0)!=0)//使用0邮箱用来发送单帧{
			{	
				if((hcan1.Instance->TSR & CAN_TSR_TME1) != 0U)//邮箱0不空，但是邮箱1空，用邮箱1发单帧
				{
					if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX1)!=0)
					return 1;
					printf("控制器-邮箱1发送成功\r\n");
				}
			}
//			printf("控制器-邮箱0送成功\r\n");
			return 0;
		}
		return 0;
}


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
 int8_t hcan_driver_send_multi_frame(uint32_t id, uint8_t *data, int fifo)
{
	CAN_TxHeaderTypeDef TxHeader;
	 
	TxHeader.ExtId = id;
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	  if((hcan1.Instance->TSR & CAN_TSR_TME2) != 0U)//如果邮箱2空的，用来发多帧
	{
		if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX2)!=0)//默认用第2邮箱发多帧
			return 1;
		else
			printf("控制器-邮箱2送成功\r\n");
			return 0;
	}
	return 0;	
}


/**
 * @brief 查询总线状态，仅查询bus-off状态
 * @return int8_t 
        0：总线正常
        -1：总线bus-off
 */
 int8_t hcan_driver_query_bus_status(void)
{
	if(HAL_CAN_GetError(&hcan1)==HAL_CAN_ERROR_BOF)
	{
		printf("CAN BUS OFF"); 
		return -1;
	}
	
	else
		return 0;
}	 



/**
 * @brief 复位故障的CAN总线收发器
 * @return int8_t 
        0:成功
        -1:失败
 */
 int8_t hcan_driver_fault_reset(void)
{
	MX_CAN1_Init();	
	HAL_CAN_ResetError(&hcan1);
	if(HAL_CAN_GetState(&hcan1)!=HAL_CAN_STATE_ERROR)
		return 0;
	else
		return -1;
}





/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
