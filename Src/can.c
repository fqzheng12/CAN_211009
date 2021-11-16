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
	hcan1_fliter_1.can_id=0x02<<6;//ֻ�п�����id��02������
	hcan1_fliter_1.mask=0xFC0;
	hcan_driver_hw_init(&hcan1_fliter_1,0);
	hcan1_fliter_2.can_id=0x3F<<6;//�㲥��ϢҲҪ�գ�����֮���ǻ�Ĺ�ϵ
	hcan1_fliter_2.mask=0xFC0;
	hcan_init(&hcan1_fliter_2,1);//����ط���ʼ���˵���֡�շ�������
  HAL_CAN_Start(&hcan1);//����CAN
	//����fifo0�����ݽ���

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
 * @brief ��ʼ��ͨ����ز���, ��ʼ��CANӲ�����ã�//�κ����ڸ�������ֻ�������ù���
 *  ������Ϊ500K
 *  �ط�����Ϊ�Զ��ط�
 *  bus-off�ָ�����Ϊ�Զ��ָ�
 * @param  can_frame_filter_cfg �������ñ�����CAN ID��MASK
 * @param  filter_num      �������ñ�Ĵ�С
 * @return int8_t 
         0����ʼ���ɹ�
        -1����ʼ��ʧ��
 */

 int8_t hcan_driver_hw_init(hcan_frame_filter_t *can_filter,uint8_t filter_num)
{
	CAN_FilterTypeDef CAN_FilterType;
// ��������id,STM32F072RBTx�ṩ��14������������id�������ó�0~13
  CAN_FilterType.FilterBank=filter_num;

// ����λ��Ϊ32λ
  CAN_FilterType.FilterScale=CAN_FILTERSCALE_32BIT;

// ����Ϊ����ģʽ
  CAN_FilterType.FilterMode=CAN_FILTERMODE_IDMASK;

// ����ǰ�����ֽڵ�STDID[10:3]��STDID[2:0]��EXID[17:13]
  CAN_FilterType.FilterIdHigh=((can_filter->can_id<<3) >>16) &0xffff;
  
// ���ú������ֽڵ�EXID[12:5]��EXID[4:0]��IDE��RTR��Ԥ����һ��0
  CAN_FilterType.FilterIdLow=(can_filter->can_id<<3) | CAN_ID_EXT;
  
// ��������ǰ�����ֽ�,����3λ�ٻ�CAN_ID_EXT����Ϊ������λ������ID������IDE��RTR��Ԥ����0
  CAN_FilterType.FilterMaskIdHigh=((can_filter->mask<<3|CAN_ID_EXT)>>16)&0xffff;

// ��������������ֽ�,����3λ�ٻ�CAN_ID_EXT����Ϊ������λ������ID������IDE��RTR��Ԥ����0
  CAN_FilterType.FilterMaskIdLow=(can_filter->mask<<3|CAN_ID_EXT)&0xffff;

// ����Ϣ�ŵ�FIFO0���������
  CAN_FilterType.FilterFIFOAssignment=CAN_RX_FIFO0;

// ���������
  CAN_FilterType.FilterActivation=ENABLE;

// ���ù�����
  if(HAL_CAN_ConfigFilter(&hcan1,&CAN_FilterType)!=HAL_OK)
  {
    Error_Handler();
  }

	return 0;
}



//�жϽ��յĻص�����
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
 * @brief ���ٱ��ķ��ͣ� ���ڵ�֡���ķ���
 *          ��֡����ʹ�ó���֡���͵���������������䣬�������֡������á�
 * @param  frame            CAN֡
 * @return int8_t 
        0: ���ͳɹ�
        1: ����ʧ��
 */
 int8_t hcan_driver_send_single_frame(uint32_t id, uint8_t *data, uint8_t len)
{	  
	CAN_TxHeaderTypeDef TxHeader;
	 
	TxHeader.ExtId = id;
	TxHeader.DLC = len;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	  if((hcan1.Instance->TSR & CAN_TSR_TME0) != 0U)//�������0�յģ���������֡
		{
			if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX0)!=0)//ʹ��0�����������͵�֡{
			{	
				if((hcan1.Instance->TSR & CAN_TSR_TME1) != 0U)//����0���գ���������1�գ�������1����֡
				{
					if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX1)!=0)
					return 1;
					printf("������-����1���ͳɹ�\r\n");
				}
			}
//			printf("������-����0�ͳɹ�\r\n");
			return 0;
		}
		return 0;
}


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
 int8_t hcan_driver_send_multi_frame(uint32_t id, uint8_t *data, int fifo)
{
	CAN_TxHeaderTypeDef TxHeader;
	 
	TxHeader.ExtId = id;
	TxHeader.DLC = 8;
	TxHeader.IDE = CAN_ID_EXT;
	TxHeader.RTR = CAN_RTR_DATA;
	  if((hcan1.Instance->TSR & CAN_TSR_TME2) != 0U)//�������2�յģ���������֡
	{
		if(HAL_CAN_AddTxMessage(&hcan1,&TxHeader,data,(uint32_t*)CAN_TX_MAILBOX2)!=0)//Ĭ���õ�2���䷢��֡
			return 1;
		else
			printf("������-����2�ͳɹ�\r\n");
			return 0;
	}
	return 0;	
}


/**
 * @brief ��ѯ����״̬������ѯbus-off״̬
 * @return int8_t 
        0����������
        -1������bus-off
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
 * @brief ��λ���ϵ�CAN�����շ���
 * @return int8_t 
        0:�ɹ�
        -1:ʧ��
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
