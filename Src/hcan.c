/**
* @copyright Copyright @ 2017 HB Technologies Corp.
* All rights reserved
*
* Redistribution and use in source and binary forms, with or without
* modification, are NOT permitted except as agreed by HB Techonologies Corp.
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*/
/**
 * @file hcan.c
 * @brief
 * @author zhangqinghang (zhangqinghang705@hellobike.com)
 * @version 1.1
 * @date 2021-08-23
 *
 * @copyright Copyright (c) 2021  Hello Bike
 *
 */
#include "hcan_private.h"
#include "hcan.h"
#include "lib_crc.h"
#include "lib_list.h"
#include "can.h"
//#include "board.h"
#include "usart.h"
#include "main.h"
#include "flash_if.h"
#include "tim.h"
/* 帧接收缓存：
*   帧接收缓存通过链表来实现, 头插尾取
* */
static LIST_DEFINE(recv_frame_head);
static hcan_recv_frame_t hcan_recv_frame_buffer[HCAN_RECV_BUFFER_LEN];

static uint8_t hcan_bus_off_status = 0;
extern uint8_t jump_flag;
/**
* @brief 检测当前总线是否正常或已经恢复
 * @return int8_t
* 0:异常
* 1:正常
 */
int8_t hcan_is_hw_working(void)
{
    return (hcan_bus_off_status == 0) ? 1 : 0;
}
/**
 * @brief
 * @return int8_t
 */
static int8_t hcan_hw_bus_off_handle(void)
{
    static uint16_t error_counter = 0;
    hcan_frame_t recover_frame = {0};

    if (hcan_driver_query_bus_status() < 0) {//如果总线busoff
        error_counter += HCAN_LOOP_PERIOD;//这个算的是ms，所以是周期长度累加
        hcan_bus_off_status = 1;
        if (error_counter >= HCAN_BUSOFF_RECOVER_LOOP_MAX) {
            /*多帧发送失败的回调时间相对较长，首帧300毫秒，连续帧100毫秒*/
            hcan_driver_fault_reset();
            hcan_driver_send_single_frame(HCAN_ID(0, 0, 0, 0, 0), recover_frame.data, 1); /*发送0帧，尝试恢复总线*/
            hcan_msg_error_handle_notify(HCAN_ERROR_BUS_OFF, NULL, 0);
            hcan_multi_frame_handle_init();          
            error_counter = 0;
        }
    } else {
        hcan_bus_off_status = 0;
        error_counter = 0;
    }
    return 0;
}
/**
 * @brief 执行初始化操作：软件缓存初始化，硬件初始化
 * @param  hcan_frame_filter_cfg 传入过滤参数
 * @param  filter_num 过滤数量
 * @return int8_t
* 0: 初始化成功
* -2: 硬件初始化失败
 */
int8_t hcan_init(hcan_frame_filter_t *hcan_frame_filter_cfg, uint8_t filter_num)
{
    if (0 != hcan_driver_hw_init(hcan_frame_filter_cfg, filter_num)) {
        /*硬件初始化*/
        return -2;
    }
    hcan_multi_frame_handle_init();
    hcan_single_frame_handle_init();
    list_init(&recv_frame_head);
    return 0;
}

static hcan_recv_frame_t *hcan_get_empty_recv_buffer(void)
{
    int i;

    for (i = 0; i < HCAN_RECV_BUFFER_LEN; i++) {
        if (hcan_recv_frame_buffer[i].used == 0) {
            return &(hcan_recv_frame_buffer[i]);
        }
    }
    return NULL;
}
/*
 * @brief 硬件驱动层调用此函数，将收到的数据传递给协议栈
 * @param id CANID
 * @param packet 数据payload
 * @param size 数据长度
 * @return
 *      0:recv success
 *      -1:parameter fault
 *      -2: witer fifo fail
 */
int8_t hcan_hw_frame_recv(uint32_t id, uint8_t *packet, uint8_t size)
{
    int i;
    hcan_recv_frame_t *empty_buffer = hcan_get_empty_recv_buffer();

    if (empty_buffer == NULL) {
        return -2;
    }
    if (packet == NULL || size > 8) {
        return -1;
    }
    empty_buffer->used = 1;
    empty_buffer->frame.prio = HCAN_FRAME_PRIORITY(id);
    empty_buffer->frame.flag = HCAN_FRAME_TYPE(id);
    empty_buffer->frame.rolling_seq = HCAN_FRAME_SEQ(id);
    empty_buffer->frame.dest = HCAN_FRAME_DEST(id);
    empty_buffer->frame.src = HCAN_FRAME_SOURCE(id);
    empty_buffer->frame.dlc = size;
    memcpy(empty_buffer->frame.data, packet, size);

    list_insert_tail(&recv_frame_head, &(empty_buffer->node));
    return 0;
}

/**
 * @brief 读取中断反馈的CAN报文
 * @param  frame  出参，报文指针
 * @return int8_t
* 0: 获取CAN报文成功
* -1： 参数错误
* -2： 未读取到报文
 */
static int8_t hcan_read_recv_frame(hcan_frame_t *frame)
{
    hcan_recv_frame_t *recv = NULL;

    if (frame == NULL) {
        return -1;
    }
    if (0 == list_count(&recv_frame_head)) {
        return -2;
    }

    recv = list_entry(hcan_recv_frame_t, list_first(&(recv_frame_head)), node);

    memcpy(frame, &(recv->frame), sizeof(hcan_frame_t));
    __disable_irq();
    list_delete(&(recv->node));
    recv->used = 0;
		__enable_irq();
    return 0;
}
/**
 * @brief 接收报文处理
 * @return int8_t
*   反馈接收处理结果
* 0:接收处理
* -2：未接收到报文
 */
static int8_t hcan_frame_recv_handle(void)
{
    uint8_t read_cnt = 0;
    hcan_frame_t frame;

    /*read recv buffer(fifo) */
    while (0 == hcan_read_recv_frame(&frame)) {//如果获取报文成功
        if (MULTI_FRAME_VERIFY(frame.flag)) {
            hcan_multi_frame_recv_handle(&frame);//是多帧给多帧，是单帧给单帧
        } else {
            hcan_single_frame_recv_handle(&frame);
        }
        if (read_cnt++ > HCAN_READ_RECV_FRAME_MAX_PER_LOOP) {//每个循环读取报文次数限制
            break;
        }
    }
    return -2;
}
/**
 * @brief CAN帧发送管理
 */
static void hcan_frame_transmit_handle(void)
{
    hcan_single_frame_transmit();
    hcan_multi_frame_transmit();
}
/**
 * @brief CAN收发时间管理
 */
static void hcan_frame_timer_handle(void)
{
    hcan_single_frame_timer_handle();
    hcan_multi_frame_timer_handle();
}
/**
* @brief CAN通信主循环, 此函数可以被重写，
*    以匹配不同的应用场景，但整体的结构不能变更
 */
void hcan_loop(void)
{
    /* can frame recv handle*/
    hcan_frame_recv_handle();
    /*can frame send handle*/
    hcan_frame_transmit_handle();
    /*can timer control handle*/
    hcan_frame_timer_handle();
    /*can bus error handle*/
    hcan_hw_bus_off_handle();
}
/**
 * @brief
 * @param  priority         优先级
 * @param  taget_id         目标节点
 * @param  self_id          源节点
 * @param  is_need_ack      是否需要ACK
 * @param  weight           帧权重
 * @param  pdata            发送数据
 * @param  data_len         数据长度
 * @return int8_t
* 0: 包发送成功
* -1: 参数错误
* -5: CAN 总线异常
 */
int8_t hcan_packet_transmit(hcan_priority_t priority,
                            hcan_id_t taget_id,
                            hcan_id_t self_id,
                            uint8_t is_need_ack,
                            uint8_t weight,
                            uint8_t *pdata,
                            uint16_t data_len)
{
    if (!(PRIORITY_VERIFY(priority)) || !(HCAN_ID_VERIFY(taget_id)) || !(HCAN_ID_VERIFY(self_id)) || pdata == NULL || data_len == 0) {
        return -1;
    }
    if (hcan_is_hw_working() == 0) {
        return -5;
    }
    if (data_len <= HCAN_HW_FRAME_SIZE) {
        return hcan_single_packet_transmit(priority, taget_id, self_id, is_need_ack, weight, pdata, data_len);
    } else {
        return hcan_multi_packet_transmit(priority, taget_id, self_id, pdata, data_len);
    }
}
/**
 * @brief 报文接收回调
 */
__attribute__((weak)) int8_t hcan_msg_recv_notify(uint8_t prio, uint8_t dest, uint8_t src_id, uint8_t *data, uint16_t length)
{
		int i,a;
  	  uint32_t ramsource[64];
	uint8_t Data[]={0xAA,0x55,0xAA,0x54,0xAA,0x53,0xAA,0x52,0xAA,0x51,0xAA,0x50,0xAA,0x51,0xAA,0x52,0xAA,0x53,0xAA,0x54,0xAA,0x55,};

//	else if(lengt
//	  {
//			printf("多帧接收结束，串口打印\r\n");
//			for(i=0;i<=length;i++)
//			{
//			 printf("%x",data[i]);
//			
//			}
//		}
//	
//	}
	if(data[0]==0xfe)
	{
		
//	printf("跳转APP运行\r\n");
//    MX_TIM2_Init();
//		  MX_CAN1_Init();
//		HAL_CAN_MspDeInit(&hcan1);
//		HAL_UART_MspDeInit(&huart5);
//		HAL_TIM_Base_MspDeInit(&htim2);
//		HAL_NVIC_ClearPendingIRQ(TIM2_IRQn);
//		HAL_DeInit();
////		HAL_FLASH_Lock();
////		NVIC_SystemReset();
////		HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
////		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
////		HAL_NVIC_DisableIRQ(TIM3_IRQn);
//		HAL_NVIC_DisableIRQ(SysTick_IRQn);
//	HAL_NVIC_ClearPendingIRQ(SysTick_IRQn);
////		NVIC_SystemReset();
////    __disable_irq();  /关闭总中断
//////		__disable_irq();
////		 /*
////        Reset of all peripherals  
////        这些外设关闭非常重要，否则不能够正常实现程序跳转功能，切记切记
////        */
////        __APB1_FORCE_RESET();
////        __APB1_RELEASE_RESET();
////        __APB2_FORCE_RESET();
////        __APB2_RELEASE_RESET();
////        
////        __AHB1_FORCE_RESET();
////        __AHB1_RELEASE_RESET();   
////        
////        __AHB2_FORCE_RESET();
////        __AHB2_RELEASE_RESET();  
////        
////        __AHB3_FORCE_RESET();
////        __AHB3_RELEASE_RESET();        

////        HAL_RCC_DeInit();   
//				SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk;   // 关闭滴答定时器中断
//    SysTick->VAL = 0;
//    SysTick->LOAD = 0;   
////    __DSB(); 
//	SCB->ICSR |=0x02000000U;
//	SCB->AIRCR =0xFA050300U;
////	  __DSB(); 
//	Jump2APP(APPLICATION_ADDRESS);
  jump_flag=1;
	}
	if(data[0]==0x80)
	{
//		printf("进入升级数据写入flash\r\n");		
	
	__IO  uint32_t flashdestination ;

  for(i=0;i<=(length-6)/4;i++)
	ramsource[i]=(((uint32_t)data[4*i+6])<<24)|(((uint32_t)data[4*i+7])<<16)|(((uint32_t)data[4*i+8])<<8)|((uint32_t)data[4*i+9]);

		

printf("数据包总长度为%d\r\n",length );

		flashdestination=APPLICATION_ADDRESS|((uint32_t)(data[4]<<8));//取地址,进行偏移
		printf("偏移地址:%d,%d\r\n",((uint32_t)(data[4]<<8)),data[4]);
//		if(((data[4]*256)%1024)==0)//当偏移地址是1k的整数倍
//			 ramsource = (uint32_t) & data[6];
//		for(i=0;i<64;i++)
//	  {
//			ramsource[i]=(((uint32_t)data[4*i+6])<<24)|(((uint32_t)data[4*i+7])<<16)|(((uint32_t)data[4*i+8])<<8)|((uint32_t)data[4*i+9]);
//		}
//		ramsource = (uint32_t)( & data[6]);//取数据地址
//	printf("初始地址的值为：%x\r\n",ramsource[0]);
//	if(((data[4]*256)%4096)==0)
//	{  	if(FLASH_If_Erase(flashdestination)==0)
//	    printf("***************用户flash擦除成功****\r\n");//擦除了内存区40k的数据
//	}
//	else
//    printf("***************用户flash擦除失败****\r\n");
		  FLASH_If_Init();//每次写之前都必须清标志位，不然不成功
		a=FLASH_If_Write(flashdestination, (uint32_t*)&data[6],(length-6)/4);

		if(a==FLASHIF_OK)
		  {
		
		  printf("写入成功\r\n");
      HAL_FLASH_Lock(); 
		  }
		else
			printf("写入失败,错误码为%d\r\n",a);

		
//		a=&(APPLICATION_ADDRESS|((uint32_t) & data[2]));
		
//    printf("取地址上第一个的数据:%x\r\n",ramsource[0]);

//		printf("取地址%x的数据:%x\r\n",(flashdestination+length-10),*(__IO uint32_t*)(flashdestination+length-10));
		printf("取地址%x的数据:%x\r\n",(flashdestination),*(__IO uint32_t*)(flashdestination));
  }
    return 0;
}
/**
 * @brief ACK， 多帧发送完成回调
 */
__attribute__((weak)) int8_t hcan_ack_fc_notify(uint8_t type, uint8_t prio, uint8_t dest, uint8_t src_id, uint8_t *data, uint16_t length)
{
    return 0;
}
/**
 * @brief 异常回调
 */
__attribute__((weak)) int8_t hcan_msg_error_handle_notify(can_error_code_t error_code, uint8_t *data, uint8_t data_len)
{
    return 0;
}
