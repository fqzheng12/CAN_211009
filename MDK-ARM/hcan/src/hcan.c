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
/* 帧接收缓存：
*   帧接收缓存通过链表来实现, 头插尾取
* */
static LIST_DEFINE(recv_frame_head);
static hcan_recv_frame_t hcan_recv_frame_buffer[HCAN_RECV_BUFFER_LEN];

static uint8_t hcan_bus_off_status = 0;
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
	printf("进入了回调函数\r\n");
		uint16_t i;
//	if(prio==1&&dest==2&&src_id==1)
//	{if(length<=8)
//		hcan_packet_transmit(PRIORITY_CONTROL,DEV_CENTRAL_CONTROLER,DEV_MOTOR_CONTROLER,0,HCAN_FRAME_NORMAL,data,sizeof (data)+1);
//	else if(length>=8)
//	  {
			printf("多帧接收结束，串口打印\r\n");
			for(i=0;i<=length;i++)
			{
			 printf("%x",data[i]);
			
			}
//		}
//	
//	}
	
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
