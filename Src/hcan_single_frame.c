/**
* @copyright Copyright @ 2018 HB Technologies Corp.
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
 * @file hcan_single_frame.c
 * @brief
 * @author zhangqinghang (zhangqinghang705@hellobike.com)
 * @version 1.1
 * @date 2021-08-23
 *  参考《第一部分：整车CAN通信规范---通信报文与传输规范》
 *  参考《第二部分：整车CAN通信规范---时序与应用规范》
 * @copyright Copyright (c) 2021  Hello Bike
 *
 */
#include "hcan_private.h"
#include "hcan.h"
#include "string.h"
#include "lib_crc.h"
#include "can.h"

//#include "osi.h"

/*
    单帧发送管理模块
*/
static single_frame_tx_t hcan_single_frame_tx = {0};





/*
    找到当前空的buffer位置
*/

static single_frame_item_t *hcan_get_empty_buffer(void)
{
    int i;

    for (i = 0; i < HCAN_TX_SINGLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_single_frame_tx.list_buffer[i].used == 0) {
            return &(hcan_single_frame_tx.list_buffer[i]);
        }
    }
    if (i >= HCAN_TX_SINGLE_FRAME_LIST_LENGTH) {
        return NULL;
    }
		return NULL;
}




/**
 * @brief 向单帧链表插入待发送的CAN报文,
        本接口支持主动发送单帧报文，反馈ACK报文， 发送流控帧报文。
 * @param  priority     优先级
 * @param  dest         目标节点
 * @param  src          源节点
 * @param  type         帧类型
                        参数：单帧无ACK， ACK帧， 流控帧
 * @param  is_need_ack  是否需要ACK，
                        如果需要ACK， 则程序自动屏蔽帧类型参数，帧类型为单帧需要ACK类型
                        如果不需要ACK， 则帧类型为参数定义，
 * @param  weight       单帧权重
 *                      正常帧从尾部插入， 权重帧从头部插入
 *                      发送报文时从头部获取
 *                      对于非权重（正常）帧，插入和获取操作不同的指针，可以不使用互斥
 * @param  pdata        数据
 * @param data_len      数据长度
 * @return int8_t
 * 0: 单帧接入成功
 * -2: 链表已满，加入失败
 */
int8_t hcan_add_single_frame_list(uint8_t priority, uint8_t dest, uint8_t src,
                                  hcan_frame_type_t type, uint8_t is_need_ack, uint8_t weight,
                                  uint8_t *pdata, uint16_t data_len)
{
    static uint8_t ack_seq = 0;
    single_frame_item_t *empty_buffer = hcan_get_empty_buffer();//拿到空的buffer

    if (empty_buffer == NULL) {
        return -2;
    }
    if (!PRIORITY_VERIFY(priority) || !HCAN_ID_VERIFY(dest) || !HCAN_ID_VERIFY(src) || !FRAME_TYPE_VERIFY(type) || pdata == NULL || data_len == 0) {
        return -1;//查参数。
    }
    empty_buffer->used = 1;
    empty_buffer->retry_cnt = 0;
    empty_buffer->frame.prio = priority;
    empty_buffer->frame.dest = dest;
    empty_buffer->frame.src = src;
    if (is_need_ack == 0) {
        empty_buffer->frame.rolling_seq = 0;
        empty_buffer->frame.flag = type;
    } else {
        ack_seq++;
        ack_seq &= 0x1F;

        empty_buffer->frame.rolling_seq = ack_seq;
        empty_buffer->frame.flag = SINGLE_FRAME_NEED_ACK;
    }
    empty_buffer->frame.dlc = data_len;

    memcpy(&(empty_buffer->frame.data), pdata, data_len);

    __disable_irq();/*关闭中断，确保链表操作*/
    if (weight == HCAN_FRAME_NORMAL) {
        list_insert_tail(&(hcan_single_frame_tx.wait_send_head), &(empty_buffer->node));
    } else {
        list_insert(&(hcan_single_frame_tx.wait_send_head), &(empty_buffer->node));
    }
    __enable_irq();/*恢复中断*/
    return 0;
}





/**
 * @brief 删除已经ACK的帧，解除重发//已经ACK后的
 * @param  ack_frame        ACK帧
 * @return int8_t
 0:操作成功
 -2:当前没有匹配,非本节点的ID
 */
static int8_t hcan_delete_resend_frame(hcan_frame_t *ack_frame)
{
    list_t *list;
    single_frame_item_t *re_tx_item;

    if (ack_frame == NULL) {
        return -1;
    }
    if (list_count(& (hcan_single_frame_tx.wait_ack_head)) == 0) {
        return -2;
    }
    if (HCAN_FRAME_DEST_VERIFY(ack_frame->dest)) {
        list_for_each(list, &(hcan_single_frame_tx.wait_ack_head)) {
            re_tx_item = list_entry(single_frame_item_t, list, node);
            if (re_tx_item->retry_cnt != 0 &&
                re_tx_item->frame.rolling_seq == ack_frame->rolling_seq &&
                re_tx_item->frame.src == ack_frame->dest &&
                re_tx_item->frame.dest == ack_frame->src) {
                list_delete(&(re_tx_item->node));
                re_tx_item->used = 0;
                return 0;
            }
        }
    } else {
        return -2;
    }
    return 0;
}






/**
 * @brief 重发定时管理，//根据设定的周期进行重发计数
 */
void hcan_single_frame_timer_handle(void)
{
    list_t *list;
    single_frame_item_t *re_tx_item;

    if (list_count(&(hcan_single_frame_tx.wait_ack_head)) == 0) {
        return;
    }
    list_for_each(list, &(hcan_single_frame_tx.wait_ack_head)) {
        re_tx_item = list_entry(single_frame_item_t, list, node);
        re_tx_item->retry_timer += HCAN_LOOP_PERIOD;
        if (re_tx_item->retry_timer > HCAN_WAIT_ACK_TIMEOUT_MS) {
            list = re_tx_item->node.pre;
            list_delete(&(re_tx_item->node));
            re_tx_item->retry_timer = 0;
            list_insert_tail(&(hcan_single_frame_tx.wait_send_head), &(re_tx_item->node));
        }
    }
}





/**
 * @brief CAN单帧发送管理//管理发送的主要的逻辑在这里实现//如果是要ACK的发送帧，在回调里面收到该帧的ACK之前，会一直重发，周期为loop周期
 * @return int8_t//接收里面有专门的函数在接收到回调后删除重发
 * 1:发送正常
 * 0: 空
 * -2: 无法获取互斥量不能发送
 *
 */
int8_t hcan_single_frame_transmit(void)
{
    uint32_t id;//专门把ID拿出来
    hcan_frame_t tx_frame = {0};//定义一个空的发送帧
    single_frame_item_t *single_frame = NULL;//单帧管理结构体

    if (list_count(&hcan_single_frame_tx.wait_send_head) == 0) {
        return 0;//应该是看看列表里面有多少个单元内容
    }
    single_frame = list_entry(single_frame_item_t, list_first(&(hcan_single_frame_tx.wait_send_head)), node);//找到单帧发送序列里面的那个等待发送的单帧管理结构体，并把它拿出来

    id = HCAN_ID(single_frame->frame.prio, single_frame->frame.flag, single_frame->frame.rolling_seq, single_frame->frame.dest, single_frame->frame.src);//拼出要发送单帧的ID
    if (0 == hcan_driver_send_single_frame(id, single_frame->frame.data, single_frame->frame.dlc)) {//这个驱动函数要自己写
        hcan_single_frame_tx.hw_fifo_busy_cnt = 0;//如果发送成功，则将发送序列的发送繁忙计数清0.
        list_delete(&(single_frame->node));//应该是将链表中的当前已发送节点删除，这个节点编号是放在帧里面的
        if (single_frame->frame.flag == SINGLE_FRAME_NEED_ACK) {//如果是需要即时ACK的帧
            single_frame->retry_timer = 0;//初始化重发计数为0
            single_frame->retry_cnt++;//重发计数加加
            if (single_frame->retry_cnt < HCAN_FRAME_NEED_ACK_RETRY_TIMES) {
                list_insert_tail(&(hcan_single_frame_tx.wait_ack_head), &(single_frame->node));
            } else {/*重发失败，释放缓存*///重发次数已经超过允许最大重发次数
                single_frame->used = 0;
                single_frame->retry_cnt = 0;
            }
        } else {
            single_frame->used = 0;//不需要ACK，直接把链表缓存清零
        }
    } else {//如果发送不成功
        hcan_single_frame_tx.hw_fifo_busy_cnt++;//还在发送中
        if (hcan_single_frame_tx.hw_fifo_busy_cnt > HCAN_HW_BUSY_RETRY_CNT_MAX) {//超时
            hcan_single_frame_tx.hw_fifo_busy_cnt = 0;//归零
            hcan_msg_error_handle_notify(HCAN_ERROR_WRITE_FIFO, NULL, 0);//调一个回调
        }
    }
    return 1;
}





/**
* @brief  单帧包发送接口，处理上层下发的单包（帧）数据，写入单帧发送链表
 * @param  priority         优先级
 * @param  dest         目标节点号
 * @param  src          源节点号
 * @param  is_ned_ack       是否需要ACK应答
 * @param  weight           单帧权重
 * @param  pdata            数据
 * @param  data_len         数据长度
 * @return int8_t
 * 0: 发送成功
 * -2:当前链表已满
 * -3:互斥量获取失败
 */
int8_t hcan_single_packet_transmit(uint8_t priority, uint8_t dest,
                                   uint8_t src, uint8_t is_need_ack, uint8_t weight,
                                   uint8_t *pdata, uint16_t data_len)
{
    return hcan_add_single_frame_list(priority, dest, src,
                                      SINGLE_FRAME_NO_ACK, is_need_ack, weight,
                                      pdata, data_len);//转了个接口，需要发送单帧，就把它塞到单帧发送管理链表里面去，权重高插前面，权重低插后面，发送的地方每个循环从链表里面拿数据发送
}




								   
/**
 * @brief 处理接收到的单帧数据,函数由hcan_loop驱动调用，处理接收到的CAN帧//接收到的单帧数据传给这个函数进行分类处理
 * @param  frame            CAN报文
 * @return int8_t
* 0: 处理成功
 */
int8_t hcan_single_frame_recv_handle(hcan_frame_t *frame)
{
    hcan_frame_t ack_frame;

    if (frame->flag == SINGLE_FRAME_ACK && HCAN_FRAME_DEST_VERIFY(frame->dest)) {
        /*处理ACK，删除重发指令*/
        hcan_ack_fc_notify(NOTIFY_ACK_CONFIRMED, frame->prio, frame->dest, frame->src, frame->data, frame->dlc);
        hcan_delete_resend_frame(frame);
    } else {
        if (frame->flag == SINGLE_FRAME_NEED_ACK && HCAN_FRAME_DEST_VERIFY(frame->dest)) {
            if (hcan_add_single_frame_list(frame->prio, frame->src, frame->dest,
                                           SINGLE_FRAME_ACK, 0, 0,
                                           frame->data, frame->dlc) < 0) {
                hcan_msg_error_handle_notify(HCAN_ERROR_SEND_ACK, NULL, 0);
            }
        }
        hcan_msg_recv_notify(frame->prio, frame->dest, frame->src, frame->data, frame->dlc);/* 执行报文接收回调 */
    }
    return 0;
}







/**
 * @brief 单帧处理初始化
 */
void hcan_single_frame_handle_init(void)
{
    /*init fifo*/
    memset(&(hcan_single_frame_tx.list_buffer), HCAN_DEFAULT_ZERO, 32 * sizeof(single_frame_item_t));
    list_init(&(hcan_single_frame_tx.wait_send_head));
    list_init(&(hcan_single_frame_tx.wait_ack_head));
}

