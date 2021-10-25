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
* @file can_private_data.h
* @brief 
* @author zhangqinghang (zhangqinghang705@hellobike.com)
* @version 1.1
* @date 2021-08-11
* 
*/
#ifndef __HCAN_PRIVATE_H__
#define __HCAN_PRIVATE_H__

#include "hcan.h"
#include "lib_list.h"
//#include "debug.h"

#ifndef HCAN_READ_RECV_FRAME_MAX_PER_LOOP
#define HCAN_READ_RECV_FRAME_MAX_PER_LOOP (10)
#endif

#ifndef HCAN_BUSOFF_RECOVER_LOOP_MAX
#define HCAN_BUSOFF_RECOVER_LOOP_MAX (500)
#endif

#ifndef HCAN_MULTI_FRAME_TX_BUFFER_LEN
#define HCAN_MULTI_FRAME_TX_BUFFER_LEN (512)
#endif

#ifndef HCAN_MULTI_FRAME_RX_BUFFER_LEN
#define HCAN_MULTI_FRAME_RX_BUFFER_LEN (1030)
#endif
/*
 * ID 29bits
 * |<-优先级(3bits)->|<-电池保留(1bit)->|<-保留(5bits)->|<-帧标志(3bits)->|<-计数器(5bits)->|<-目标ID->|<-源ID->|
 */

#ifndef HCAN_MULTI_PACKET_DATA_OFFSET
#define HCAN_MULTI_PACKET_DATA_OFFSET (2)
#endif

/* ------------------------------------------------------------------------ */
/* CAN硬件发送失败重试时长，单位ms */
/* ------------------------------------------------------------------------ */
#ifndef HCAN_HW_BUSY_RETRY_CNT_MAX
#define HCAN_HW_BUSY_RETRY_CNT_MAX (20)
#endif

#define    HCAN_FB_CODE_HANDLE_ERROR (-5)
#define    HCAN_FB_CODE_STOP  (-4)
#define    HCAN_FB_CODE_CRC_ERROR (-3)
#define    HCAN_FB_CODE_SEQ_ERROR (-2)
#define    HCAN_FB_CODE_RECV_BUSY (-1)
#define    HCAN_FB_CODE_RECV_FF (0)
#define    HCAN_FB_CODE_RECV_CPLT  (1)
#define    HCAN_FB_CODE_RECV_BS (2)

/* ------------------------------------------------------------------------ */
/* 帧标志 */
/* ------------------------------------------------------------------------ */
typedef enum {
    SINGLE_FRAME_NO_ACK = 0,
    MULTIPLE_FRAME_FIRST,
    MULTIPLE_FRAME_OTHERS,
    MULTIPLE_FRAME_FC,
    SINGLE_FRAME_ACK,
    SINGLE_FRAME_NEED_ACK,
    FRAME_TYPE_END,
} hcan_frame_type_t;

#define FRAME_TYPE_VERIFY(type)         ((type) < FRAME_TYPE_END)
#define MULTI_FRAME_VERIFY(type)         (((type) == MULTIPLE_FRAME_FIRST) || \
                                            ((type) == MULTIPLE_FRAME_OTHERS) || \
                                            ((type) == MULTIPLE_FRAME_FC))

/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */

#define HCAN_FRAME_PRIORITY(id)         ((id>>26)&0x07)
#define HCAN_FRAME_TYPE(id)             ((id>>17)&0x07)
#define HCAN_FRAME_DEST(id)             ((id>>6)&0x3F)
#define HCAN_FRAME_SOURCE(id)           ((id)&(0x3fl))
#define HCAN_FRAME_SEQ(id)              ((id>>12)&0x1F)

#define HCAN_ID(priority, frame_type, count, dest, source) \
    ((uint32_t)((((priority)&0x07)<<26) |     \
        (0x01l<<25) |                       \
        (((frame_type)&0x07)<<17) |           \
        (((count)&0x1f)<<12) |                \
        (((dest)&0x3f)<<6) |                  \
        (((source)&0x3f))))

/* ack 帧匹配 */
#define HCAN_REQ_ACK_CMP(req_id, ack_id)    \
    ((HCAN_FRAME_PRIORITY(req_id)==HCAN_FRAME_PRIORITY(ack_id)) &&  \
     (HCAN_FRAME_SEQ(req_id)==HCAN_FRAME_SEQ(ack_id)) &&            \
     (HCAN_FRAME_DEST(req_id)==HCAN_FRAME_SOURCE(ack_id)) &&        \
     (HCAN_FRAME_SOURCE(req_id)==HCAN_FRAME_DEST(ack_id)))

#define NOTIFY_ACK_CONFIRMED (1)
#define NOTIFY_MULTI_FRAME_TX_CMPLT (2)
#define HCAN_DEFAULT_ZERO (0)
/**
* @brief 关键信息掩码，优先级，编号，源节点，目标节点。。
*/
typedef enum 
{
    FRAME_SEND_FF = 0,
    FRAME_SEND_CF,
    FRAME_WAIT_FC,
    FRAME_SEND_ALL
}frame_send_stm_t;

typedef struct 
{
    uint8_t prio;
    uint8_t flag;
    uint8_t rolling_seq;
    uint8_t dest;
    uint8_t src;
    uint8_t dlc;
    uint8_t data[HCAN_HW_FRAME_SIZE];
}hcan_frame_t;

typedef struct {
    uint8_t used;
    hcan_frame_t frame;
    list_t node;
}hcan_recv_frame_t;

typedef struct  
{
    uint16_t bs;
    uint16_t br;
    uint8_t stmin;
    uint8_t update_flag;
    uint8_t send_period_control;/*发送周期控制， 对齐STMIN*/
}segmented_frame_tx_para_t;

typedef struct {
    uint16_t t_h_ds;
    uint16_t t_h_bs;
    
    uint8_t t_h_ds_en;/*发送首帧后，等待首帧流控帧， 接收到首帧流控反馈后，关闭 */    
    uint8_t t_h_bs_en;/* 发送BS之后， 等待流控帧，或者全部发送完成之后，等待流控帧， 接收到相应的流控帧后，关闭*/
    uint8_t ff_fc_ovtim_flag;/*首帧发送后，流控帧反馈超时标志*/
    uint8_t cf_fc_ovtim_flag;/* 其他流控帧反馈超时*/
}tx_seg_frame_timer_t;

typedef struct 
{
    /*接收定时参数*/
    uint8_t t_h_cr_en;/*发送首帧流控，或者BS满流控后，激活，等待连续帧， 发送接收完成后，关闭*/ 
    uint8_t cf_ovtim_flag;/* 连续帧发送超时*/
    uint16_t t_h_cr;
}rx_seg_frame_timer_t;

typedef struct 
{
    uint8_t used;
    uint8_t retry_cnt;
    uint8_t retry_timer;
    hcan_frame_t frame;
    list_t node;
}single_frame_item_t;
/**
* transmit control module
*/
typedef struct 
{
    hcan_frame_t single_frame_backup;/*单帧无法写入到发送寄存器，备份再次发送*/
    list_t wait_send_head;/*单帧发送队列头*/
    list_t wait_ack_head;/*单帧重发队列头*/
    single_frame_item_t list_buffer[HCAN_TX_SINGLE_FRAME_LIST_LENGTH];
    uint8_t hw_fifo_busy_cnt;/*写入硬件FIFO失败计数：fifo忙，导致无法写入*/
}single_frame_tx_t;

typedef struct  
{
    /*多帧发送FIFO缓存
     * 支持的最长多帧为256byte，先设置为512，足够接收，并留有缓冲空间
    */
    uint8_t data[HCAN_MULTI_FRAME_TX_BUFFER_LEN];
    /*报文参数：优先级，目标节点，源节点*/
    uint8_t prio;
    uint8_t dest;
    uint8_t src;
    int8_t fc_result_code;
    /*首帧发送失败，直接获取重发*/
    hcan_frame_t current_tx_frame;
    uint8_t hw_fifo_busy_cnt;/*写入硬件FIFO失败计数：fifo忙，导致无法写入*/
    /*多帧发送计时*/
    tx_seg_frame_timer_t tx_timer;
    /*需要发送的数据总长度*/
    uint16_t total_data_len;
    /*已经发送的数据长度*/
    uint16_t send_data_len;
    /*已经发送的数据长度是否到达BS * 8长
     * 发送首帧后，清零
     * 接收数据到BS*8，流控帧反馈之后，清零
     * */
    uint16_t send_data_len_bs;
    /*首帧发送失败重试计数
     * 重发3次后，放弃
     * */
    uint8_t ff_retry_cnt;
    uint8_t rolling_cnt;
    /*接收节点反馈的发送参数*/
    segmented_frame_tx_para_t seg_tx_para;
    /*多帧发送状态机*/
    frame_send_stm_t send_stm;
    /*  0: not in use
        1： in use
    */
    uint8_t used;
}multi_frame_tx_t;
/*
recv control module
*/
typedef struct 
{
    uint8_t used;/*0：当前buffer空， 1：当前buffer被占用*/
    uint8_t prio;
    uint8_t dest;
    uint8_t src;/*CAN ID相关参数*/
    uint16_t total_data_len;
    uint16_t crc_value;
    uint16_t recv_data_len;
    uint16_t recv_len_bs;/*接收长度控制*/
    uint8_t rolling_cnt;
    uint8_t rx_buffer[HCAN_MULTI_FRAME_RX_BUFFER_LEN];
    rx_seg_frame_timer_t rx_timer;/*接收时间控制*/
}multi_frame_rx_t;


extern int8_t hcan_single_frame_transmit(void);

extern int8_t hcan_multi_frame_transmit(void);

extern void hcan_single_frame_timer_handle(void);
 
extern void hcan_multi_frame_timer_handle(void);

extern int8_t hcan_single_frame_recv_handle(hcan_frame_t * frame);

extern void hcan_clear_multi_frame_rcm(void);

extern void hcan_clear_multi_frame_tcm(uint8_t tcm_index);

extern int8_t hcan_single_packet_transmit( uint8_t priority, uint8_t taget_id,
                                            uint8_t self_id,uint8_t is_need_ack,uint8_t weight,
                                            uint8_t *pdata, uint16_t data_len);

extern int8_t hcan_multi_packet_transmit(uint8_t priority, uint8_t taget_id, uint8_t self_id, uint8_t *pdata, uint16_t data_len);

extern void hcan_multi_frame_handle_init(void);

extern void hcan_single_frame_handle_init(void);

extern int8_t hcan_multi_frame_recv_handle(hcan_frame_t * frame);

extern int8_t hcan_add_single_frame_list(uint8_t priority, uint8_t dest, uint8_t src,
                                  hcan_frame_type_t type, uint8_t is_need_ack, uint8_t weight,
                                  uint8_t *pdata, uint16_t data_len);
#endif
