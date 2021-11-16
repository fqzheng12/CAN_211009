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
 * @file hcan.h
 * @brief 
 * @author zhangqinghang (zhangqinghang705@hellobike.com)
 * @version 1.1
 * @date 2021-08-20
 * 
 * @copyright Copyright (c) 2021  Hello Bike
 * 
 */
#ifndef __HCAN_H__
#define __HCAN_H__

#include "stdint.h"
//#include "main.h"
#include "stdio.h"
/*HCAN 协议栈版本号*/
#ifndef HCAN_VERSION
#define HCAN_VERSION (1)
#endif




/* 优先级 */
typedef enum {
    PRIORITY_WARING = 0,
    PRIORITY_CONTROL,
    PRIORITY_STATUS,
    PRIORITY_FOTA,
    PRIORITY_DIAG,
    PRIORITY_END,
} hcan_priority_t;





#define PRIORITY_VERIFY(priority)       ((priority) < PRIORITY_END)

/* 设备ID */
typedef enum {
    DEV_UNKOWN = 0,
    DEV_CENTRAL_CONTROLER = 1,
    DEV_MOTOR_CONTROLER = 2,
    DEV_METER = 3,
    DEV_BATTERY_LOCK = 4,
    DEV_BATTERY = 5,
    DEV_WEIGHING_CELL = 6,
    DEV_LIGHT = 7,
    DEV_HELMET_LOCK = 8,
    DEV_WHEEL_LOCK = 9,
    DEV_DIAGNOSTIC_UNIT = 0x3F,
    DEV_END,
} hcan_id_t;




#define HCAN_ID_VERIFY(id)          (((id)!=DEV_UNKOWN) && ((id)<DEV_END))

/*多帧处理自身节点编号:
    HCAN_SELF_NUMBER_P: primary node number, 本节点的节点号
    HCAN_SELF_NUMBER_S: secondary noe number, 本节点的次节点号
    如果不使用次节点号， 设置次节点号为零
    参考第二部分：整车CAN通信规范---时序与应用规范
*/
#define HCAN_SELF_NUMBER_P (2)
#define HCAN_SELF_NUMBER_S (0)

#define HCAN_FRAME_DEST_VERIFY(dest) ((HCAN_SELF_NUMBER_P == (dest)) || (HCAN_SELF_NUMBER_S == (dest)))


#ifndef HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH
#define HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH (1)
#endif

/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */
#define HCAN_HW_FRAME_SIZE          8

/* ------------------------------------------------------------------------ */
/* 单帧队列长度 */
/* ------------------------------------------------------------------------ */
#ifndef HCAN_TX_SINGLE_FRAME_LIST_LENGTH
#define HCAN_TX_SINGLE_FRAME_LIST_LENGTH    2
#endif

/*ACK 不响应尝试重发次数*/
#ifndef HCAN_FRAME_NEED_ACK_RETRY_TIMES
#define HCAN_FRAME_NEED_ACK_RETRY_TIMES     3
#endif
/*首帧发送失败重试次数*/
#ifndef HCAN_FRAME_FF_RETRY_TIMES
#define HCAN_FRAME_FF_RETRY_TIMES     3
#endif

/* ------------------------------------------------------------------------ */
/* 多帧队列长度 */
/* ------------------------------------------------------------------------ */
#ifndef HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH
#define HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH  2
#endif

/* ------------------------------------------------------------------------ */
/* 多帧缓冲BUF长度 */
/* ------------------------------------------------------------------------ */
#ifndef HCAN_TX_MULTIPLE_FRAME_BUF_MAX
#define HCAN_TX_MULTIPLE_FRAME_BUF_MAX      128
#endif

/* ------------------------------------------------------------------------ */
/* 等待ACK帧的超时时间，单位ms */
/* ------------------------------------------------------------------------ */
#ifndef HCAN_WAIT_ACK_TIMEOUT_MS
#define HCAN_WAIT_ACK_TIMEOUT_MS            100
#endif

/* ------------------------------------------------------------------------ */
/*  */
/* ------------------------------------------------------------------------ */
#ifndef HW_CAN_ID_EXT
#define HW_CAN_ID_EXT       1
#endif

#ifndef HW_CAN_RTR_DATA
#define HW_CAN_RTR_DATA     1
#endif

/**
* @brief 帧接收缓存长度
*/
#ifndef HCAN_RECV_BUFFER_LEN
#define HCAN_RECV_BUFFER_LEN (320)
#endif
/**
* @brief 单帧ACK报文缓存空间
*/
#ifndef HCAN_RESEND_FRAME_MAX
#define HCAN_RESEND_FRAME_MAX (32)
#endif
/**
 * @brief 规定数据接收连续帧时的反馈频率, 块大小（帧数）
 */
#ifndef HCAN_BS
#define HCAN_BS (33)
#endif
/* * 
 *@brief 接收端的接收能力，发送端最小发送间隔
 */
#ifndef HCAN_STMIN
#define HCAN_STMIN (1)
#endif
/**
 *@brief 连续帧最大间隔时间
 */
#ifndef HCAN_H_CR
#define HCAN_H_CR (100)
#endif
/**
 * @brief 接收完成，或者接收BS数据后的处理延时：
    反馈给发送端，明确非首帧流控帧的流控帧的应答延时
*/
#ifndef HCAN_H_BR
#define HCAN_H_BR (100)
#endif
/**
 * @brief 首帧应答时间延时
 */
#ifndef HCAN_H_DS
#define HCAN_H_DS (100)
#endif
/**
 * @brief 自动重发与BUS-OFF自动恢复配置， 驱动需要支持自动重发与BUS-OFF自动恢复功能
 */
#ifndef CAN_EN_AUTO_RE_TRANSMIT
#define CAN_EN_AUTO_RE_TRANSMIT (1)
#endif
#ifndef CAN_EN_AUTO_BUS_OFF_RECOVER
#define CAN_EN_AUTO_BUS_OFF_RECOVER (1)
#endif
/**
 *@brief 周期调用函数周期
 * */
#ifndef HCAN_LOOP_PERIOD
#define HCAN_LOOP_PERIOD (10)
#endif

/* ------------------------------------------------------------------------ */
/* 发送数据时的权重,FAST类型会优先发送 */
/* ------------- ----------------------------------------------------------- */
#ifndef HCAN_FRAME_NORMAL
#define HCAN_FRAME_NORMAL   0
#endif

#ifndef HCAN_FRAME_FAST
#define HCAN_FRAME_FAST     0xfffffffe
#endif

/*
 * @brief error_code
        0:没有错误，多帧发送完成
        1, 总线断线
        2, 单帧ACK失败
        3, 首帧发送没有流控帧
        4, 连续帧没有流控帧
        5，多帧数据传输中断
        6，接收端忙，不能继续接收多帧
        7，CRC异常
        8，发送ACK异常
        9，发送流控帧异常
        10，未知错误
        11，写入硬件FIFO失败（总线忙）
*/
typedef enum{
    HCAN_ERROR_NONE=0,
    HCAN_ERROR_BUS_OFF,
    HCAN_ERROR_ACK_FAIL,
    HCAN_ERROR_FF_FAIL,
    HCAN_ERROR_CF_FAIL,
    HCAN_ERROR_CF_INTERRUPT,
    HCAN_ERROR_RECVER_BUSY,
    HCAN_ERROR_CRC_FAIL,
    HCAN_ERROR_SEND_ACK,
    HCAN_ERROR_SEND_FC,
    HCAN_ERROR_UNKOWN,
    HCAN_ERROR_WRITE_FIFO,
    HCAN_ERROR_ALL
}can_error_code_t;

typedef struct hcan_frame_filter_s
{
    uint32_t mask;
    uint32_t can_id;
}hcan_frame_filter_t;

/**
 * @brief 初始化接口, 完成CAN通信新模块的初始化

 * @param  cb_funct_tbl     回调函数表，可以为空， 不为空时， 末尾为{0，NULL}代表结束
 * @param  can_frame_filter_cfgMy 过滤配置表，可以为空
 * @return int8_t 
         0：初始化成功
         -1：使能模块异常
 */
extern int8_t hcan_init(hcan_frame_filter_t *can_frame_filter_cfg, uint8_t filter_num);

/**
 * @brief 
 * 数据发送接口参数不包含功能码，功能吗包含在数据中
 * @param  priority         优先级
 * @param  taget_id         目标节点
 * @param  self_id          源节点
 * @param  is_need_ack         是否需要ACK
 * @param  weight              单帧权重
 * @param  pdata            发送数据
 * @param  data_len         发送数据长度,最大长度 260
 * @return int8_t 
 *      -1: 参数错误
 *      -2: 总线异常
 *      -3: 缓存不足，协议栈忙碌
 *      -4: 发送长度过长,fifo空间不足
 *      -5: bus-off
 */
extern int8_t hcan_packet_transmit(hcan_priority_t priority, 
                                hcan_id_t taget_id, 
                                hcan_id_t self_id,
                                uint8_t is_need_ack,
                                uint8_t weight,
                                uint8_t *pdata,
                                uint16_t data_len);
/**
 * @brief 本函数用于判定当前是否还有k空间可用于多帧发送
 * @return int8_t 
 * 1:有
 * 0:没有
 */
extern int8_t hcan_is_multi_frame_buffer_avilable(void);
/**
 * @brief 数据包处理周期调用函数
    由应用层周期调用，周期由应用决定
 */
extern void hcan_loop(void);





/*外部实现函数，ACK反馈回调，报文接收回调， 通信错误回调*/
/**
 * @brief 接收到ACK报文，或者多帧发送完成回调函数
 * @param  type             1, 单帧ACK应答
                                data: ACK 应答帧
                            2, 多帧传输完成FC回调
                                多帧首帧
 * @param  data             回调数据
 * @return int8_t 
 *          0: 无错误
 *          -1:参数错误
 */
extern int8_t hcan_ack_fc_notify(uint8_t type,uint8_t prio, uint8_t dest, uint8_t src_id, uint8_t * data, uint16_t length);





/**
 * @brief 错误反馈函数接口
 * @param  error_code       0:没有错误，多帧发送完成
                            1, 总线断线
                            2, 单帧ACK失败
                            3, 首帧发送没有流控帧
                            4, 连续帧没有流控帧
                            5，多帧数据传输中断
                            6，接收端忙，不能继续接收多帧
                            7，CRC异常
                            8, 缓存不足，写入ACK 帧失败
                            9, 缓存不足，发送流控帧失败
                            10, 意外故障
                            11, 写入FIFO失败
 * @param  data             传入数据
 * @param  data_len         数据长度
 * @return int8_t 
 *          0: 无错误
 *          -1:参数错误
 */
extern int8_t hcan_msg_error_handle_notify(can_error_code_t error_code, uint8_t *data, uint8_t data_len);







/**
 * @brief  报文接收回调函数接口
 * @param  prio             优先级
 * @param  seq              序列号
 * @param  dest        目标节点
 * @param  src_id           源节点
 * @param  data             数据
 * @param  length           数据长度
 * @return int8_t 
 *          0: 无错误
 *          -1:参数错误
 */
extern int8_t hcan_msg_recv_notify(uint8_t prio, uint8_t dest, uint8_t src_id, uint8_t * data, uint16_t length);






/**
 * @brief  硬件接收报文接口， 由CAN通信驱动调用
 * @param  id             优先级
 * @param  packet         报文数据
 * @param  size             报文长度
 * @return int8_t 
 *          0: 无错误
 *          -1:参数错误
 *          -2:空间不足
 */
extern int8_t hcan_hw_frame_recv(uint32_t id, uint8_t *packet, uint8_t size);

#endif

