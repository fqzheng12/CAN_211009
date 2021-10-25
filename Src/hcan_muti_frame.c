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
 * @file hcan_muti_frame.c
 * @brief CAN收发多帧处理模块//主要思想还是先建立好循环发送的通道和一个链表，将需要发送的数据插到表里面，循环持续发送或者接收
 * 1，多帧包发送接收
 * 2，多帧帧接收发送
 * 3，多帧协议处理
 * @author zhangqinghang (zhangqinghang705@hellobike.com)
 * @version 1.1
 * @date 2021-08-23
 *  参考《第一部分：整车CAN通信规范---通信报文与传输规范》
 *  参考《第二部分：整车CAN通信规范---时序与应用规范》
 * @copyright Copyright (c) 2021  Hello Bike



 
 *
 */
#include "can.h"
#include "hcan_private.h"
#include "hcan.h"
#include "string.h"
#include "lib_crc.h"
/*多帧收发缓存与控制模块*///收发的缓存数组初始化
static multi_frame_tx_t hcan_multi_frame_tx[HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH];
static multi_frame_rx_t hcan_multi_frame_rx[HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH];
static uint16_t recv_bs_in_byte = (HCAN_HW_FRAME_SIZE *HCAN_BS);//计算一个bs有多少个字节



/**
 * @brief 初始化多帧操作接口
 */
void hcan_multi_frame_handle_init(void)
{
    memset(&hcan_multi_frame_rx[0], HCAN_DEFAULT_ZERO, (HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH)*sizeof(multi_frame_rx_t));//对上面定义的数组开始地址后面的区域全部赋值为默认can值。
    memset(&(hcan_multi_frame_tx[0]), HCAN_DEFAULT_ZERO, (HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH)*sizeof(multi_frame_tx_t));
    recv_bs_in_byte = (HCAN_HW_FRAME_SIZE * HCAN_BS);
}




/**
 * @brief 根据接收到的流控帧，调整对应的发送参数
 * @param  frame            反馈的流控帧
 * @return int8_t
 * 0: 调整成功
 * -1: 参数错误
 * -2: 非法流控帧
 */
static int8_t hcan_adjust_send_parameter(hcan_frame_t *frame)//传进来的应该是个流控帧
{
    int i;
    if (frame == NULL) {//一般进来都是先验参，有问题就返回了
        return -1;
    }
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {//多帧队列的长度//这段没看懂
        if (hcan_multi_frame_tx[i].dest == frame->src && hcan_multi_frame_tx[i].src == frame->dest && hcan_multi_frame_tx[i].prio == frame->prio) {
            /*更新发送参数*/
            hcan_multi_frame_tx[i].seg_tx_para.bs = HCAN_HW_FRAME_SIZE * frame->data[0];
            hcan_multi_frame_tx[i].seg_tx_para.stmin = frame->data[1];
            hcan_multi_frame_tx[i].seg_tx_para.br = 10 * frame->data[2];/*BR 反馈时间单位为10毫秒*/
            hcan_multi_frame_tx[i].fc_result_code = frame->data[3];
            hcan_multi_frame_tx[i].seg_tx_para.update_flag = 1;/*通知流控已经更新*/
            break;
        }
    }
    if (i >= HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH) {
//        port_trace("flow control not for men");
        return -2;
    }
    return 0;
}





/**
 * @brief 本函数用于发送流控帧
 * @param  prio             优先级
 * @param  dest        目标节点
 * @param  src           源节点
 * @param  bs               block size， 块大小
 * @param  stmin            连续帧的帧间隔
 * @param  br               流控反馈最大延时
 * @param  fb_code          反馈码
 * @return int8_t
 * 0: 发送成功
 * -2: 发送失败
 */

static int8_t hcan_can_send_flow_control_frame(uint8_t prio, uint8_t dest, uint8_t src, uint8_t bs, uint8_t stmin, uint8_t br, int8_t result_code)
{
    uint8_t data[8] = {0};

    data[0] = bs;
    data[1] = stmin;
    data[2] = br;
    data[3] = (uint8_t)result_code;

    if (0 != hcan_add_single_frame_list(prio, dest, src, MULTIPLE_FRAME_FC, 0, 0, data, HCAN_HW_FRAME_SIZE)) {
        hcan_msg_error_handle_notify(HCAN_ERROR_SEND_FC, NULL, 0);
//         ("send flow control frame fail: prio: %d target: %d src: %dn", prio, dest, src);
        return -2;
    }

    return 0;
}







static multi_frame_rx_t *hcan_get_empty_recv_buffer(void)//找到空的接收位置
{
    int i;
    for (i = 0; i < HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_rx[i].used == 0) {
            return &(hcan_multi_frame_rx[i]);
        }
    }
    return NULL;
}







static int8_t hcan_handle_recv_first_frame(hcan_frame_t *frame)//接收首帧
{
    multi_frame_rx_t *empty_buffer = hcan_get_empty_recv_buffer();
    if (empty_buffer == NULL) {
        hcan_can_send_flow_control_frame(frame->prio, frame->src, frame->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_BUSY);
        return -2;
    }
    if (frame == NULL) {
        return -1;
    }

    empty_buffer->used = 1;
    empty_buffer->prio = frame->prio;
    empty_buffer->src = frame->src;
    empty_buffer->dest = frame->dest;
    empty_buffer->recv_data_len = HCAN_DEFAULT_ZERO;
    empty_buffer->rolling_cnt = HCAN_DEFAULT_ZERO;
    empty_buffer->rx_buffer[0] = frame->data[0];
    empty_buffer->rx_buffer[1] = frame->data[1];
    empty_buffer->total_data_len = (uint16_t)(frame->data[2]) << 8 | frame->data[3];
    empty_buffer->crc_value = (uint16_t)(frame->data[4]) << 8 | frame->data[5];
    hcan_can_send_flow_control_frame(frame->prio, frame->src, frame->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_FF);/*反馈流控帧，已经接收到首帧*/
    memset(&(empty_buffer->rx_timer), HCAN_DEFAULT_ZERO, sizeof(rx_seg_frame_timer_t));/*备份接收到的首帧*/
    empty_buffer->rx_timer.t_h_cr_en = 1;/*打开连续帧间隔超时计时*/

    return 0;
}







static multi_frame_rx_t *hcan_get_matched_recv_buffer(hcan_frame_t *frame)
{
    int i;
    for (i = 0; i < HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH; i++) {//接收多帧列表长度
        /*连续帧：根据已经分配好的缓存存入*/
        if (hcan_multi_frame_rx[i].prio == frame->prio &&
            hcan_multi_frame_rx[i].src == frame->src &&
            hcan_multi_frame_rx[i].dest == frame->dest) {
            return &(hcan_multi_frame_rx[i]);
        }
    }
    return NULL;
}






static int8_t hcan_handle_recv_continuous_frame(hcan_frame_t *frame)//接收连续帧
{
    multi_frame_rx_t *matched_buffer = hcan_get_matched_recv_buffer(frame);//获得已经匹配好的一帧
    if (matched_buffer == NULL) {
        hcan_can_send_flow_control_frame(frame->prio, frame->src, frame->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_BUSY);
        return -2;
    }
    if (frame == NULL) {
        return -1;
    }
    matched_buffer->rx_timer.t_h_cr = 0;/*接收连续帧，清空 CR定时器*/
    if (frame->rolling_seq - matched_buffer->rolling_cnt == 1 || (frame->rolling_seq == 1 && matched_buffer->rolling_cnt == 0x1F)) {
        matched_buffer->rolling_cnt = frame->rolling_seq;
    } else {
        /*反馈流控帧，数据顺序错误*/
//							printf("跳入了改错误\r\n");
        hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_SEQ_ERROR);
        memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));/*清空当前多帧缓存*/
        return -3;
    }
    memcpy(&(matched_buffer->rx_buffer[matched_buffer->recv_data_len + HCAN_MULTI_PACKET_DATA_OFFSET]), frame->data, HCAN_HW_FRAME_SIZE);
    matched_buffer->recv_data_len += HCAN_HW_FRAME_SIZE;
    matched_buffer->recv_len_bs += HCAN_HW_FRAME_SIZE;

    if (matched_buffer->recv_data_len >= matched_buffer->total_data_len) {
        /*检查接收长度*/
        if (crc16_calc((matched_buffer->rx_buffer + HCAN_MULTI_PACKET_DATA_OFFSET), matched_buffer->total_data_len) == matched_buffer->crc_value) {
            /*CRC 校验*/
            if (0 == hcan_msg_recv_notify(matched_buffer->prio,
                                          matched_buffer->dest, matched_buffer->src,
                                          matched_buffer->rx_buffer, matched_buffer->total_data_len + 2)) {
                hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_CPLT);/*反馈流控帧，全部数据传输完成*/
            } else {
                hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_HANDLE_ERROR);/*反馈流控帧，数据处理失败*/
            }
            memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
        } else {
            /*反馈流控帧，CRC 校验错误*/
            hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_CRC_ERROR);
            memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
        }
    } else {
        if (matched_buffer->recv_len_bs == recv_bs_in_byte) {
            /*反馈流控帧，接收到BS长度的数据*/
            matched_buffer->recv_len_bs = 0;
            hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_BS);
        }
    }

    return 0;
}







/**
 * @brief 多帧接收处理函数，用于将接收到的多帧进行拼包。
 * @param  frame           接收到的多帧
 * @return int8_t
 * 0: 接收处理成功
 * -1:参数错误
 * -3:CAN帧不是本节点的多帧
 */
int8_t hcan_multi_frame_recv_handle(hcan_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }
    if (!HCAN_FRAME_DEST_VERIFY(frame->dest)) {
        /*如果多帧报文目标并非本节点，直接丢弃*/
        return -3;
    }
    if (frame->flag == MULTIPLE_FRAME_FIRST) {
        hcan_handle_recv_first_frame(frame);
    } else if (frame->flag == MULTIPLE_FRAME_OTHERS) {
        hcan_handle_recv_continuous_frame(frame);
    } else {  
        hcan_adjust_send_parameter(frame);/*接收到流控帧，激活连续帧发送*/
    }
    return 0;
}






/**
 * @brief 本函数用于对多帧收发的定时器进行管理
 */
void hcan_multi_frame_timer_handle(void)
{
    int i;
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_tx[i].tx_timer.t_h_bs_en != 0) {
            hcan_multi_frame_tx[i].tx_timer.t_h_bs += HCAN_LOOP_PERIOD;
            if (hcan_multi_frame_tx[i].tx_timer.t_h_bs >= hcan_multi_frame_tx[i].seg_tx_para.br) {
                hcan_multi_frame_tx[i].tx_timer.cf_fc_ovtim_flag = 1;
                hcan_multi_frame_tx[i].tx_timer.t_h_bs = 0;
            }
        }
        if (hcan_multi_frame_tx[i].tx_timer.t_h_ds_en != 0) {
            hcan_multi_frame_tx[i].tx_timer.t_h_ds += HCAN_LOOP_PERIOD;
            if (hcan_multi_frame_tx[i].tx_timer.t_h_ds >= HCAN_H_DS) {
                hcan_multi_frame_tx[i].tx_timer.ff_fc_ovtim_flag = 1;
                hcan_multi_frame_tx[i].tx_timer.t_h_ds = 0;
            }
        }
    }
    for (i = 0; i < HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_rx[i].rx_timer.t_h_cr_en != 0) {
            hcan_multi_frame_rx[i].rx_timer.t_h_cr += HCAN_LOOP_PERIOD;
            if (hcan_multi_frame_rx[i].rx_timer.t_h_cr > HCAN_H_CR) {
                /*连续帧间隔过久，接收失败*/
                hcan_multi_frame_rx[i].rx_timer.cf_ovtim_flag = 1;
                hcan_can_send_flow_control_frame(hcan_multi_frame_rx[i].prio, hcan_multi_frame_rx[i].src, hcan_multi_frame_rx[i].dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_STOP);
                memset(&hcan_multi_frame_rx[i], HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
            }
        }
    }
}







/**
 * @brief 本函数用于判定当前是否还有空间可用于多帧发送
 * @return int8_t
 * 1:有
 * 0:没有
 */
int8_t hcan_is_multi_frame_buffer_avilable(void)
{
    int i;
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_tx[i].used != 0) {
            continue;
        } else {
            break;
        }
    }
    if (i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH) {
        return 1;
    } else {
        return 0;
    }
}






/* 检查是否有发送给相同节点的同优先级的多帧在发送，
    如果有正在发送，则当前多帧不能发送，
    会导致多帧判断失败。
    0：没有
    1：有
*/
int8_t hcan_packet_is_transmiting(uint8_t priority, uint8_t taget_id, uint8_t self_id)
{
    int i;
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_tx[i].used != 0 &&
            hcan_multi_frame_tx[i].prio == priority &&
            hcan_multi_frame_tx[i].src == self_id &&
            hcan_multi_frame_tx[i].dest == taget_id) {
            break;
        }
    }
    if (i >= HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH) {
        return 0;
    }
    return 1;
}







static multi_frame_tx_t *hcan_get_empty_tx_buffer(void)
{
    int i;
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_tx[i].used == 0) {
            return &(hcan_multi_frame_tx[i]);
        }
    }
    return NULL;
}






/**
 * @brief 本接口将待发送的数据拆包，缓存到待发送的fifo，等待发送
 * @param  priority         包优先级
 * @param  taget_id         目标节点
 * @param  self_id          源节点
 * @param  pdata            数据
 * @param  data_len         数据长度
 * @return int8_t
 * -1: 参数错误
 * -2: 相同的多帧正在发送
 * -3: 没有空间继续发送多帧
 * -4: 写入buffer错误
 */
int8_t hcan_multi_packet_transmit(uint8_t priority, uint8_t taget_id, uint8_t self_id, uint8_t *pdata, uint16_t data_len)
{
    uint8_t data[HCAN_HW_FRAME_SIZE] = {0};
    uint16_t crc = 0;
    multi_frame_tx_t *empty_buffer = hcan_get_empty_tx_buffer();
    if (empty_buffer == NULL) {
        return -3;
    }
    if (hcan_packet_is_transmiting(priority, taget_id, self_id) != 0) {
        return -2;
    }
    empty_buffer->used = 1;

    empty_buffer->prio = priority;
    empty_buffer->src = self_id;
    empty_buffer->dest = taget_id;
    empty_buffer->total_data_len = data_len - HCAN_MULTI_PACKET_DATA_OFFSET;
    /*打包当前帧为首帧*/
    empty_buffer->current_tx_frame.dlc = HCAN_HW_FRAME_SIZE;
    empty_buffer->current_tx_frame.rolling_seq = 0;
    empty_buffer->current_tx_frame.prio = priority;
    empty_buffer->current_tx_frame.src = self_id;
    empty_buffer->current_tx_frame.dest = taget_id;
    empty_buffer->current_tx_frame.flag = MULTIPLE_FRAME_FIRST;

    empty_buffer->current_tx_frame.data[0] = pdata[0];
    empty_buffer->current_tx_frame.data[1] = pdata[1];
    empty_buffer->current_tx_frame.data[2] = (uint8_t)(((data_len - HCAN_MULTI_PACKET_DATA_OFFSET) & 0xFF00) >> 8); /*数据长度不包含功能码*/
    empty_buffer->current_tx_frame.data[3] = (uint8_t)((data_len - HCAN_MULTI_PACKET_DATA_OFFSET) & 0xFF);
    crc = crc16_calc(&pdata[2], data_len - HCAN_MULTI_PACKET_DATA_OFFSET);/* CRC */
    empty_buffer->current_tx_frame.data[4] = (uint8_t)((crc & 0xFF00) >> 8);
    empty_buffer->current_tx_frame.data[5] = (uint8_t)(crc & 0xFF);

    memset(empty_buffer->data, 0, data_len - HCAN_MULTI_PACKET_DATA_OFFSET);
    memcpy(empty_buffer->data, pdata + HCAN_MULTI_PACKET_DATA_OFFSET, data_len - HCAN_MULTI_PACKET_DATA_OFFSET);

    empty_buffer->send_stm = FRAME_SEND_FF;
    return 0;
}







/**
 * @brief 多帧发送管理
 * @return int8_t
* 0: 没有故障
* -3: 数据传输故障， 反馈了异常反馈码
 */
static int8_t hcan_fisrt_frame_transmit(uint8_t tcm_index)
{
    multi_frame_tx_t *tx_buffer = &(hcan_multi_frame_tx[tcm_index]);

    uint32_t id = HCAN_ID(tx_buffer->prio, MULTIPLE_FRAME_FIRST, 0, tx_buffer->dest, tx_buffer->src);
    /*多帧FIFO专用，发送首帧之前有已经有流控反馈之前数据发送完成，FIFO为空
        首帧又通过流控帧反馈来兜底， 不再对是否能写入做判断*/
    hcan_driver_send_multi_frame(id, tx_buffer->current_tx_frame.data, tcm_index);

    if (tx_buffer->ff_retry_cnt == 0) {
        tx_buffer->ff_retry_cnt = 1;
        tx_buffer->rolling_cnt = 1;
        tx_buffer->send_data_len_bs = 0;
        tx_buffer->send_data_len = 0;
        tx_buffer->tx_timer.t_h_ds_en = 1;
        tx_buffer->tx_timer.t_h_ds = 0;
        tx_buffer->send_stm = FRAME_WAIT_FC;
    } else {
        tx_buffer->send_stm = FRAME_WAIT_FC;
        tx_buffer->tx_timer.t_h_ds = 0;
    }
    return 0;
}







static int8_t hcan_continuous_frame_transmit(uint8_t tcm_index)
{
    uint32_t id;
    multi_frame_tx_t *tx = &(hcan_multi_frame_tx[tcm_index]);

    if (tx->fc_result_code < 0 && tx->seg_tx_para.update_flag != 0) {
        /*传输过程中故障*/
        tx->current_tx_frame.data[6] = tx->fc_result_code;
//        hcan_msg_error_handle_notify(HCAN_ERROR_CF_INTERRUPT, &(tx->current_tx_frame), sizeof(hcan_frame_t));
        memset(tx, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
        return -3;
    }
    tx->seg_tx_para.send_period_control += HCAN_LOOP_PERIOD;
    if (tx->seg_tx_para.send_period_control >= tx->seg_tx_para.stmin) {
        tx->seg_tx_para.send_period_control = 0;
        if (tx->hw_fifo_busy_cnt == 0) {
            /*如果多帧的连续帧发送没有出现硬件忙，
                则获取数据，更新翻滚计数器，
                否则，不更新数据，也不更新翻滚计数器*/
            tx->current_tx_frame.rolling_seq = tx->rolling_cnt++;
            if (tx->rolling_cnt >= 0x20) {
                tx->rolling_cnt = 1;
            }
            memset(tx->current_tx_frame.data, 0, HCAN_HW_FRAME_SIZE);
            memcpy(tx->current_tx_frame.data, &(tx->data[tx->send_data_len]), HCAN_HW_FRAME_SIZE);
        }
        id = HCAN_ID(tx->prio, MULTIPLE_FRAME_OTHERS, tx->current_tx_frame.rolling_seq,
                     tx->dest, tx->src);
        if (0 == hcan_driver_send_multi_frame(id, tx->current_tx_frame.data, tcm_index)) {
            /*数据写入成功，更新数据偏移与发送计数*/
            tx->hw_fifo_busy_cnt = 0;
            tx->send_data_len += HCAN_HW_FRAME_SIZE;
            tx->send_data_len_bs += HCAN_HW_FRAME_SIZE;
            if (tx->send_data_len < tx->total_data_len) {
                if (tx->send_data_len_bs >= tx->seg_tx_para.bs) {
                    tx->send_stm = FRAME_WAIT_FC;
                }
            } else {
                tx->send_stm = FRAME_WAIT_FC;
            }
        } else {
            tx->hw_fifo_busy_cnt++;
            if (tx->hw_fifo_busy_cnt > HCAN_HW_BUSY_RETRY_CNT_MAX) {
                memset(tx, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
                hcan_msg_error_handle_notify(HCAN_ERROR_WRITE_FIFO, NULL, 0);
            }
        }
    }
    return 0;
}






static int8_t hcan_handle_flow_control(uint8_t tcm_index)
{
    multi_frame_tx_t *tx_buffer = &(hcan_multi_frame_tx[tcm_index]);

    if (tx_buffer->seg_tx_para.update_flag != 0) {
        tx_buffer->seg_tx_para.update_flag = 0;
        if (tx_buffer->fc_result_code == HCAN_FB_CODE_RECV_FF || tx_buffer->fc_result_code == HCAN_FB_CODE_RECV_BS) {
            if (tx_buffer->fc_result_code == HCAN_FB_CODE_RECV_FF) {
                memset(&tx_buffer->tx_timer, HCAN_DEFAULT_ZERO, sizeof(tx_seg_frame_timer_t));
                tx_buffer->tx_timer.t_h_bs_en = 1;
            } else {
                tx_buffer->send_data_len_bs = 0;
                tx_buffer->tx_timer.t_h_bs = 0;
            }
            tx_buffer->send_stm = FRAME_SEND_CF;
        } else if (tx_buffer->fc_result_code == HCAN_FB_CODE_RECV_CPLT) {
            hcan_ack_fc_notify(NOTIFY_MULTI_FRAME_TX_CMPLT, tx_buffer->current_tx_frame.prio,
                               tx_buffer->current_tx_frame.dest,
                               tx_buffer->current_tx_frame.src,
                               tx_buffer->current_tx_frame.data, 8);
//            hcan_msg_error_handle_notify(HCAN_ERROR_NONE, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
            memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
        } else if (tx_buffer->fc_result_code == HCAN_FB_CODE_RECV_BUSY) {
            /*接收端忙*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_RECVER_BUSY, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
        } else if (tx_buffer->fc_result_code == HCAN_FB_CODE_CRC_ERROR) {
            /*接收端检测到CRC异常*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_CRC_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
        }
    } else {
        /*首帧应答检测使能，并检测到超时*/
        if (tx_buffer->tx_timer.t_h_ds_en != 0 && tx_buffer->tx_timer.ff_fc_ovtim_flag != 0) {
            tx_buffer->ff_retry_cnt++;
            tx_buffer->tx_timer.t_h_ds = 0;
            tx_buffer->tx_timer.ff_fc_ovtim_flag = 0;
            tx_buffer->send_stm = FRAME_SEND_FF;
            if (tx_buffer->ff_retry_cnt > HCAN_FRAME_FF_RETRY_TIMES) {
                /*重试次数达到3次， 清空对应TCM， 放弃发送*/
//                hcan_msg_error_handle_notify(HCAN_ERROR_FF_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
//                port_trace("data send ff fail n");
                memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
            }
        } else if (tx_buffer->tx_timer.t_h_bs_en != 0 && tx_buffer->tx_timer.cf_fc_ovtim_flag != 0) {
            /*连续帧流控检测定时使能，并检测到超时，直接放弃发送，返回错误*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_CF_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
            memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
        } else if (tx_buffer->tx_timer.t_h_bs_en == 0 && tx_buffer->tx_timer.t_h_ds_en == 0) {
            /* 不应该到达这里*/
            memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
            hcan_msg_error_handle_notify(HCAN_ERROR_UNKOWN, NULL, 0);
        }
    }
    return 0;
}







int8_t hcan_multi_frame_transmit(void)
{
    int i;
    hcan_frame_t tx_frame = {0};
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_tx[i].used != 0) {
            switch (hcan_multi_frame_tx[i].send_stm) {
                case FRAME_SEND_FF:
                    hcan_fisrt_frame_transmit(i);
                    break;
                case FRAME_SEND_CF:
                    hcan_continuous_frame_transmit(i);
                    break;
                case FRAME_WAIT_FC:
                    hcan_handle_flow_control(i);
                    break;
                default:
                    break;
            }
        }
    }
    return 0;
}
