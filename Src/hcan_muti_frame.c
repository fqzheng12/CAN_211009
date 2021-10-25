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
 * @brief CAN�շ���֡����ģ��//��Ҫ˼�뻹���Ƚ�����ѭ�����͵�ͨ����һ����������Ҫ���͵����ݲ嵽�����棬ѭ���������ͻ��߽���
 * 1����֡�����ͽ���
 * 2����֡֡���շ���
 * 3����֡Э�鴦��
 * @author zhangqinghang (zhangqinghang705@hellobike.com)
 * @version 1.1
 * @date 2021-08-23
 *  �ο�����һ���֣�����CANͨ�Ź淶---ͨ�ű����봫��淶��
 *  �ο����ڶ����֣�����CANͨ�Ź淶---ʱ����Ӧ�ù淶��
 * @copyright Copyright (c) 2021  Hello Bike



 
 *
 */
#include "can.h"
#include "hcan_private.h"
#include "hcan.h"
#include "string.h"
#include "lib_crc.h"
/*��֡�շ����������ģ��*///�շ��Ļ��������ʼ��
static multi_frame_tx_t hcan_multi_frame_tx[HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH];
static multi_frame_rx_t hcan_multi_frame_rx[HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH];
static uint16_t recv_bs_in_byte = (HCAN_HW_FRAME_SIZE *HCAN_BS);//����һ��bs�ж��ٸ��ֽ�



/**
 * @brief ��ʼ����֡�����ӿ�
 */
void hcan_multi_frame_handle_init(void)
{
    memset(&hcan_multi_frame_rx[0], HCAN_DEFAULT_ZERO, (HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH)*sizeof(multi_frame_rx_t));//�����涨������鿪ʼ��ַ���������ȫ����ֵΪĬ��canֵ��
    memset(&(hcan_multi_frame_tx[0]), HCAN_DEFAULT_ZERO, (HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH)*sizeof(multi_frame_tx_t));
    recv_bs_in_byte = (HCAN_HW_FRAME_SIZE * HCAN_BS);
}




/**
 * @brief ���ݽ��յ�������֡��������Ӧ�ķ��Ͳ���
 * @param  frame            ����������֡
 * @return int8_t
 * 0: �����ɹ�
 * -1: ��������
 * -2: �Ƿ�����֡
 */
static int8_t hcan_adjust_send_parameter(hcan_frame_t *frame)//��������Ӧ���Ǹ�����֡
{
    int i;
    if (frame == NULL) {//һ�������������Σ�������ͷ�����
        return -1;
    }
    for (i = 0; i < HCAN_TX_MULTIPLE_FRAME_LIST_LENGTH; i++) {//��֡���еĳ���//���û����
        if (hcan_multi_frame_tx[i].dest == frame->src && hcan_multi_frame_tx[i].src == frame->dest && hcan_multi_frame_tx[i].prio == frame->prio) {
            /*���·��Ͳ���*/
            hcan_multi_frame_tx[i].seg_tx_para.bs = HCAN_HW_FRAME_SIZE * frame->data[0];
            hcan_multi_frame_tx[i].seg_tx_para.stmin = frame->data[1];
            hcan_multi_frame_tx[i].seg_tx_para.br = 10 * frame->data[2];/*BR ����ʱ�䵥λΪ10����*/
            hcan_multi_frame_tx[i].fc_result_code = frame->data[3];
            hcan_multi_frame_tx[i].seg_tx_para.update_flag = 1;/*֪ͨ�����Ѿ�����*/
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
 * @brief ���������ڷ�������֡
 * @param  prio             ���ȼ�
 * @param  dest        Ŀ��ڵ�
 * @param  src           Դ�ڵ�
 * @param  bs               block size�� ���С
 * @param  stmin            ����֡��֡���
 * @param  br               ���ط��������ʱ
 * @param  fb_code          ������
 * @return int8_t
 * 0: ���ͳɹ�
 * -2: ����ʧ��
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







static multi_frame_rx_t *hcan_get_empty_recv_buffer(void)//�ҵ��յĽ���λ��
{
    int i;
    for (i = 0; i < HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH; i++) {
        if (hcan_multi_frame_rx[i].used == 0) {
            return &(hcan_multi_frame_rx[i]);
        }
    }
    return NULL;
}







static int8_t hcan_handle_recv_first_frame(hcan_frame_t *frame)//������֡
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
    hcan_can_send_flow_control_frame(frame->prio, frame->src, frame->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_FF);/*��������֡���Ѿ����յ���֡*/
    memset(&(empty_buffer->rx_timer), HCAN_DEFAULT_ZERO, sizeof(rx_seg_frame_timer_t));/*���ݽ��յ�����֡*/
    empty_buffer->rx_timer.t_h_cr_en = 1;/*������֡�����ʱ��ʱ*/

    return 0;
}







static multi_frame_rx_t *hcan_get_matched_recv_buffer(hcan_frame_t *frame)
{
    int i;
    for (i = 0; i < HCAN_RX_MULTIPLE_FRAME_LIST_LENGTH; i++) {//���ն�֡�б���
        /*����֡�������Ѿ�����õĻ������*/
        if (hcan_multi_frame_rx[i].prio == frame->prio &&
            hcan_multi_frame_rx[i].src == frame->src &&
            hcan_multi_frame_rx[i].dest == frame->dest) {
            return &(hcan_multi_frame_rx[i]);
        }
    }
    return NULL;
}






static int8_t hcan_handle_recv_continuous_frame(hcan_frame_t *frame)//��������֡
{
    multi_frame_rx_t *matched_buffer = hcan_get_matched_recv_buffer(frame);//����Ѿ�ƥ��õ�һ֡
    if (matched_buffer == NULL) {
        hcan_can_send_flow_control_frame(frame->prio, frame->src, frame->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_BUSY);
        return -2;
    }
    if (frame == NULL) {
        return -1;
    }
    matched_buffer->rx_timer.t_h_cr = 0;/*��������֡����� CR��ʱ��*/
    if (frame->rolling_seq - matched_buffer->rolling_cnt == 1 || (frame->rolling_seq == 1 && matched_buffer->rolling_cnt == 0x1F)) {
        matched_buffer->rolling_cnt = frame->rolling_seq;
    } else {
        /*��������֡������˳�����*/
//							printf("�����˸Ĵ���\r\n");
        hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_SEQ_ERROR);
        memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));/*��յ�ǰ��֡����*/
        return -3;
    }
    memcpy(&(matched_buffer->rx_buffer[matched_buffer->recv_data_len + HCAN_MULTI_PACKET_DATA_OFFSET]), frame->data, HCAN_HW_FRAME_SIZE);
    matched_buffer->recv_data_len += HCAN_HW_FRAME_SIZE;
    matched_buffer->recv_len_bs += HCAN_HW_FRAME_SIZE;

    if (matched_buffer->recv_data_len >= matched_buffer->total_data_len) {
        /*�����ճ���*/
        if (crc16_calc((matched_buffer->rx_buffer + HCAN_MULTI_PACKET_DATA_OFFSET), matched_buffer->total_data_len) == matched_buffer->crc_value) {
            /*CRC У��*/
            if (0 == hcan_msg_recv_notify(matched_buffer->prio,
                                          matched_buffer->dest, matched_buffer->src,
                                          matched_buffer->rx_buffer, matched_buffer->total_data_len + 2)) {
                hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_CPLT);/*��������֡��ȫ�����ݴ������*/
            } else {
                hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_HANDLE_ERROR);/*��������֡�����ݴ���ʧ��*/
            }
            memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
        } else {
            /*��������֡��CRC У�����*/
            hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_CRC_ERROR);
            memset(matched_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
        }
    } else {
        if (matched_buffer->recv_len_bs == recv_bs_in_byte) {
            /*��������֡�����յ�BS���ȵ�����*/
            matched_buffer->recv_len_bs = 0;
            hcan_can_send_flow_control_frame(matched_buffer->prio, matched_buffer->src, matched_buffer->dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_RECV_BS);
        }
    }

    return 0;
}







/**
 * @brief ��֡���մ����������ڽ����յ��Ķ�֡����ƴ����
 * @param  frame           ���յ��Ķ�֡
 * @return int8_t
 * 0: ���մ���ɹ�
 * -1:��������
 * -3:CAN֡���Ǳ��ڵ�Ķ�֡
 */
int8_t hcan_multi_frame_recv_handle(hcan_frame_t *frame)
{
    if (frame == NULL) {
        return -1;
    }
    if (!HCAN_FRAME_DEST_VERIFY(frame->dest)) {
        /*�����֡����Ŀ�겢�Ǳ��ڵ㣬ֱ�Ӷ���*/
        return -3;
    }
    if (frame->flag == MULTIPLE_FRAME_FIRST) {
        hcan_handle_recv_first_frame(frame);
    } else if (frame->flag == MULTIPLE_FRAME_OTHERS) {
        hcan_handle_recv_continuous_frame(frame);
    } else {  
        hcan_adjust_send_parameter(frame);/*���յ�����֡����������֡����*/
    }
    return 0;
}






/**
 * @brief ���������ڶԶ�֡�շ��Ķ�ʱ�����й���
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
                /*����֡������ã�����ʧ��*/
                hcan_multi_frame_rx[i].rx_timer.cf_ovtim_flag = 1;
                hcan_can_send_flow_control_frame(hcan_multi_frame_rx[i].prio, hcan_multi_frame_rx[i].src, hcan_multi_frame_rx[i].dest, HCAN_BS, HCAN_STMIN, HCAN_H_BR, HCAN_FB_CODE_STOP);
                memset(&hcan_multi_frame_rx[i], HCAN_DEFAULT_ZERO, sizeof(multi_frame_rx_t));
            }
        }
    }
}







/**
 * @brief �����������ж���ǰ�Ƿ��пռ�����ڶ�֡����
 * @return int8_t
 * 1:��
 * 0:û��
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






/* ����Ƿ��з��͸���ͬ�ڵ��ͬ���ȼ��Ķ�֡�ڷ��ͣ�
    ��������ڷ��ͣ���ǰ��֡���ܷ��ͣ�
    �ᵼ�¶�֡�ж�ʧ�ܡ�
    0��û��
    1����
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
 * @brief ���ӿڽ������͵����ݲ�������浽�����͵�fifo���ȴ�����
 * @param  priority         �����ȼ�
 * @param  taget_id         Ŀ��ڵ�
 * @param  self_id          Դ�ڵ�
 * @param  pdata            ����
 * @param  data_len         ���ݳ���
 * @return int8_t
 * -1: ��������
 * -2: ��ͬ�Ķ�֡���ڷ���
 * -3: û�пռ�������Ͷ�֡
 * -4: д��buffer����
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
    /*�����ǰ֡Ϊ��֡*/
    empty_buffer->current_tx_frame.dlc = HCAN_HW_FRAME_SIZE;
    empty_buffer->current_tx_frame.rolling_seq = 0;
    empty_buffer->current_tx_frame.prio = priority;
    empty_buffer->current_tx_frame.src = self_id;
    empty_buffer->current_tx_frame.dest = taget_id;
    empty_buffer->current_tx_frame.flag = MULTIPLE_FRAME_FIRST;

    empty_buffer->current_tx_frame.data[0] = pdata[0];
    empty_buffer->current_tx_frame.data[1] = pdata[1];
    empty_buffer->current_tx_frame.data[2] = (uint8_t)(((data_len - HCAN_MULTI_PACKET_DATA_OFFSET) & 0xFF00) >> 8); /*���ݳ��Ȳ�����������*/
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
 * @brief ��֡���͹���
 * @return int8_t
* 0: û�й���
* -3: ���ݴ�����ϣ� �������쳣������
 */
static int8_t hcan_fisrt_frame_transmit(uint8_t tcm_index)
{
    multi_frame_tx_t *tx_buffer = &(hcan_multi_frame_tx[tcm_index]);

    uint32_t id = HCAN_ID(tx_buffer->prio, MULTIPLE_FRAME_FIRST, 0, tx_buffer->dest, tx_buffer->src);
    /*��֡FIFOר�ã�������֮֡ǰ���Ѿ������ط���֮ǰ���ݷ�����ɣ�FIFOΪ��
        ��֡��ͨ������֡���������ף� ���ٶ��Ƿ���д�����ж�*/
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
        /*��������й���*/
        tx->current_tx_frame.data[6] = tx->fc_result_code;
//        hcan_msg_error_handle_notify(HCAN_ERROR_CF_INTERRUPT, &(tx->current_tx_frame), sizeof(hcan_frame_t));
        memset(tx, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
        return -3;
    }
    tx->seg_tx_para.send_period_control += HCAN_LOOP_PERIOD;
    if (tx->seg_tx_para.send_period_control >= tx->seg_tx_para.stmin) {
        tx->seg_tx_para.send_period_control = 0;
        if (tx->hw_fifo_busy_cnt == 0) {
            /*�����֡������֡����û�г���Ӳ��æ��
                ���ȡ���ݣ����·�����������
                ���򣬲��������ݣ�Ҳ�����·���������*/
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
            /*����д��ɹ�����������ƫ���뷢�ͼ���*/
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
            /*���ն�æ*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_RECVER_BUSY, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
        } else if (tx_buffer->fc_result_code == HCAN_FB_CODE_CRC_ERROR) {
            /*���ն˼�⵽CRC�쳣*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_CRC_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
        }
    } else {
        /*��֡Ӧ����ʹ�ܣ�����⵽��ʱ*/
        if (tx_buffer->tx_timer.t_h_ds_en != 0 && tx_buffer->tx_timer.ff_fc_ovtim_flag != 0) {
            tx_buffer->ff_retry_cnt++;
            tx_buffer->tx_timer.t_h_ds = 0;
            tx_buffer->tx_timer.ff_fc_ovtim_flag = 0;
            tx_buffer->send_stm = FRAME_SEND_FF;
            if (tx_buffer->ff_retry_cnt > HCAN_FRAME_FF_RETRY_TIMES) {
                /*���Դ����ﵽ3�Σ� ��ն�ӦTCM�� ��������*/
//                hcan_msg_error_handle_notify(HCAN_ERROR_FF_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
//                port_trace("data send ff fail n");
                memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
            }
        } else if (tx_buffer->tx_timer.t_h_bs_en != 0 && tx_buffer->tx_timer.cf_fc_ovtim_flag != 0) {
            /*����֡���ؼ�ⶨʱʹ�ܣ�����⵽��ʱ��ֱ�ӷ������ͣ����ش���*/
//            hcan_msg_error_handle_notify(HCAN_ERROR_CF_FAIL, &(tx_buffer->current_tx_frame), sizeof(hcan_frame_t));
            memset(tx_buffer, HCAN_DEFAULT_ZERO, sizeof(multi_frame_tx_t));
        } else if (tx_buffer->tx_timer.t_h_bs_en == 0 && tx_buffer->tx_timer.t_h_ds_en == 0) {
            /* ��Ӧ�õ�������*/
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
