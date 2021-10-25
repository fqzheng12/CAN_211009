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
 *  �ο�����һ���֣�����CANͨ�Ź淶---ͨ�ű����봫��淶��
 *  �ο����ڶ����֣�����CANͨ�Ź淶---ʱ����Ӧ�ù淶��
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
    ��֡���͹���ģ��
*/
static single_frame_tx_t hcan_single_frame_tx = {0};





/*
    �ҵ���ǰ�յ�bufferλ��
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
 * @brief ��֡�����������͵�CAN����,
        ���ӿ�֧���������͵�֡���ģ�����ACK���ģ� ��������֡���ġ�
 * @param  priority     ���ȼ�
 * @param  dest         Ŀ��ڵ�
 * @param  src          Դ�ڵ�
 * @param  type         ֡����
                        ��������֡��ACK�� ACK֡�� ����֡
 * @param  is_need_ack  �Ƿ���ҪACK��
                        �����ҪACK�� ������Զ�����֡���Ͳ�����֡����Ϊ��֡��ҪACK����
                        �������ҪACK�� ��֡����Ϊ�������壬
 * @param  weight       ��֡Ȩ��
 *                      ����֡��β�����룬 Ȩ��֡��ͷ������
 *                      ���ͱ���ʱ��ͷ����ȡ
 *                      ���ڷ�Ȩ�أ�������֡������ͻ�ȡ������ͬ��ָ�룬���Բ�ʹ�û���
 * @param  pdata        ����
 * @param data_len      ���ݳ���
 * @return int8_t
 * 0: ��֡����ɹ�
 * -2: ��������������ʧ��
 */
int8_t hcan_add_single_frame_list(uint8_t priority, uint8_t dest, uint8_t src,
                                  hcan_frame_type_t type, uint8_t is_need_ack, uint8_t weight,
                                  uint8_t *pdata, uint16_t data_len)
{
    static uint8_t ack_seq = 0;
    single_frame_item_t *empty_buffer = hcan_get_empty_buffer();//�õ��յ�buffer

    if (empty_buffer == NULL) {
        return -2;
    }
    if (!PRIORITY_VERIFY(priority) || !HCAN_ID_VERIFY(dest) || !HCAN_ID_VERIFY(src) || !FRAME_TYPE_VERIFY(type) || pdata == NULL || data_len == 0) {
        return -1;//�������
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

    __disable_irq();/*�ر��жϣ�ȷ���������*/
    if (weight == HCAN_FRAME_NORMAL) {
        list_insert_tail(&(hcan_single_frame_tx.wait_send_head), &(empty_buffer->node));
    } else {
        list_insert(&(hcan_single_frame_tx.wait_send_head), &(empty_buffer->node));
    }
    __enable_irq();/*�ָ��ж�*/
    return 0;
}





/**
 * @brief ɾ���Ѿ�ACK��֡������ط�//�Ѿ�ACK���
 * @param  ack_frame        ACK֡
 * @return int8_t
 0:�����ɹ�
 -2:��ǰû��ƥ��,�Ǳ��ڵ��ID
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
 * @brief �ط���ʱ����//�����趨�����ڽ����ط�����
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
 * @brief CAN��֡���͹���//�����͵���Ҫ���߼�������ʵ��//�����ҪACK�ķ���֡���ڻص������յ���֡��ACK֮ǰ����һֱ�ط�������Ϊloop����
 * @return int8_t//����������ר�ŵĺ����ڽ��յ��ص���ɾ���ط�
 * 1:��������
 * 0: ��
 * -2: �޷���ȡ���������ܷ���
 *
 */
int8_t hcan_single_frame_transmit(void)
{
    uint32_t id;//ר�Ű�ID�ó���
    hcan_frame_t tx_frame = {0};//����һ���յķ���֡
    single_frame_item_t *single_frame = NULL;//��֡����ṹ��

    if (list_count(&hcan_single_frame_tx.wait_send_head) == 0) {
        return 0;//Ӧ���ǿ����б������ж��ٸ���Ԫ����
    }
    single_frame = list_entry(single_frame_item_t, list_first(&(hcan_single_frame_tx.wait_send_head)), node);//�ҵ���֡��������������Ǹ��ȴ����͵ĵ�֡����ṹ�壬�������ó���

    id = HCAN_ID(single_frame->frame.prio, single_frame->frame.flag, single_frame->frame.rolling_seq, single_frame->frame.dest, single_frame->frame.src);//ƴ��Ҫ���͵�֡��ID
    if (0 == hcan_driver_send_single_frame(id, single_frame->frame.data, single_frame->frame.dlc)) {//�����������Ҫ�Լ�д
        hcan_single_frame_tx.hw_fifo_busy_cnt = 0;//������ͳɹ����򽫷������еķ��ͷ�æ������0.
        list_delete(&(single_frame->node));//Ӧ���ǽ������еĵ�ǰ�ѷ��ͽڵ�ɾ��������ڵ����Ƿ���֡�����
        if (single_frame->frame.flag == SINGLE_FRAME_NEED_ACK) {//�������Ҫ��ʱACK��֡
            single_frame->retry_timer = 0;//��ʼ���ط�����Ϊ0
            single_frame->retry_cnt++;//�ط������Ӽ�
            if (single_frame->retry_cnt < HCAN_FRAME_NEED_ACK_RETRY_TIMES) {
                list_insert_tail(&(hcan_single_frame_tx.wait_ack_head), &(single_frame->node));
            } else {/*�ط�ʧ�ܣ��ͷŻ���*///�ط������Ѿ�������������ط�����
                single_frame->used = 0;
                single_frame->retry_cnt = 0;
            }
        } else {
            single_frame->used = 0;//����ҪACK��ֱ�Ӱ�����������
        }
    } else {//������Ͳ��ɹ�
        hcan_single_frame_tx.hw_fifo_busy_cnt++;//���ڷ�����
        if (hcan_single_frame_tx.hw_fifo_busy_cnt > HCAN_HW_BUSY_RETRY_CNT_MAX) {//��ʱ
            hcan_single_frame_tx.hw_fifo_busy_cnt = 0;//����
            hcan_msg_error_handle_notify(HCAN_ERROR_WRITE_FIFO, NULL, 0);//��һ���ص�
        }
    }
    return 1;
}





/**
* @brief  ��֡�����ͽӿڣ������ϲ��·��ĵ�����֡�����ݣ�д�뵥֡��������
 * @param  priority         ���ȼ�
 * @param  dest         Ŀ��ڵ��
 * @param  src          Դ�ڵ��
 * @param  is_ned_ack       �Ƿ���ҪACKӦ��
 * @param  weight           ��֡Ȩ��
 * @param  pdata            ����
 * @param  data_len         ���ݳ���
 * @return int8_t
 * 0: ���ͳɹ�
 * -2:��ǰ��������
 * -3:��������ȡʧ��
 */
int8_t hcan_single_packet_transmit(uint8_t priority, uint8_t dest,
                                   uint8_t src, uint8_t is_need_ack, uint8_t weight,
                                   uint8_t *pdata, uint16_t data_len)
{
    return hcan_add_single_frame_list(priority, dest, src,
                                      SINGLE_FRAME_NO_ACK, is_need_ack, weight,
                                      pdata, data_len);//ת�˸��ӿڣ���Ҫ���͵�֡���Ͱ���������֡���͹�����������ȥ��Ȩ�ظ߲�ǰ�棬Ȩ�صͲ���棬���͵ĵط�ÿ��ѭ�����������������ݷ���
}




								   
/**
 * @brief ������յ��ĵ�֡����,������hcan_loop�������ã�������յ���CAN֡//���յ��ĵ�֡���ݴ�������������з��ദ��
 * @param  frame            CAN����
 * @return int8_t
* 0: ����ɹ�
 */
int8_t hcan_single_frame_recv_handle(hcan_frame_t *frame)
{
    hcan_frame_t ack_frame;

    if (frame->flag == SINGLE_FRAME_ACK && HCAN_FRAME_DEST_VERIFY(frame->dest)) {
        /*����ACK��ɾ���ط�ָ��*/
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
        hcan_msg_recv_notify(frame->prio, frame->dest, frame->src, frame->data, frame->dlc);/* ִ�б��Ľ��ջص� */
    }
    return 0;
}







/**
 * @brief ��֡�����ʼ��
 */
void hcan_single_frame_handle_init(void)
{
    /*init fifo*/
    memset(&(hcan_single_frame_tx.list_buffer), HCAN_DEFAULT_ZERO, 32 * sizeof(single_frame_item_t));
    list_init(&(hcan_single_frame_tx.wait_send_head));
    list_init(&(hcan_single_frame_tx.wait_ack_head));
}

