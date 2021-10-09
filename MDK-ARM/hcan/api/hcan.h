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




typedef struct hcan_frame_filter_s
{
    uint32_t mask;
    uint32_t can_id;
}hcan_frame_filter_t;


#endif /* __HCAN_H__ */
