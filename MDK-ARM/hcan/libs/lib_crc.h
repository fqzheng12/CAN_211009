#ifndef __CRC_H__
#define __CRC_H__

extern uint16_t crc16_calc(uint8_t *packet, int32_t length);
extern uint8_t crc8_calc(uint8_t *data, uint8_t length) ;
extern uint16_t crc16_ccitt(uint8_t *data, int32_t length);

#endif /* #ifndef __CRC_H__ */

