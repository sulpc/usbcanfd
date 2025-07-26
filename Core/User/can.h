#ifndef _CAN_H_
#define _CAN_H_

#include "util_types.h"

typedef void (*can_notify_t)(uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len);

int  can_config(uint32_t nomi_bitrate, uint8_t nomi_sample, uint32_t data_bitrate, uint8_t data_sample);
int  can_connect(void);
int  can_unconnect(void);
int  can_send(uint32_t id, bool ide, bool fdf, bool brs, const uint8_t* data, uint8_t len);
void can_process(void);
void can_set_nofity(can_notify_t notify);

#endif
