// host slave link
#ifndef _HSL_H_
#define _HSL_H_

#include "util_types.h"

void hsl_init(void);
void hsl_process(void);
void hsl_usb_recv(uint8_t* data, uint16_t len);   // called by usb isr

#endif
