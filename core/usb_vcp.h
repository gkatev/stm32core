#ifndef USB_VCP_H
#define USB_VCP_H

#include <core/types.h>

void vcp_init();
// void vcp_poll();

bool vcp_is_connected();
size_t vcp_available();

uint8_t vcp_read();
void vcp_write(uint8_t data);

size_t vcp_printf(const char *fmt, ...);

#endif
