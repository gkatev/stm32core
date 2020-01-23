/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

/* @gkatev Used code from:
 * https://github.com/libopencm3/libopencm3-examples
 * https://github.com/dhylands/libopencm3-usb-serial */

#include <stdlib.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <libopencm3/stm32/st_usbfs.h>
#include <libopencm3/stm32/f1/bkp.h>
#include <libopencm3/stm32/pwr.h>

#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#include <core/types.h>
#include <core/lib/mini_printf.h>

#include "usb_vcp.h"

// ---------------------------------

#define USB_CDC_REQ_SEND_BREAK 0x23
#define DFU_MAGICWORD 0xF103DFB1

// ---------------------------------

static constexpr size_t vcp_tx_size = 1024;
static constexpr size_t vcp_rx_size = 1024;

static uint8_t vcp_tx_buf[1024];
static uint8_t vcp_rx_buf[1024];

static volatile size_t vcp_tx_start = 0, vcp_rx_start = 0;
static volatile size_t vcp_tx_len = 0, vcp_rx_len = 0;

static usbd_device *g_usbd_dev = nullptr;
static volatile bool g_usbd_is_connected = false;
static volatile bool g_tx_busy = false;
static volatile bool g_tx_zlp = false;

static char usb_serialnum[13];
static uint8_t usbd_control_buffer[128];

static const struct usb_device_descriptor dev = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = USB_CLASS_CDC,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5740,
	.bcdDevice = 0x0200,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

/* This notification endpoint isn't implemented. According to CDC
 * spec it's optional, but its absence causes a NULL pointer
 * dereference in the Linux cdc_acm driver. */
static const struct usb_endpoint_descriptor comm_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x83,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 16,
	.bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x01,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x82,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64,
	.bInterval = 1
}};

static const struct {
	struct usb_cdc_header_descriptor header;
	struct usb_cdc_call_management_descriptor call_mgmt;
	struct usb_cdc_acm_descriptor acm;
	struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) vcp_functional_descriptors = {
	.header = {
		.bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_HEADER,
		.bcdCDC = 0x0110
	},
	
	.call_mgmt = {
		.bFunctionLength =
			sizeof(struct usb_cdc_call_management_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
		.bmCapabilities = 0,
		.bDataInterface = 1
	},
	
	.acm = {
		.bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_ACM,
		.bmCapabilities = 0
	},
	
	.cdc_union = {
		.bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
		.bDescriptorType = CS_INTERFACE,
		.bDescriptorSubtype = USB_CDC_TYPE_UNION,
		.bControlInterface = 0,
		.bSubordinateInterface0 = 1
	 },
};

static const struct usb_interface_descriptor comm_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_CDC,
	.bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
	.bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
	.iInterface = 0,

	.endpoint = comm_endp,

	.extra = &vcp_functional_descriptors,
	.extralen = sizeof(vcp_functional_descriptors)
}};

static const struct usb_interface_descriptor data_iface[] = {{
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 1,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_DATA,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 0,

	.endpoint = data_endp
}};

static const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = comm_iface,
}, {
	.num_altsetting = 1,
	.altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 2,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0x80,
	// .bMaxPower = 0x32,
	.bMaxPower = 250, // 500 mA

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Manufacturer",
	"STM32F103 CDC ACM",
	usb_serialnum
};

// ---------------------------------

static void write_dfu_magicword();
static void kill_usb();

static void vcp_rx_callback(usbd_device *usbd_dev, uint8_t ep);
static void vcp_tx_callback(usbd_device *usbd_dev, uint8_t ep);
static void vcp_tx_push();

// ---------------------------------

static enum usbd_request_return_codes vcp_control_request(
		usbd_device *usbd_dev, struct usb_setup_data *req,
		uint8_t **buf, uint16_t *len, void (**complete)(usbd_device *usbd_dev,
		struct usb_setup_data *req)) {
	
	switch (req->bRequest) {
		
		case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
			g_usbd_is_connected = req->wValue & 1;
			
			return USBD_REQ_HANDLED;
		
		case USB_CDC_REQ_SET_LINE_CODING:
			if (*len < sizeof(struct usb_cdc_line_coding))
				return USBD_REQ_NOTSUPP;
			
			return USBD_REQ_HANDLED;
		
		case USB_CDC_REQ_SEND_BREAK:
			write_dfu_magicword();
			kill_usb();
			
			scb_reset_system();
			
			// Won't actually return
			return USBD_REQ_HANDLED;
	}
	
	return USBD_REQ_NOTSUPP;
}

static void vcp_rx_callback(usbd_device *usbd_dev, uint8_t ep) {
	size_t cont_bytes, sec_bytes;
	size_t empty_space_start;
	size_t len;
	
	if(vcp_rx_start + vcp_rx_len  < vcp_rx_size) {
		empty_space_start = vcp_rx_start + vcp_rx_len;
		cont_bytes = vcp_rx_size - empty_space_start;
		sec_bytes = vcp_rx_start;
	} else {
		empty_space_start = (vcp_rx_start + vcp_rx_len) % vcp_rx_size;
		cont_bytes = vcp_rx_start - empty_space_start;
		sec_bytes = 0;
	}
	
	if(cont_bytes >= 64) {
		len = usbd_ep_read_packet(usbd_dev, ep,
			vcp_rx_buf + empty_space_start, cont_bytes);
	} else {
		uint8_t buf[cont_bytes + sec_bytes];
		
		len = usbd_ep_read_packet(usbd_dev, ep,
			buf, sizeof buf);
		
		cont_bytes = (len > cont_bytes ? cont_bytes : len);
		sec_bytes = len - cont_bytes;
		
		memcpy(vcp_rx_buf + empty_space_start, buf, cont_bytes);
		memcpy(vcp_rx_buf, buf + cont_bytes, sec_bytes);
	}
	
	vcp_rx_len += len;
}

static void vcp_tx_callback(usbd_device *usbd_dev, uint8_t ep) {
	g_tx_busy = false;
}

static void vcp_tx_push() {
	size_t len, sent;
	
	len = vcp_tx_size - vcp_tx_start;
	if(len > vcp_tx_len) len = vcp_tx_len;
	
	len = (len <= 64 ? len : 64);
	
	if(g_tx_zlp) {
		if(len == 0)
			usbd_ep_write_packet(g_usbd_dev, 0x82, nullptr, 0);
		
		g_tx_zlp = false;
	}
	
	if(len == 0)
		return;
	
	sent = usbd_ep_write_packet(g_usbd_dev, 0x82,
		vcp_tx_buf + vcp_tx_start, len);
	
	vcp_tx_start = (vcp_tx_start + sent) % vcp_tx_size;
	vcp_tx_len -= sent;
	
	if(sent == 64 && vcp_tx_len == 0)
		g_tx_zlp = true;
}

static void vcp_set_config(usbd_device *usbd_dev, uint16_t wValue) {
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, vcp_rx_callback);
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, vcp_tx_callback);
	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, nullptr);
	
	usbd_register_control_callback(usbd_dev,
		USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
		USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
		vcp_control_request);
}

static void write_dfu_magicword() {
	rcc_periph_clock_enable(RCC_PWR);
	rcc_periph_clock_enable(RCC_BKP);
	
	pwr_disable_backup_domain_write_protect();
	
	BKP_DR1 = DFU_MAGICWORD >> 16;
	BKP_DR2 = DFU_MAGICWORD & 0xFFFF;
	
	pwr_enable_backup_domain_write_protect();
}

static void kill_usb() {
	*USB_CNTR_REG |= USB_CNTR_PWDN;
	
	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	gpio_clear(GPIOA, GPIO12);
	
	rcc_periph_clock_disable(RCC_USB);
}

/* This document: http://www.usb.org/developers/docs/devclass_docs/usbmassbulk_10.pdf
 * says that the serial number has to be at least 12 digits long and that
 * the last 12 digits need to be unique. It also stipulates that the valid
 * character set is that of upper-case hexadecimal digits.
 * 
 * The onboard DFU bootloader produces a 12-digit serial number based on
 * the 96-bit unique ID, so for consistency we go with this algorithm.
 * You can see the serial number with dfu-util -l
 * 
 * See: https://my.st.com/52d187b7 for the algorithim used. */
static void calculate_usb_serialnum() {
    uint8_t *id = (uint8_t *)DESIG_UNIQUE_ID_BASE;

	uint8_t serial[6];
	serial[0] = id[11];
	serial[1] = id[10] + id[2];
	serial[2] = id[9];
	serial[3] = id[8] + id[0];
	serial[4] = id[7];
	serial[5] = id[6];

	uint8_t *ser = &serial[0];
	uint8_t *end = &serial[6];
	char *ser_str = usb_serialnum;
	const char hex_digit[] = "0123456789ABCDEF";

	for (; ser < end; ser++) {
		*ser_str++ = hex_digit[(*ser >> 4) & 0x0f];
		*ser_str++ = hex_digit[(*ser >> 0) & 0x0f];
	}
	*ser_str = '\0';
}

// ---------------------------------

extern "C" void usb_lp_can_rx0_isr(void) {
	usbd_poll(g_usbd_dev);
	
	if((vcp_tx_len || g_tx_zlp) && g_usbd_is_connected && !g_tx_busy) {
		vcp_tx_push();
		g_tx_busy = true;
	}
}

void vcp_init() {
	calculate_usb_serialnum();
	
	g_usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config,
		usb_strings, sizeof(usb_strings)/sizeof(usb_strings[0]),
		usbd_control_buffer, sizeof(usbd_control_buffer));
	
	usbd_register_set_config_callback(g_usbd_dev, vcp_set_config);
	
	*USB_CNTR_REG &= USB_CNTR_CTRM | USB_CNTR_RESETM | 0x00FF;
	
	nvic_set_priority(NVIC_USB_LP_CAN_RX0_IRQ, 0x88);
	nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
}

/* void vcp_poll() {
	usbd_poll(g_usbd_dev);
} */

bool vcp_is_connected() {
	return g_usbd_is_connected;
}

size_t vcp_available() {
	return vcp_rx_len;
}

uint8_t vcp_read() {
	CM_ATOMIC_CONTEXT();
	
	if(vcp_rx_len == 0)
		return 0;
	
	uint8_t data = vcp_rx_buf[vcp_rx_start];
	
	vcp_rx_start = (vcp_rx_start + 1) % vcp_rx_size;
	vcp_rx_len--;
	
	return data;
}

// In case of overflow, the old data will be overwritten
void vcp_write(uint8_t data) {
	CM_ATOMIC_CONTEXT();
	
	size_t idx = (vcp_tx_start + vcp_tx_len) % vcp_tx_size;
	vcp_tx_buf[idx] = data;
	
	if(vcp_tx_len < vcp_tx_size) vcp_tx_len++;
	else vcp_tx_start = (vcp_tx_start + 1) % vcp_tx_size;
	
	if(g_usbd_is_connected && !g_tx_busy)
		nvic_generate_software_interrupt(NVIC_USB_LP_CAN_RX0_IRQ);
}

size_t vcp_printf(const char *fmt, ...) {
	iovec_t iov[2];
	
	CM_ATOMIC_CONTEXT();
	
	size_t empty_space_start = (vcp_tx_start + vcp_tx_len) % vcp_tx_size;
	
	if(vcp_tx_start + vcp_tx_len < vcp_tx_size) {
		iov[0] = {
			.iov_base = vcp_tx_buf + empty_space_start,
			.iov_len = vcp_tx_size - empty_space_start
		};
		
		iov[1] = {
			.iov_base = vcp_tx_buf,
			.iov_len = vcp_tx_start
		};
	} else {
		iov[0] = {
			.iov_base = vcp_tx_buf + empty_space_start,
			.iov_len = vcp_tx_start - empty_space_start
		};
		
		iov[1] = {};
	}
	
	va_list va;
	va_start(va, fmt);
	
	size_t len = mini_vsnprintf(iov, 2, fmt, va);
	va_end(va);
	
	vcp_tx_len += len;
	
	if(g_usbd_is_connected && !g_tx_busy)
		nvic_generate_software_interrupt(NVIC_USB_LP_CAN_RX0_IRQ);
	
	return len;
}
