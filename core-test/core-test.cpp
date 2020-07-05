#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <core/usb_vcp.h>
#include <core/UART.h>
#include <core/millis.h>

#include <core/lib/mini_printf.h>

int main() {
	rcc_clock_setup_in_hse_8mhz_out_72mhz();
	
	UART1.init(115200);
	millis_init();
	
	vcp_init();
	
	// -----------------------
	
	char buf[50] = {};
	
	iovec_t iov[5] = {
		{buf, 10},
		{buf+10, 10},
		{buf+20, 10},
		{buf+30, 10},
		{buf+40, 9}
	};
	
	size_t len = mini_snprintf(iov, 5, "Hey there! This is a number: %d. %s.\n", -13, "The end");
	UART1.write(buf, len);
	
	UART1.printf("Hey there! This is a number: %d. %s.\n", -13, "The end");
	
	// Workaround to avoid linux's echoing while the serial port is opening
	while(!vcp_is_connected());
	delay_ms(10);
	
	vcp_printf("Hello from printf!\n");
	vcp_printf("Hey there! This is a number: %d. %s.\n", -13, "The end");
	
	// -----------------------
	
	char sbuf[5];
	iovec_t i = {sbuf, 5};
	
	mini_snprintf(&i, 1, "%s\n", "Looooooooong");
	
	for(size_t i = 0; i < 5; i++)
		vcp_write(sbuf[i]);
	
	// -----------------------
	
	while(1) {
		if(vcp_available()) {
			char mbuf[64];
			size_t b = 0;
			
			delay_ms(1);
			
			while(vcp_available() && b < 63)
				mbuf[b++] = vcp_read();
			
			mbuf[b] = '\0';
			
			vcp_printf("Message: %s\n", mbuf);
		}
	}
}
