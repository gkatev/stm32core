#include <cstring>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include <core/types.h>
#include <core/lib/mini_printf.h>

#include "UART.h"

constexpr char radix_digits[] = {'0', '1', '2', '3', '4', '5',
	'6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

// MCU specific methods
// -----------------------------------------

#ifdef STM32F1
	_UART1 UART1;
	
	void _UART1::init(uint32_t bitrate) {
		rcc_periph_clock_enable(RCC_GPIOA);
		rcc_periph_clock_enable(RCC_AFIO);
		rcc_periph_clock_enable(RCC_USART1);
		
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
		
		gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);
		
		usart_set_baudrate(USART1, bitrate);
		usart_set_databits(USART1, 8);
		usart_set_parity(USART1, USART_PARITY_NONE);
		usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
		usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
		usart_set_mode(USART1, USART_MODE_TX_RX);
		
		nvic_enable_irq(NVIC_USART1_IRQ);
		usart_enable_rx_interrupt(USART1);
		
		usart_enable(USART1);
	}
	
	extern "C" void usart1_isr(void) {
		
		// RX Interrupt
		if((USART_CR1(USART1) & USART_CR1_RXNEIE)
				&& usart_get_flag(USART1, USART_SR_RXNE)) {
			
			UART1.rx_push(usart_recv(USART1));
		}
		
		// TX Interrupt
		if((USART_CR1(USART1) & USART_CR1_TXEIE)
				&& usart_get_flag(USART1, USART_SR_TXE)) {
			
			if(UART1.tx_has())
				usart_send(USART1, UART1.tx_pop());
			
			// Disable TX interrupt if no more data in queue
			if(!UART1.tx_has())
				usart_disable_tx_interrupt(USART1);
		}
	}
	
	void _UART1::write(uint8_t data) {
		while(tx_len == tx_size);
		
		CM_ATOMIC_BLOCK() {
			uint8_t index = (tx_start + tx_len) % tx_size;
			tx_buffer[index] = data;
			
			tx_len++;
			
			if(!(USART_CR1(USART1) & USART_CR1_TXEIE))
				usart_enable_tx_interrupt(USART1);
		}
	}
	
	void _UART1::write(void *data, size_t count) {
		for(size_t i = 0; i < count; i++)
			write(((uint8_t *) data)[i]);
	}
	
#else
	#error Please provide UART definitions for this MCU
#endif

// Methods used by ISRs
// -----------------------------------------

void UART_VIRTUAL::rx_push(uint8_t data) {
	if(rx_len == rx_size)
		return;
	
	size_t index = (rx_start + rx_len) % rx_size;
	rx_buffer[index] = data;
	
	rx_len++;
}

uint8_t UART_VIRTUAL::tx_pop() {
	if(tx_len == 0) return 0;
	
	uint8_t data = tx_buffer[tx_start];
	
	tx_start = (tx_start + 1) % tx_size;
	tx_len--;
	
	return data;
}

bool UART_VIRTUAL::tx_has() {
	return (tx_len > 0);
}

// Receive methods
// -----------------------------------------

uint8_t UART_VIRTUAL::read() {
	uint8_t data;
	
	while(rx_len == 0);
	
	CM_ATOMIC_BLOCK() {
		data = rx_buffer[rx_start];
		
		rx_start = (rx_start + 1) % rx_size;
		rx_len--;
	}
	
	return data;
}

size_t UART_VIRTUAL::available() {
	return rx_len;
}

uint8_t UART_VIRTUAL::peek() {
	uint8_t data;
	
	while(rx_len == 0);
	
	CM_ATOMIC_BLOCK() {
		data = rx_buffer[rx_start];
	}
	
	return data;
}

// Transmit buffer methods
// -----------------------------------------

size_t UART_VIRTUAL::tx_items() {
	return tx_len;
}

void UART_VIRTUAL::tx_flush() {
	while(tx_len != 0);
}

size_t UART_VIRTUAL::printf(const char *fmt, ...) {
	char printf_buffer[128];
	
	va_list va;
	va_start(va, fmt);
	
	size_t len = mini_vsnprintf(printf_buffer,
		sizeof printf_buffer, fmt, va);
	
	va_end(va);
	
	write((uint8_t *) printf_buffer, len);
	
	return len;
}

// -----------------------------------------

void UART_VIRTUAL::write_word(uint16_t data) {
	data = __builtin_bswap16(data);
	write((uint8_t *) &data, 2);
}

void UART_VIRTUAL::write_dword(uint32_t data) {
	data = __builtin_bswap32(data);
	write((uint8_t *) &data, 4);
}

uint16_t UART_VIRTUAL::read_word() {
	return ((uint16_t) read() << 8) | read();
}

uint32_t UART_VIRTUAL::read_dword() {
	return ((uint32_t) read() << 24) | ((uint32_t)read() << 16)
		| ((uint16_t) read() << 8) | read();
}
