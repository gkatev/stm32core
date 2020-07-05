#ifndef UART_H
#define UART_H

#include <core/types.h>

#define UART_TX_BUF_SIZE 128
#define UART_RX_BUF_SIZE 128

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class UART_VIRTUAL {
	protected:
		static constexpr size_t tx_size = UART_TX_BUF_SIZE;
		static constexpr size_t rx_size = UART_RX_BUF_SIZE;
		
		uint8_t tx_buffer[tx_size];
		uint8_t rx_buffer[rx_size];
		
		volatile size_t rx_start = 0, tx_start = 0;
		volatile size_t rx_len = 0, tx_len = 0;
	
	public:
		/* Not really meant to be called by the user.
		 * Public so that they can be called from ISRs.
		 * Also, not async-safe. */
		
		void rx_push(uint8_t data);
		uint8_t tx_pop();
		bool tx_has();
	
	public:
		bool initialized = false;
		
		virtual void init(uint32_t bitrate) = 0;
		
		virtual void write(uint8_t data) = 0;
		virtual void write(void *data, size_t count) = 0;
		uint8_t read();
		
		size_t available();
		uint8_t peek();
		
		size_t tx_items();
		void tx_flush();
		
		size_t printf(const char *fmt, ...);
		
		void write_word(uint16_t data);
		void write_dword(uint32_t data);
		
		uint16_t read_word();
		uint32_t read_dword();
};

#ifdef STM32F1
	extern class _UART1 : public UART_VIRTUAL {
		public:
			void init(uint32_t bitrate);
			
			void write(uint8_t data);
			void write(void *data, size_t count);
	} UART1;
	
#else
	#error Please provide UART definitions for this MCU
#endif

#endif
