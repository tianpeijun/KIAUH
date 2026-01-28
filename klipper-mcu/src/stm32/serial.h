/**
 * @file    serial.h
 * @brief   STM32F407 USART serial driver interface
 * 
 * Provides serial communication functions for STM32F407.
 * Used by gcode.c for G-code input and debug output.
 * Supports line-buffered input for G-code parsing.
 * Follows Klipper coding style (C99, snake_case).
 */

#ifndef STM32_SERIAL_H
#define STM32_SERIAL_H

#include <stdint.h>
#include <stddef.h>

/* ========== Serial Configuration ========== */

/* Default baud rate */
#define SERIAL_BAUD_DEFAULT     115200

/* Buffer sizes */
#define SERIAL_RX_BUFFER_SIZE   256     /* Receive buffer size */
#define SERIAL_TX_BUFFER_SIZE   256     /* Transmit buffer size */
#define SERIAL_LINE_BUFFER_SIZE 128     /* Line buffer for G-code */

/* USART selection */
typedef enum {
    SERIAL_USART1 = 0,          /* USART1: PA9(TX), PA10(RX) */
    SERIAL_USART2,              /* USART2: PA2(TX), PA3(RX) */
    SERIAL_USART3,              /* USART3: PB10(TX), PB11(RX) */
    SERIAL_COUNT
} serial_port_t;

/* Serial configuration structure */
typedef struct {
    serial_port_t port;         /* USART port selection */
    uint32_t baud;              /* Baud rate */
} serial_config_t;

/* ========== Serial Functions ========== */

/**
 * @brief   Initialize serial subsystem
 * 
 * Initializes the default USART (USART1) with 115200 baud rate.
 * Configures GPIO pins for alternate function.
 * Enables receive interrupt for line-buffered input.
 */
void serial_init(void);

/**
 * @brief   Initialize serial with configuration
 * @param   config  Serial configuration
 * @retval  0 on success, negative on error
 */
int serial_init_config(const serial_config_t *config);

/**
 * @brief   Write data to serial port
 * @param   data    Data buffer to write
 * @param   len     Number of bytes to write
 * @retval  Number of bytes written, or negative on error
 * 
 * Writes data to the transmit buffer. If buffer is full,
 * blocks until space is available.
 */
int serial_write(const uint8_t *data, size_t len);

/**
 * @brief   Write a single byte to serial port
 * @param   byte    Byte to write
 * @retval  0 on success, negative on error
 */
int serial_putc(uint8_t byte);

/**
 * @brief   Write a null-terminated string to serial port
 * @param   str     String to write
 * @retval  Number of bytes written, or negative on error
 */
int serial_puts(const char *str);

/**
 * @brief   Read data from serial port
 * @param   data    Buffer to store received data
 * @param   len     Maximum number of bytes to read
 * @retval  Number of bytes read, or negative on error
 * 
 * Non-blocking read from receive buffer.
 * Returns 0 if no data available.
 */
int serial_read(uint8_t *data, size_t len);

/**
 * @brief   Read a single byte from serial port
 * @param   byte    Pointer to store received byte
 * @retval  0 on success, -1 if no data available
 */
int serial_getc(uint8_t *byte);

/**
 * @brief   Read a complete line from serial port
 * @param   line    Buffer to store the line (null-terminated)
 * @param   maxlen  Maximum line length (including null terminator)
 * @retval  Line length (excluding null), 0 if no complete line, negative on error
 * 
 * Returns a complete line (terminated by '\n' or '\r').
 * The line terminator is not included in the output.
 * Used for G-code line-by-line parsing.
 */
int serial_readline(char *line, size_t maxlen);

/**
 * @brief   Check if a complete line is available
 * @retval  1 if line available, 0 otherwise
 */
int serial_line_available(void);

/**
 * @brief   Get number of bytes available in receive buffer
 * @retval  Number of bytes available
 */
size_t serial_rx_available(void);

/**
 * @brief   Get free space in transmit buffer
 * @retval  Number of bytes free
 */
size_t serial_tx_free(void);

/**
 * @brief   Flush transmit buffer (wait for all data to be sent)
 */
void serial_flush(void);

/**
 * @brief   Clear receive buffer
 */
void serial_rx_clear(void);

/**
 * @brief   Enable serial receive interrupt
 */
void serial_rx_enable(void);

/**
 * @brief   Disable serial receive interrupt
 */
void serial_rx_disable(void);

/* ========== Debug Output ========== */

/**
 * @brief   Print formatted debug message
 * @param   fmt     Format string (printf-style)
 * @param   ...     Format arguments
 * @retval  Number of characters printed
 * 
 * Simple printf-like function for debug output.
 * Supports: %d, %u, %x, %s, %c
 */
int serial_printf(const char *fmt, ...);

/* ========== USART Pin Mapping ========== */

/*
 * STM32F407 USART pin mapping:
 * 
 * USART1:
 *   TX: PA9  (AF7) or PB6 (AF7)
 *   RX: PA10 (AF7) or PB7 (AF7)
 * 
 * USART2:
 *   TX: PA2 (AF7) or PD5 (AF7)
 *   RX: PA3 (AF7) or PD6 (AF7)
 * 
 * USART3:
 *   TX: PB10 (AF7) or PC10 (AF7) or PD8 (AF7)
 *   RX: PB11 (AF7) or PC11 (AF7) or PD9 (AF7)
 */

/* Default pin configuration (USART1) */
#define SERIAL_TX_GPIO          GPIO_PA9
#define SERIAL_RX_GPIO          GPIO_PA10
#define SERIAL_AF               7           /* AF7 for USART1-3 */

#endif /* STM32_SERIAL_H */
