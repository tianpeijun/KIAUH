/**
 * @file    serial.c
 * @brief   STM32F407 USART serial driver implementation
 * 
 * USART configuration and communication for STM32F407.
 * Supports interrupt-driven receive with ring buffer.
 * Provides line-buffered input for G-code parsing.
 * Follows Klipper coding style (C99, snake_case).
 */

#include "serial.h"
#include "gpio.h"
#include "internal.h"
#include "board/irq.h"
#include <stdarg.h>

/* ========== USART Register Definitions ========== */

/* USART register structure */
typedef struct {
    volatile uint32_t SR;       /* Status register */
    volatile uint32_t DR;       /* Data register */
    volatile uint32_t BRR;      /* Baud rate register */
    volatile uint32_t CR1;      /* Control register 1 */
    volatile uint32_t CR2;      /* Control register 2 */
    volatile uint32_t CR3;      /* Control register 3 */
    volatile uint32_t GTPR;     /* Guard time and prescaler register */
} usart_regs_t;

/* USART base addresses */
#define USART1                  ((usart_regs_t *)USART1_BASE)
#define USART2                  ((usart_regs_t *)USART2_BASE)
#define USART3                  ((usart_regs_t *)USART3_BASE)

/* RCC registers for clock enable */
#define RCC_BASE                0x40023800
#define RCC_APB1ENR             (*(volatile uint32_t *)(RCC_BASE + 0x40))
#define RCC_APB2ENR             (*(volatile uint32_t *)(RCC_BASE + 0x44))

/* ========== USART Status Register (SR) Bits ========== */

#define USART_SR_PE             (1 << 0)    /* Parity error */
#define USART_SR_FE             (1 << 1)    /* Framing error */
#define USART_SR_NF             (1 << 2)    /* Noise detected flag */
#define USART_SR_ORE            (1 << 3)    /* Overrun error */
#define USART_SR_IDLE           (1 << 4)    /* IDLE line detected */
#define USART_SR_RXNE           (1 << 5)    /* Read data register not empty */
#define USART_SR_TC             (1 << 6)    /* Transmission complete */
#define USART_SR_TXE            (1 << 7)    /* Transmit data register empty */
#define USART_SR_LBD            (1 << 8)    /* LIN break detection flag */
#define USART_SR_CTS            (1 << 9)    /* CTS flag */

/* ========== USART Control Register 1 (CR1) Bits ========== */

#define USART_CR1_SBK           (1 << 0)    /* Send break */
#define USART_CR1_RWU           (1 << 1)    /* Receiver wakeup */
#define USART_CR1_RE            (1 << 2)    /* Receiver enable */
#define USART_CR1_TE            (1 << 3)    /* Transmitter enable */
#define USART_CR1_IDLEIE        (1 << 4)    /* IDLE interrupt enable */
#define USART_CR1_RXNEIE        (1 << 5)    /* RXNE interrupt enable */
#define USART_CR1_TCIE          (1 << 6)    /* Transmission complete interrupt enable */
#define USART_CR1_TXEIE         (1 << 7)    /* TXE interrupt enable */
#define USART_CR1_PEIE          (1 << 8)    /* PE interrupt enable */
#define USART_CR1_PS            (1 << 9)    /* Parity selection */
#define USART_CR1_PCE           (1 << 10)   /* Parity control enable */
#define USART_CR1_WAKE          (1 << 11)   /* Wakeup method */
#define USART_CR1_M             (1 << 12)   /* Word length */
#define USART_CR1_UE            (1 << 13)   /* USART enable */
#define USART_CR1_OVER8         (1 << 15)   /* Oversampling mode */

/* ========== USART Control Register 2 (CR2) Bits ========== */

#define USART_CR2_STOP_1        (0 << 12)   /* 1 stop bit */
#define USART_CR2_STOP_0_5      (1 << 12)   /* 0.5 stop bits */
#define USART_CR2_STOP_2        (2 << 12)   /* 2 stop bits */
#define USART_CR2_STOP_1_5      (3 << 12)   /* 1.5 stop bits */
#define USART_CR2_STOP_MASK     (3 << 12)

/* ========== USART Control Register 3 (CR3) Bits ========== */

#define USART_CR3_EIE           (1 << 0)    /* Error interrupt enable */
#define USART_CR3_IREN          (1 << 1)    /* IrDA mode enable */
#define USART_CR3_IRLP          (1 << 2)    /* IrDA low-power */
#define USART_CR3_HDSEL         (1 << 3)    /* Half-duplex selection */
#define USART_CR3_NACK          (1 << 4)    /* Smartcard NACK enable */
#define USART_CR3_SCEN          (1 << 5)    /* Smartcard mode enable */
#define USART_CR3_DMAR          (1 << 6)    /* DMA enable receiver */
#define USART_CR3_DMAT          (1 << 7)    /* DMA enable transmitter */
#define USART_CR3_RTSE          (1 << 8)    /* RTS enable */
#define USART_CR3_CTSE          (1 << 9)    /* CTS enable */
#define USART_CR3_CTSIE         (1 << 10)   /* CTS interrupt enable */
#define USART_CR3_ONEBIT        (1 << 11)   /* One sample bit method enable */

/* ========== Ring Buffer Structure ========== */

typedef struct {
    volatile uint8_t buffer[SERIAL_RX_BUFFER_SIZE];
    volatile size_t head;       /* Write index */
    volatile size_t tail;       /* Read index */
    volatile size_t count;      /* Number of bytes in buffer */
} ring_buffer_t;

/* ========== Private Variables ========== */

/* Active USART port */
static usart_regs_t *s_usart = USART1;
static uint8_t s_usart_irq = IRQ_USART1;

/* Receive ring buffer */
static ring_buffer_t s_rx_buffer;

/* Transmit ring buffer */
static ring_buffer_t s_tx_buffer;

/* Line buffer for G-code parsing */
static char s_line_buffer[SERIAL_LINE_BUFFER_SIZE];
static volatile size_t s_line_len = 0;
static volatile uint8_t s_line_ready = 0;

/* Initialization flag */
static uint8_t s_initialized = 0;

/* ========== Private Functions ========== */

/**
 * @brief   Initialize ring buffer
 */
static void
ring_buffer_init(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

/**
 * @brief   Put byte into ring buffer
 * @retval  0 on success, -1 if buffer full
 */
static int
ring_buffer_put(ring_buffer_t *rb, uint8_t byte)
{
    if (rb->count >= SERIAL_RX_BUFFER_SIZE) {
        return -1;  /* Buffer full */
    }
    
    rb->buffer[rb->head] = byte;
    rb->head = (rb->head + 1) % SERIAL_RX_BUFFER_SIZE;
    rb->count++;
    
    return 0;
}

/**
 * @brief   Get byte from ring buffer
 * @retval  0 on success, -1 if buffer empty
 */
static int
ring_buffer_get(ring_buffer_t *rb, uint8_t *byte)
{
    if (rb->count == 0) {
        return -1;  /* Buffer empty */
    }
    
    *byte = rb->buffer[rb->tail];
    rb->tail = (rb->tail + 1) % SERIAL_RX_BUFFER_SIZE;
    rb->count--;
    
    return 0;
}

/**
 * @brief   Get USART peripheral clock frequency
 */
static uint32_t
get_usart_clock(usart_regs_t *usart)
{
    /* USART1 and USART6 are on APB2 (84 MHz) */
    /* USART2, USART3, UART4, UART5 are on APB1 (42 MHz) */
    if (usart == USART1) {
        return APB2_FREQ;
    } else {
        return APB1_FREQ;
    }
}

/**
 * @brief   Calculate BRR value for baud rate
 */
static uint32_t
calculate_brr(usart_regs_t *usart, uint32_t baud)
{
    uint32_t pclk = get_usart_clock(usart);
    
    /* BRR = fck / (16 * baud) for oversampling by 16 */
    /* Using fixed-point: BRR = (fck * 100) / (16 * baud) / 100 */
    /* Simplified: BRR = fck / baud, then divide mantissa and fraction */
    
    uint32_t div = (pclk + (baud / 2)) / baud;
    
    return div;
}

/**
 * @brief   Configure GPIO pins for USART
 */
static void
configure_gpio_pins(serial_port_t port)
{
    uint8_t tx_gpio, rx_gpio;
    
    switch (port) {
        case SERIAL_USART1:
            tx_gpio = GPIO_PA9;
            rx_gpio = GPIO_PA10;
            break;
        case SERIAL_USART2:
            tx_gpio = GPIO_PA2;
            rx_gpio = GPIO_PA3;
            break;
        case SERIAL_USART3:
            tx_gpio = GPIO_PB10;
            rx_gpio = GPIO_PB11;
            break;
        default:
            return;
    }
    
    /* Configure TX pin as alternate function push-pull */
    gpio_config_t tx_config = {
        .mode = GPIO_MODE_AF,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_HIGH,
        .pupd = GPIO_PUPD_UP,
        .af = GPIO_AF_USART1     /* AF7 for USART1-3 */
    };
    gpio_configure(tx_gpio, &tx_config);
    
    /* Configure RX pin as alternate function with pull-up */
    gpio_config_t rx_config = {
        .mode = GPIO_MODE_AF,
        .otype = GPIO_OTYPE_PP,
        .speed = GPIO_SPEED_HIGH,
        .pupd = GPIO_PUPD_UP,
        .af = GPIO_AF_USART1     /* AF7 for USART1-3 */
    };
    gpio_configure(rx_gpio, &rx_config);
}

/**
 * @brief   Enable USART clock
 */
static void
enable_usart_clock(serial_port_t port)
{
    switch (port) {
        case SERIAL_USART1:
            RCC_APB2ENR |= (1 << 4);    /* USART1EN */
            break;
        case SERIAL_USART2:
            RCC_APB1ENR |= (1 << 17);   /* USART2EN */
            break;
        case SERIAL_USART3:
            RCC_APB1ENR |= (1 << 18);   /* USART3EN */
            break;
        default:
            break;
    }
    
    /* Small delay for clock to stabilize */
    for (volatile int i = 0; i < 100; i++) {
        __asm__ __volatile__("nop");
    }
}

/**
 * @brief   Get USART registers and IRQ number
 */
static void
get_usart_info(serial_port_t port, usart_regs_t **usart, uint8_t *irq)
{
    switch (port) {
        case SERIAL_USART1:
            *usart = USART1;
            *irq = IRQ_USART1;
            break;
        case SERIAL_USART2:
            *usart = USART2;
            *irq = IRQ_USART2;
            break;
        case SERIAL_USART3:
            *usart = USART3;
            *irq = IRQ_USART3;
            break;
        default:
            *usart = USART1;
            *irq = IRQ_USART1;
            break;
    }
}

/**
 * @brief   Process received byte (called from ISR)
 */
static void
process_rx_byte(uint8_t byte)
{
    /* Add to ring buffer */
    ring_buffer_put(&s_rx_buffer, byte);
    
    /* Line buffer processing for G-code */
    if (!s_line_ready) {
        if (byte == '\n' || byte == '\r') {
            /* Line complete */
            if (s_line_len > 0) {
                s_line_buffer[s_line_len] = '\0';
                s_line_ready = 1;
            }
        } else if (byte == '\b' || byte == 0x7F) {
            /* Backspace - remove last character */
            if (s_line_len > 0) {
                s_line_len--;
            }
        } else if (s_line_len < SERIAL_LINE_BUFFER_SIZE - 1) {
            /* Add character to line buffer */
            s_line_buffer[s_line_len++] = (char)byte;
        }
    }
}

/* ========== Interrupt Handler ========== */

/**
 * @brief   USART interrupt handler
 * 
 * Handles receive and transmit interrupts.
 * This function should be called from the USART ISR vector.
 */
void
serial_irq_handler(void)
{
    uint32_t sr = s_usart->SR;
    
    /* Receive data available */
    if (sr & USART_SR_RXNE) {
        uint8_t byte = (uint8_t)(s_usart->DR & 0xFF);
        process_rx_byte(byte);
    }
    
    /* Transmit data register empty */
    if ((sr & USART_SR_TXE) && (s_usart->CR1 & USART_CR1_TXEIE)) {
        uint8_t byte;
        if (ring_buffer_get(&s_tx_buffer, &byte) == 0) {
            s_usart->DR = byte;
        } else {
            /* No more data to send, disable TXE interrupt */
            s_usart->CR1 &= ~USART_CR1_TXEIE;
        }
    }
    
    /* Handle errors by reading SR and DR to clear flags */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NF | USART_SR_PE)) {
        (void)s_usart->DR;  /* Clear error flags by reading DR */
    }
}

/* USART1 ISR - weak symbol, can be overridden */
void __attribute__((weak))
USART1_IRQHandler(void)
{
    if (s_usart == USART1) {
        serial_irq_handler();
    }
}

/* USART2 ISR - weak symbol, can be overridden */
void __attribute__((weak))
USART2_IRQHandler(void)
{
    if (s_usart == USART2) {
        serial_irq_handler();
    }
}

/* USART3 ISR - weak symbol, can be overridden */
void __attribute__((weak))
USART3_IRQHandler(void)
{
    if (s_usart == USART3) {
        serial_irq_handler();
    }
}

/* ========== Public Functions ========== */

/**
 * @brief   Initialize serial subsystem with default settings
 */
void
serial_init(void)
{
    serial_config_t config = {
        .port = SERIAL_USART1,
        .baud = SERIAL_BAUD_DEFAULT
    };
    serial_init_config(&config);
}

/**
 * @brief   Initialize serial with configuration
 */
int
serial_init_config(const serial_config_t *config)
{
    if (config == NULL) {
        return -1;
    }
    
    if (config->port >= SERIAL_COUNT) {
        return -2;
    }
    
    /* Initialize GPIO subsystem if not already done */
    gpio_init();
    
    /* Initialize buffers */
    ring_buffer_init(&s_rx_buffer);
    ring_buffer_init(&s_tx_buffer);
    s_line_len = 0;
    s_line_ready = 0;
    
    /* Get USART info */
    get_usart_info(config->port, &s_usart, &s_usart_irq);
    
    /* Enable USART clock */
    enable_usart_clock(config->port);
    
    /* Configure GPIO pins */
    configure_gpio_pins(config->port);
    
    /* Disable USART during configuration */
    s_usart->CR1 = 0;
    s_usart->CR2 = 0;
    s_usart->CR3 = 0;
    
    /* Configure baud rate */
    s_usart->BRR = calculate_brr(s_usart, config->baud);
    
    /* Configure: 8 data bits, no parity, 1 stop bit */
    s_usart->CR2 = USART_CR2_STOP_1;
    
    /* Enable USART, TX, RX, and RXNE interrupt */
    s_usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    
    /* Enable NVIC interrupt */
    nvic_set_priority(s_usart_irq, 64);  /* Medium priority */
    nvic_enable_irq(s_usart_irq);
    
    s_initialized = 1;
    
    return 0;
}

/**
 * @brief   Write data to serial port
 */
int
serial_write(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    
    if (!s_initialized) {
        return -2;
    }
    
    size_t written = 0;
    
    for (size_t i = 0; i < len; i++) {
        /* Wait for TXE (transmit data register empty) */
        uint32_t timeout = 100000;
        while (!(s_usart->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return (int)written;  /* Timeout */
            }
        }
        
        /* Write byte directly to data register */
        s_usart->DR = data[i];
        written++;
    }
    
    return (int)written;
}

/**
 * @brief   Write a single byte to serial port
 */
int
serial_putc(uint8_t byte)
{
    return serial_write(&byte, 1) == 1 ? 0 : -1;
}

/**
 * @brief   Write a null-terminated string to serial port
 */
int
serial_puts(const char *str)
{
    if (str == NULL) {
        return -1;
    }
    
    size_t len = 0;
    while (str[len] != '\0') {
        len++;
    }
    
    return serial_write((const uint8_t *)str, len);
}

/**
 * @brief   Read data from serial port
 */
int
serial_read(uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return -1;
    }
    
    if (!s_initialized) {
        return -2;
    }
    
    size_t read_count = 0;
    uint32_t irqflag = irq_disable();
    
    while (read_count < len) {
        uint8_t byte;
        if (ring_buffer_get(&s_rx_buffer, &byte) == 0) {
            data[read_count++] = byte;
        } else {
            break;  /* No more data */
        }
    }
    
    irq_restore(irqflag);
    
    return (int)read_count;
}

/**
 * @brief   Read a single byte from serial port
 */
int
serial_getc(uint8_t *byte)
{
    if (byte == NULL) {
        return -1;
    }
    
    return serial_read(byte, 1) == 1 ? 0 : -1;
}

/**
 * @brief   Read a complete line from serial port
 */
int
serial_readline(char *line, size_t maxlen)
{
    if (line == NULL || maxlen == 0) {
        return -1;
    }
    
    if (!s_initialized) {
        return -2;
    }
    
    if (!s_line_ready) {
        return 0;  /* No complete line available */
    }
    
    uint32_t irqflag = irq_disable();
    
    /* Copy line to output buffer */
    size_t copy_len = s_line_len;
    if (copy_len >= maxlen) {
        copy_len = maxlen - 1;
    }
    
    for (size_t i = 0; i < copy_len; i++) {
        line[i] = s_line_buffer[i];
    }
    line[copy_len] = '\0';
    
    /* Reset line buffer */
    s_line_len = 0;
    s_line_ready = 0;
    
    irq_restore(irqflag);
    
    return (int)copy_len;
}

/**
 * @brief   Check if a complete line is available
 */
int
serial_line_available(void)
{
    return s_line_ready ? 1 : 0;
}

/**
 * @brief   Get number of bytes available in receive buffer
 */
size_t
serial_rx_available(void)
{
    return s_rx_buffer.count;
}

/**
 * @brief   Get free space in transmit buffer
 */
size_t
serial_tx_free(void)
{
    return SERIAL_TX_BUFFER_SIZE - s_tx_buffer.count;
}

/**
 * @brief   Flush transmit buffer
 */
void
serial_flush(void)
{
    if (!s_initialized) {
        return;
    }
    
    /* Wait for transmission complete */
    uint32_t timeout = 1000000;
    while (!(s_usart->SR & USART_SR_TC)) {
        if (--timeout == 0) {
            break;
        }
    }
}

/**
 * @brief   Clear receive buffer
 */
void
serial_rx_clear(void)
{
    uint32_t irqflag = irq_disable();
    ring_buffer_init(&s_rx_buffer);
    s_line_len = 0;
    s_line_ready = 0;
    irq_restore(irqflag);
}

/**
 * @brief   Enable serial receive interrupt
 */
void
serial_rx_enable(void)
{
    if (s_initialized) {
        s_usart->CR1 |= USART_CR1_RXNEIE;
    }
}

/**
 * @brief   Disable serial receive interrupt
 */
void
serial_rx_disable(void)
{
    if (s_initialized) {
        s_usart->CR1 &= ~USART_CR1_RXNEIE;
    }
}

/* ========== Debug Output Functions ========== */

/**
 * @brief   Print unsigned integer
 */
static int
print_uint(uint32_t val, int base, int width, char pad)
{
    char buf[12];
    int len = 0;
    int count = 0;
    
    if (val == 0) {
        buf[len++] = '0';
    } else {
        while (val > 0) {
            int digit = val % base;
            if (digit < 10) {
                buf[len++] = '0' + digit;
            } else {
                buf[len++] = 'a' + digit - 10;
            }
            val /= base;
        }
    }
    
    /* Padding */
    while (len < width) {
        serial_putc(pad);
        count++;
        width--;
    }
    
    /* Print in reverse order */
    while (len > 0) {
        serial_putc(buf[--len]);
        count++;
    }
    
    return count;
}

/**
 * @brief   Print signed integer
 */
static int
print_int(int32_t val, int width, char pad)
{
    int count = 0;
    
    if (val < 0) {
        serial_putc('-');
        count++;
        val = -val;
        if (width > 0) {
            width--;
        }
    }
    
    count += print_uint((uint32_t)val, 10, width, pad);
    
    return count;
}

/**
 * @brief   Print formatted debug message
 */
int
serial_printf(const char *fmt, ...)
{
    if (fmt == NULL) {
        return -1;
    }
    
    if (!s_initialized) {
        return -2;
    }
    
    va_list args;
    va_start(args, fmt);
    
    int count = 0;
    
    while (*fmt != '\0') {
        if (*fmt == '%') {
            fmt++;
            
            /* Parse width */
            int width = 0;
            char pad = ' ';
            
            if (*fmt == '0') {
                pad = '0';
                fmt++;
            }
            
            while (*fmt >= '0' && *fmt <= '9') {
                width = width * 10 + (*fmt - '0');
                fmt++;
            }
            
            /* Parse format specifier */
            switch (*fmt) {
                case 'd':
                case 'i':
                    count += print_int(va_arg(args, int32_t), width, pad);
                    break;
                    
                case 'u':
                    count += print_uint(va_arg(args, uint32_t), 10, width, pad);
                    break;
                    
                case 'x':
                case 'X':
                    count += print_uint(va_arg(args, uint32_t), 16, width, pad);
                    break;
                    
                case 'c':
                    serial_putc((uint8_t)va_arg(args, int));
                    count++;
                    break;
                    
                case 's': {
                    const char *str = va_arg(args, const char *);
                    if (str != NULL) {
                        while (*str != '\0') {
                            serial_putc(*str++);
                            count++;
                        }
                    }
                    break;
                }
                    
                case '%':
                    serial_putc('%');
                    count++;
                    break;
                    
                default:
                    /* Unknown format, print as-is */
                    serial_putc('%');
                    serial_putc(*fmt);
                    count += 2;
                    break;
            }
            fmt++;
        } else {
            serial_putc(*fmt++);
            count++;
        }
    }
    
    va_end(args);
    
    return count;
}
