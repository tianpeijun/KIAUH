/**
 * @file    adc.c
 * @brief   STM32F407 ADC driver implementation
 * 
 * ADC configuration and reading for STM32F407.
 * Supports ADC1 with single conversion mode for temperature reading.
 * Follows Klipper coding style (C99, snake_case).
 */

#include "adc.h"
#include "gpio.h"
#include "internal.h"
#include "board/irq.h"

/* ========== ADC Register Definitions ========== */

#define ADC1_BASE               0x40012000
#define ADC_COMMON_BASE         0x40012300

/* ADC1 registers */
#define ADC1_SR                 (*(volatile uint32_t *)(ADC1_BASE + 0x00))
#define ADC1_CR1                (*(volatile uint32_t *)(ADC1_BASE + 0x04))
#define ADC1_CR2                (*(volatile uint32_t *)(ADC1_BASE + 0x08))
#define ADC1_SMPR1              (*(volatile uint32_t *)(ADC1_BASE + 0x0C))
#define ADC1_SMPR2              (*(volatile uint32_t *)(ADC1_BASE + 0x10))
#define ADC1_JOFR1              (*(volatile uint32_t *)(ADC1_BASE + 0x14))
#define ADC1_JOFR2              (*(volatile uint32_t *)(ADC1_BASE + 0x18))
#define ADC1_JOFR3              (*(volatile uint32_t *)(ADC1_BASE + 0x1C))
#define ADC1_JOFR4              (*(volatile uint32_t *)(ADC1_BASE + 0x20))
#define ADC1_HTR                (*(volatile uint32_t *)(ADC1_BASE + 0x24))
#define ADC1_LTR                (*(volatile uint32_t *)(ADC1_BASE + 0x28))
#define ADC1_SQR1               (*(volatile uint32_t *)(ADC1_BASE + 0x2C))
#define ADC1_SQR2               (*(volatile uint32_t *)(ADC1_BASE + 0x30))
#define ADC1_SQR3               (*(volatile uint32_t *)(ADC1_BASE + 0x34))
#define ADC1_JSQR               (*(volatile uint32_t *)(ADC1_BASE + 0x38))
#define ADC1_JDR1               (*(volatile uint32_t *)(ADC1_BASE + 0x3C))
#define ADC1_JDR2               (*(volatile uint32_t *)(ADC1_BASE + 0x40))
#define ADC1_JDR3               (*(volatile uint32_t *)(ADC1_BASE + 0x44))
#define ADC1_JDR4               (*(volatile uint32_t *)(ADC1_BASE + 0x48))
#define ADC1_DR                 (*(volatile uint32_t *)(ADC1_BASE + 0x4C))

/* ADC common registers */
#define ADC_CCR                 (*(volatile uint32_t *)(ADC_COMMON_BASE + 0x04))

/* RCC register for ADC clock enable */
#define RCC_BASE                0x40023800
#define RCC_APB2ENR             (*(volatile uint32_t *)(RCC_BASE + 0x44))

/* ========== ADC Status Register (SR) Bits ========== */

#define ADC_SR_AWD              (1 << 0)    /* Analog watchdog flag */
#define ADC_SR_EOC              (1 << 1)    /* End of conversion */
#define ADC_SR_JEOC             (1 << 2)    /* Injected channel end of conversion */
#define ADC_SR_JSTRT            (1 << 3)    /* Injected channel start flag */
#define ADC_SR_STRT             (1 << 4)    /* Regular channel start flag */
#define ADC_SR_OVR              (1 << 5)    /* Overrun */

/* ========== ADC Control Register 1 (CR1) Bits ========== */

#define ADC_CR1_AWDCH_MASK      (0x1F << 0) /* Analog watchdog channel select */
#define ADC_CR1_EOCIE           (1 << 5)    /* EOC interrupt enable */
#define ADC_CR1_AWDIE           (1 << 6)    /* Analog watchdog interrupt enable */
#define ADC_CR1_JEOCIE          (1 << 7)    /* Injected EOC interrupt enable */
#define ADC_CR1_SCAN            (1 << 8)    /* Scan mode */
#define ADC_CR1_AWDSGL          (1 << 9)    /* Watchdog on single channel */
#define ADC_CR1_JAUTO           (1 << 10)   /* Automatic injected conversion */
#define ADC_CR1_DISCEN          (1 << 11)   /* Discontinuous mode on regular */
#define ADC_CR1_JDISCEN         (1 << 12)   /* Discontinuous mode on injected */
#define ADC_CR1_DISCNUM_MASK    (0x07 << 13) /* Discontinuous mode channel count */
#define ADC_CR1_JAWDEN          (1 << 22)   /* Analog watchdog on injected */
#define ADC_CR1_AWDEN           (1 << 23)   /* Analog watchdog on regular */
#define ADC_CR1_RES_12BIT       (0 << 24)   /* 12-bit resolution */
#define ADC_CR1_RES_10BIT       (1 << 24)   /* 10-bit resolution */
#define ADC_CR1_RES_8BIT        (2 << 24)   /* 8-bit resolution */
#define ADC_CR1_RES_6BIT        (3 << 24)   /* 6-bit resolution */
#define ADC_CR1_OVRIE           (1 << 26)   /* Overrun interrupt enable */

/* ========== ADC Control Register 2 (CR2) Bits ========== */

#define ADC_CR2_ADON            (1 << 0)    /* ADC on/off */
#define ADC_CR2_CONT            (1 << 1)    /* Continuous conversion */
#define ADC_CR2_DMA             (1 << 8)    /* DMA mode */
#define ADC_CR2_DDS             (1 << 9)    /* DMA disable selection */
#define ADC_CR2_EOCS            (1 << 10)   /* End of conversion selection */
#define ADC_CR2_ALIGN           (1 << 11)   /* Data alignment (0=right, 1=left) */
#define ADC_CR2_JEXTSEL_MASK    (0x0F << 16) /* External event for injected */
#define ADC_CR2_JEXTEN_MASK     (0x03 << 20) /* Injected external trigger enable */
#define ADC_CR2_JSWSTART        (1 << 22)   /* Start injected conversion */
#define ADC_CR2_EXTSEL_MASK     (0x0F << 24) /* External event for regular */
#define ADC_CR2_EXTEN_MASK      (0x03 << 28) /* Regular external trigger enable */
#define ADC_CR2_SWSTART         (1 << 30)   /* Start regular conversion */

/* ========== ADC Common Control Register (CCR) Bits ========== */

#define ADC_CCR_ADCPRE_DIV2     (0 << 16)   /* PCLK2 / 2 */
#define ADC_CCR_ADCPRE_DIV4     (1 << 16)   /* PCLK2 / 4 */
#define ADC_CCR_ADCPRE_DIV6     (2 << 16)   /* PCLK2 / 6 */
#define ADC_CCR_ADCPRE_DIV8     (3 << 16)   /* PCLK2 / 8 */
#define ADC_CCR_VBATE           (1 << 22)   /* VBAT enable */
#define ADC_CCR_TSVREFE         (1 << 23)   /* Temperature sensor and VREFINT enable */

/* ========== Private Variables ========== */

/* ADC initialization flag */
static uint8_t s_adc_initialized = 0;

/* Channel configuration tracking */
static uint8_t s_channel_configured[ADC_CHANNEL_MAX] = {0};

/* ========== Private Functions ========== */

/**
 * @brief   Get ADC channel number from GPIO pin
 * @param   gpio    GPIO pin
 * @return  ADC channel number or -1 if invalid
 */
static int
get_adc_channel_from_gpio(uint8_t gpio)
{
    uint8_t port = GPIO_PORT(gpio);
    uint8_t pin = GPIO_PIN(gpio);
    
    /* Port A: PA0-PA7 -> Channel 0-7 */
    if (port == GPIO_PORT_A && pin <= 7) {
        return pin;
    }
    
    /* Port B: PB0-PB1 -> Channel 8-9 */
    if (port == GPIO_PORT_B && pin <= 1) {
        return 8 + pin;
    }
    
    /* Port C: PC0-PC5 -> Channel 10-15 */
    if (port == GPIO_PORT_C && pin <= 5) {
        return 10 + pin;
    }
    
    /* Invalid GPIO for ADC */
    return -1;
}

/**
 * @brief   Set sample time for a channel
 * @param   channel     ADC channel (0-17)
 * @param   sample_time Sample time setting
 */
static void
set_channel_sample_time(uint8_t channel, adc_sampletime_t sample_time)
{
    if (channel <= 9) {
        /* Channels 0-9 in SMPR2 */
        uint32_t shift = channel * 3;
        uint32_t mask = 0x07 << shift;
        ADC1_SMPR2 = (ADC1_SMPR2 & ~mask) | ((uint32_t)sample_time << shift);
    } else {
        /* Channels 10-17 in SMPR1 */
        uint32_t shift = (channel - 10) * 3;
        uint32_t mask = 0x07 << shift;
        ADC1_SMPR1 = (ADC1_SMPR1 & ~mask) | ((uint32_t)sample_time << shift);
    }
}

/**
 * @brief   Wait for ADC conversion to complete
 * @param   timeout_us  Timeout in microseconds
 * @return  0 on success, -1 on timeout
 */
static int
wait_for_conversion(uint32_t timeout_us)
{
    uint32_t timeout = timeout_us;
    
    while (!(ADC1_SR & ADC_SR_EOC)) {
        if (timeout == 0) {
            return -1;
        }
        timeout--;
        /* Small delay - approximately 1us at 168MHz */
        for (volatile int i = 0; i < 42; i++) {
            __asm__ __volatile__("nop");
        }
    }
    
    return 0;
}

/* ========== Public Functions ========== */

/**
 * @brief   Initialize ADC subsystem
 */
void
adc_init(void)
{
    if (s_adc_initialized) {
        return;
    }
    
    /* Enable ADC1 clock */
    RCC_APB2ENR |= (1 << 8);    /* ADC1EN */
    
    /* Small delay for clock to stabilize */
    for (volatile int i = 0; i < 100; i++) {
        __asm__ __volatile__("nop");
    }
    
    /* Configure ADC prescaler (PCLK2 / 4 = 84MHz / 4 = 21MHz) */
    /* ADC clock must be <= 36MHz for STM32F407 */
    ADC_CCR = ADC_CCR_ADCPRE_DIV4;
    
    /* Configure ADC1 */
    /* 12-bit resolution, right-aligned data, single conversion mode */
    ADC1_CR1 = ADC_CR1_RES_12BIT;
    
    /* Single conversion mode, software trigger */
    ADC1_CR2 = ADC_CR2_EOCS;    /* EOC flag at end of each conversion */
    
    /* Set default sample time for all channels (112 cycles for stability) */
    ADC1_SMPR1 = 0x00FFFFFF;    /* All channels 10-17: 480 cycles */
    ADC1_SMPR2 = 0x3FFFFFFF;    /* All channels 0-9: 480 cycles */
    
    /* Configure sequence: 1 conversion */
    ADC1_SQR1 = 0;              /* L[3:0] = 0 -> 1 conversion */
    ADC1_SQR2 = 0;
    ADC1_SQR3 = 0;              /* First conversion channel (will be set per read) */
    
    /* Enable ADC */
    ADC1_CR2 |= ADC_CR2_ADON;
    
    /* Wait for ADC to stabilize (tSTAB = 3us max) */
    for (volatile int i = 0; i < 1000; i++) {
        __asm__ __volatile__("nop");
    }
    
    s_adc_initialized = 1;
}

/**
 * @brief   Configure an ADC channel
 */
int
adc_setup(uint8_t gpio, adc_sampletime_t sample_time)
{
    /* Initialize ADC if not already done */
    if (!s_adc_initialized) {
        adc_init();
    }
    
    /* Get channel number from GPIO */
    int channel = get_adc_channel_from_gpio(gpio);
    if (channel < 0) {
        return -1;  /* Invalid GPIO for ADC */
    }
    
    /* Configure GPIO for analog mode */
    gpio_analog_setup(gpio);
    
    /* Set sample time for this channel */
    set_channel_sample_time((uint8_t)channel, sample_time);
    
    /* Mark channel as configured */
    if (channel < ADC_CHANNEL_MAX) {
        s_channel_configured[channel] = 1;
    }
    
    return 0;
}

/**
 * @brief   Read ADC value from a GPIO pin
 */
int32_t
adc_read(uint8_t gpio)
{
    /* Get channel number from GPIO */
    int channel = get_adc_channel_from_gpio(gpio);
    if (channel < 0) {
        return -1;  /* Invalid GPIO for ADC */
    }
    
    return adc_read_channel((uint8_t)channel);
}

/**
 * @brief   Read ADC value by channel number
 */
int32_t
adc_read_channel(uint8_t channel)
{
    if (channel > 18) {
        return -1;  /* Invalid channel */
    }
    
    /* Initialize ADC if not already done */
    if (!s_adc_initialized) {
        adc_init();
    }
    
    uint32_t irqflag = irq_disable();
    
    /* Clear status flags */
    ADC1_SR = 0;
    
    /* Set channel in sequence register */
    ADC1_SQR3 = channel;
    
    /* Start conversion */
    ADC1_CR2 |= ADC_CR2_SWSTART;
    
    /* Wait for conversion to complete (timeout ~100us) */
    int result = wait_for_conversion(100);
    
    if (result < 0) {
        irq_restore(irqflag);
        return -2;  /* Timeout */
    }
    
    /* Read result */
    uint32_t value = ADC1_DR & 0x0FFF;
    
    irq_restore(irqflag);
    
    return (int32_t)value;
}

/**
 * @brief   Get ADC channel number from GPIO pin
 */
int
adc_get_channel(uint8_t gpio)
{
    return get_adc_channel_from_gpio(gpio);
}

/**
 * @brief   Check if ADC is ready for conversion
 */
int
adc_is_ready(void)
{
    if (!s_adc_initialized) {
        return 0;
    }
    
    /* Check if ADC is enabled and not currently converting */
    if ((ADC1_CR2 & ADC_CR2_ADON) && !(ADC1_SR & ADC_SR_STRT)) {
        return 1;
    }
    
    return 0;
}
