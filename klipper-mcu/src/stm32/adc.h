/**
 * @file    adc.h
 * @brief   STM32F407 ADC driver interface
 * 
 * Provides ADC configuration and reading functions for STM32F407.
 * Used by heater.c for temperature reading from thermistors.
 * Follows Klipper coding style (C99, snake_case).
 */

#ifndef STM32_ADC_H
#define STM32_ADC_H

#include <stdint.h>

/* ========== ADC Definitions ========== */

/* ADC resolution */
#define ADC_MAX_VALUE           4095        /* 12-bit ADC */
#define ADC_RESOLUTION_BITS     12

/* Maximum number of ADC channels */
#define ADC_CHANNEL_MAX         16

/* ADC sample time options */
typedef enum {
    ADC_SAMPLETIME_3CYCLES      = 0,    /* 3 cycles */
    ADC_SAMPLETIME_15CYCLES     = 1,    /* 15 cycles */
    ADC_SAMPLETIME_28CYCLES     = 2,    /* 28 cycles */
    ADC_SAMPLETIME_56CYCLES     = 3,    /* 56 cycles */
    ADC_SAMPLETIME_84CYCLES     = 4,    /* 84 cycles */
    ADC_SAMPLETIME_112CYCLES    = 5,    /* 112 cycles */
    ADC_SAMPLETIME_144CYCLES    = 6,    /* 144 cycles */
    ADC_SAMPLETIME_480CYCLES    = 7     /* 480 cycles */
} adc_sampletime_t;

/* ADC channel configuration */
typedef struct {
    uint8_t channel;                    /* ADC channel number (0-15) */
    uint8_t gpio;                       /* GPIO pin for this channel */
    adc_sampletime_t sample_time;       /* Sample time */
} adc_channel_config_t;

/* ========== ADC Functions ========== */

/**
 * @brief   Initialize ADC subsystem
 * 
 * Enables ADC1 clock and configures for single conversion mode.
 * Must be called before using any other ADC functions.
 */
void adc_init(void);

/**
 * @brief   Configure an ADC channel
 * @param   gpio        GPIO pin to configure for ADC
 * @param   sample_time Sample time for this channel
 * @retval  0 on success, negative on error
 * 
 * Configures the GPIO pin for analog mode and sets up the ADC channel.
 * The channel number is determined from the GPIO pin.
 */
int adc_setup(uint8_t gpio, adc_sampletime_t sample_time);

/**
 * @brief   Read ADC value from a channel
 * @param   gpio    GPIO pin (must be previously configured with adc_setup)
 * @return  ADC value (0-4095) or negative on error
 * 
 * Performs a single conversion on the specified channel and returns
 * the result. This is a blocking call.
 */
int32_t adc_read(uint8_t gpio);

/**
 * @brief   Read ADC value by channel number
 * @param   channel ADC channel number (0-15)
 * @return  ADC value (0-4095) or negative on error
 * 
 * Performs a single conversion on the specified channel number.
 */
int32_t adc_read_channel(uint8_t channel);

/**
 * @brief   Get ADC channel number from GPIO pin
 * @param   gpio    GPIO pin
 * @return  ADC channel number (0-15) or negative if invalid
 */
int adc_get_channel(uint8_t gpio);

/**
 * @brief   Check if ADC is ready for conversion
 * @return  1 if ready, 0 if busy
 */
int adc_is_ready(void);

/* ========== ADC Channel Mapping ========== */

/*
 * STM32F407 ADC1 channel to GPIO mapping:
 * 
 * Channel 0  - PA0
 * Channel 1  - PA1
 * Channel 2  - PA2
 * Channel 3  - PA3
 * Channel 4  - PA4
 * Channel 5  - PA5
 * Channel 6  - PA6
 * Channel 7  - PA7
 * Channel 8  - PB0
 * Channel 9  - PB1
 * Channel 10 - PC0
 * Channel 11 - PC1
 * Channel 12 - PC2
 * Channel 13 - PC3
 * Channel 14 - PC4
 * Channel 15 - PC5
 * Channel 16 - Internal temperature sensor
 * Channel 17 - Internal VREFINT
 * Channel 18 - VBAT/4
 */

/* ADC channel definitions for common pins */
#define ADC_CHANNEL_PA0         0
#define ADC_CHANNEL_PA1         1
#define ADC_CHANNEL_PA2         2
#define ADC_CHANNEL_PA3         3
#define ADC_CHANNEL_PA4         4
#define ADC_CHANNEL_PA5         5
#define ADC_CHANNEL_PA6         6
#define ADC_CHANNEL_PA7         7
#define ADC_CHANNEL_PB0         8
#define ADC_CHANNEL_PB1         9
#define ADC_CHANNEL_PC0         10
#define ADC_CHANNEL_PC1         11
#define ADC_CHANNEL_PC2         12
#define ADC_CHANNEL_PC3         13
#define ADC_CHANNEL_PC4         14
#define ADC_CHANNEL_PC5         15

/* Internal channels */
#define ADC_CHANNEL_TEMP        16          /* Internal temperature sensor */
#define ADC_CHANNEL_VREFINT     17          /* Internal voltage reference */
#define ADC_CHANNEL_VBAT        18          /* VBAT/4 */

#endif /* STM32_ADC_H */
