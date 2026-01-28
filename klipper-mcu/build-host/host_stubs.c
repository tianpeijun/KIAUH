/* 自动生成的主机桩文件 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* ========== ADC 桩 ========== */
static int32_t s_mock_adc[16] = {2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048, 2048};

void adc_init(void) { }
int adc_setup(uint8_t gpio, int sample_time) { (void)gpio; (void)sample_time; return 0; }
int32_t adc_read(uint8_t gpio) {
    uint8_t port = (gpio >> 4) & 0x0F;
    uint8_t pin = gpio & 0x0F;
    int ch = -1;
    if (port == 0 && pin <= 7) ch = pin;
    else if (port == 1 && pin <= 1) ch = 8 + pin;
    else if (port == 2 && pin <= 5) ch = 10 + pin;
    return (ch >= 0 && ch < 16) ? s_mock_adc[ch] : -1;
}

/* ========== PWM 桩 ========== */
typedef struct { uint8_t pin; uint32_t cycle_time; uint16_t max_value; uint8_t invert; uint8_t use_hardware; } pwm_config_t;
static float s_pwm_duty[4] = {0};
static uint8_t s_pwm_enabled[4] = {0};

int pwm_init(void) { return 0; }
int pwm_config(int id, const pwm_config_t* cfg) { (void)id; (void)cfg; return 0; }
void pwm_enable(int id, int en) { if (id >= 0 && id < 4) s_pwm_enabled[id] = en ? 1 : 0; }
void pwm_set_duty(int id, float duty) { if (id >= 0 && id < 4) s_pwm_duty[id] = duty; }
float pwm_get_duty(int id) { return (id >= 0 && id < 4) ? s_pwm_duty[id] : -1.0f; }

/* ========== Serial 桩 ========== */
static char s_serial_buf[256];
static int s_serial_len = 0;

void serial_init(uint32_t baud) { (void)baud; }
void serial_putc(char c) { putchar(c); }
void serial_puts(const char* s) { printf("%s", s); }
int serial_line_available(void) { return 0; }
int serial_readline(char* buf, int max) { (void)buf; (void)max; return 0; }

/* ========== GPIO 桩 ========== */
void gpio_out_setup(uint8_t pin, uint8_t val) { (void)pin; (void)val; }
void gpio_out_write(uint8_t pin, uint8_t val) { (void)pin; (void)val; }
uint8_t gpio_in_read(uint8_t pin) { (void)pin; return 0; }
void gpio_in_setup(uint8_t pin, int pull) { (void)pin; (void)pull; }

/* ========== Scheduler 桩 ========== */
void sched_init(void) { }
void sched_main(void) { }
uint32_t sched_get_time(void) { return 0; }

/* ========== Stepper 桩 ========== */
void stepper_init(void) { }
int stepper_config(int id, uint8_t step, uint8_t dir, uint8_t en) { (void)id; (void)step; (void)dir; (void)en; return 0; }
void stepper_enable(int id, int en) { (void)id; (void)en; }
void stepper_set_next_step_time(int id, uint32_t time) { (void)id; (void)time; }

/* ========== Endstop 桩 ========== */
void endstop_init(void) { }
int endstop_config(int id, uint8_t pin, int pull) { (void)id; (void)pin; (void)pull; return 0; }
int endstop_read(int id) { (void)id; return 0; }
int endstop_is_triggered(int id) { (void)id; return 0; }
void endstop_set_callback(int id, void (*cb)(int, void*), void* arg) { (void)id; (void)cb; (void)arg; }
void endstop_home_start(int id) { (void)id; }
void endstop_home_end(int id) { (void)id; }

/* ========== Stepper 扩展桩 ========== */
int stepper_is_moving(int id) { (void)id; return 0; }
void stepper_stop(int id) { (void)id; }
void stepper_stop_all(void) { }

/* ========== Serial 扩展桩 ========== */
void serial_printf(const char* fmt, ...) { (void)fmt; }

