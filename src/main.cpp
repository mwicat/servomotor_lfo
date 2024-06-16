#include <Arduino.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/ledc_reg.h"

#include "servo_lfo.h"

const int servo0_output_pin = GPIO_NUM_18;
const int servo1_output_pin = GPIO_NUM_19;

static int ch0_sine_table_index = 0;
static int ch1_sine_table_index = 0;

static int pwm_duty = 127;

unsigned int ch0_sine_table_incr = 80;
unsigned int ch1_sine_table_incr = 85;

void IRAM_ATTR ledc_timer0_overflow_isr(void *arg)
{
	// clear the interrupt
	REG_SET_BIT(LEDC_INT_CLR_REG, LEDC_HSTIMER0_OVF_INT_CLR);

	// update duty, shift the duty 4 bits to the left due to ESP32 register format
	int ch0_sine_full_val = SINE_LOOKUP_TABLE[ch0_sine_table_index];
	int ch0_duty_val = map(ch0_sine_full_val, 0, 4096, 96, 300);

	int ch1_sine_full_val = SINE_LOOKUP_TABLE[ch1_sine_table_index];
	int ch1_duty_val = map(ch1_sine_full_val, 0, 4096, 96, 300);

	REG_WRITE(LEDC_HSCH0_DUTY_REG, ch0_duty_val << 4);
	REG_SET_BIT(LEDC_HSCH0_CONF1_REG, LEDC_DUTY_START_HSCH0);

	REG_WRITE(LEDC_HSCH1_DUTY_REG, ch1_duty_val << 4);
	REG_SET_BIT(LEDC_HSCH1_CONF1_REG, LEDC_DUTY_START_HSCH1);

	ch0_sine_table_index = (ch0_sine_table_index + ch0_sine_table_incr) % SINE_TABLE_SIZE;
	ch1_sine_table_index = (ch1_sine_table_index + ch1_sine_table_incr) % SINE_TABLE_SIZE;
}

ledc_timer_config_t create_ledc_timer_config(ledc_timer_t timer_num, uint32_t freq_hz)
{
	ledc_timer_config_t ledc_timer0;
	ledc_timer0.duty_resolution = LEDC_TIMER_12_BIT;
	ledc_timer0.freq_hz = freq_hz;
	ledc_timer0.speed_mode = LEDC_HIGH_SPEED_MODE;
	ledc_timer0.timer_num = timer_num;
	return ledc_timer0;
}

ledc_channel_config_t create_ledc_channel_config(ledc_channel_t channel, int gpio_num)
{
	ledc_channel_config_t ledc_channel0;
	ledc_channel0.channel = channel;
	ledc_channel0.duty = 0;
	ledc_channel0.gpio_num = gpio_num;
	ledc_channel0.hpoint = 0;
	ledc_channel0.timer_sel = LEDC_TIMER_0;
	ledc_channel0.speed_mode = LEDC_HIGH_SPEED_MODE;
	ledc_channel0.intr_type = LEDC_INTR_DISABLE;
	return ledc_channel0;
}

void initSPWM()
{
	// configure ledc timer0
	ledc_timer_config_t ledc_timer0 = create_ledc_timer_config(LEDC_TIMER_0, STEP_FREQUENCY);
	ledc_timer_config(&ledc_timer0);

	ledc_channel_config_t ledc_channel0 = create_ledc_channel_config(LEDC_CHANNEL_0, servo0_output_pin);
	ledc_channel_config(&ledc_channel0);

	ledc_channel_config_t ledc_channel1 = create_ledc_channel_config(LEDC_CHANNEL_1, servo1_output_pin);
	ledc_channel_config(&ledc_channel1);

	// register overflow interrupt handler for timer0
	ledc_isr_register(ledc_timer0_overflow_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
	// enable the overflow interrupt
	REG_SET_BIT(LEDC_INT_ENA_REG, LEDC_HSTIMER0_OVF_INT_ENA);
}

bool readLineSerial(char *buf, size_t length)
{
	size_t read_sz = 0;
	bool got_input = (Serial.available() > 0);

	if (got_input)
	{
		read_sz = Serial.readBytesUntil('\n', buf, length);
		if (read_sz > 0 && buf[read_sz - 1] == '\r')
			read_sz--;
		buf[read_sz] = '\0';
	}
	return got_input;
}

void setup()
{
	Serial.begin(115200);
	initSPWM();
}

void loop()
{
	//   char line_buf[255];
	//   bool got_input = readLineSerial(line_buf, sizeof(line_buf));
	//   if (got_input) {
	// 	char* end;
	// 	pwm_duty = strtol(line_buf, &end, 10);
	// 	Serial.printf("duty: %d\n", pwm_duty);
	//   }
}
