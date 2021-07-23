#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "esp_log.h"
#include "rotary_encoder.h"

#define GPIO_PWM0A_OUT 19   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 16 as PWM0B

static const char *TAG = "Simple_DC_Motor_Control";

/**
 GPIO initialization
*/
static void mcpwm_example_gpio_initialize(void){
  printf("initialize mcpwm gpio...\n");
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
  mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

/*
  Motor moves in forward direction, with duty cycle = duty %
*/
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){
  printf("Turing motor forward... \n");
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
}

/*
  Motor moves in backward direction, with duty cycle = duty %
*/
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle){
  printf("Turing motor backward... \n");
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
  mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle);
  mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); // call this each time, if operator was previously in low/high state
}

static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num){
  printf("Motor Stopped \n");
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
  mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void mcpwm_execute(void *arg){
  // PCNT unit between 0 to 7
  uint32_t pcnt_unit = 0;
  /*
    DC motor configurations
  */
  // 1. initialization of the GPIOs
  mcpwm_example_gpio_initialize();

  //2. Initialize mcpwm configuration
  printf("Configuring Initial Parameters of mcpwm... \n");
  mcpwm_config_t pwm_config;
  pwm_config.frequency = 1000;  // frequency = 500 Hz
  pwm_config.cmpr_a = 0;  // duty cycle for PWMxA
  pwm_config.cmpr_b = 0;  // duty cycle for PWMxB
  pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
  pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
  mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); // configure PWM0A & PWM0B with above settings

  /*
    Rotary Encoder Configuration
  */

  // create rotary encoder instance (pcnt_unit, pwn_gpio1, pwm_gpio2)
  rotary_encoder_config_t config = ROTARY_ENCODER_DEFAULT_CONFIG((rotary_encoder_dev_t)pcnt_unit, 34, 35);
  rotary_encoder_t *encoder = NULL;
  ESP_ERROR_CHECK(rotary_encoder_new_ec11(&config, &encoder));

  // Filter out glitches (1us)
  ESP_ERROR_CHECK(encoder -> set_glitch_filter(encoder,1));

  // Start Encoder
  ESP_ERROR_CHECK(encoder -> start(encoder));

  while(1){
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
    vTaskDelay(2000/portTICK_RATE_MS);
    ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));

    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    vTaskDelay(2000/portTICK_RATE_MS);
    ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));
    /*brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
    ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));
    vTaskDelay(2000/portTICK_RATE_MS);
    ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    ESP_LOGI(TAG, "Encoder value : %d", encoder->get_counter_value(encoder));
    vTaskDelay(2000/portTICK_RATE_MS);*/
  }
}

void app_main(void)
{
  printf("Testing brushed motor... \n");
  xTaskCreate(mcpwm_execute, "mcpwm_execute", 4096, NULL, 5, NULL);
}
