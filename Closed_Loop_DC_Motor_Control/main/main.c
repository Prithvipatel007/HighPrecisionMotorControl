#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_attr.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#include "esp_log.h"
#include "rotary_encoder.h"

#define GPIO_PWM0A_OUT 19   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 18   //Set GPIO 16 as PWM0B

#define ROT_ENC_A_GPIO 34   // set GPIO 34 as Rotary Encoder GPIO A
#define ROT_ENC_B_GPIO 35   // set GPIO 35 as Rotary Encoder GPIO B

#define ENABLE_HALF_STEPS false  // Set to true to enable tracking of rotary encoder at half step resolution
#define RESET_AT          0      // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION    false  // Set to true to reverse the clockwise/counterclockwise sense

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

static void rotary_encoder_execute(void *arg){
  /*
  Rotary Encoder Configuration
  */
  // esp32-rotary-encoder requires that the GPIO ISR service is installed before calling rotary_encoder_register()
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // Initialise the rotary encoder device with the GPIOs for A and B signals
  rotary_encoder_info_t info = { 0 };
  ESP_ERROR_CHECK(rotary_encoder_init(&info, ROT_ENC_A_GPIO, ROT_ENC_B_GPIO));
  ESP_ERROR_CHECK(rotary_encoder_enable_half_steps(&info, ENABLE_HALF_STEPS));
#ifdef FLIP_DIRECTION
  ESP_ERROR_CHECK(rotary_encoder_flip_direction(&info));
#endif

  // Create a queue for events from the rotary encoder driver.
  // Tasks can read from this queue to receive up to date position information.
  QueueHandle_t event_queue = rotary_encoder_create_queue();
  ESP_ERROR_CHECK(rotary_encoder_set_queue(&info, event_queue));

  while (1)
  {
      // Wait for incoming events on the event queue.
      rotary_encoder_event_t event = { 0 };
      if (xQueueReceive(event_queue, &event, 1000 / portTICK_PERIOD_MS) == pdTRUE)
      {
          ESP_LOGI(TAG, "Event: position %d, direction %s", event.state.position,
                   event.state.direction ? (event.state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "clockwise" : "Anti-Clockwise") : "Not set");
      }
      else
      {
          // Poll current position and direction
          rotary_encoder_state_t state = { 0 };
          ESP_ERROR_CHECK(rotary_encoder_get_state(&info, &state));
          ESP_LOGI(TAG, "Poll: position %d, direction %s", state.position,
                   state.direction ? (state.direction == ROTARY_ENCODER_DIRECTION_CLOCKWISE ? "clockwise" : "Anti-Clockwise") : "Not Set");

          // Reset the device
          if (RESET_AT && (state.position >= RESET_AT || state.position <= -RESET_AT))
          {
              ESP_LOGI(TAG, "Reseting the Device");
              ESP_ERROR_CHECK(rotary_encoder_reset(&info));
          }
      }
  }
  ESP_LOGE(TAG, "queue receive failed");

  ESP_ERROR_CHECK(rotary_encoder_uninit(&info));
}

static void mcpwm_execute(void *arg){
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

  while(1){
    // Move Clockwise
    brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 50.0);
    vTaskDelay(2000/portTICK_RATE_MS);

    //Stopped
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    vTaskDelay(2000/portTICK_RATE_MS);

    // Move Anti-Clockwise
    brushed_motor_backward(MCPWM_UNIT_0, MCPWM_TIMER_0, 30.0);
    vTaskDelay(2000/portTICK_RATE_MS);

    // Stopped
    brushed_motor_stop(MCPWM_UNIT_0, MCPWM_TIMER_0);
    vTaskDelay(2000/portTICK_RATE_MS);
  }
}

void app_main(void)
{
  printf("Testing brushed motor... \n");
  xTaskCreate(mcpwm_execute, "mcpwm_execute", 4096, NULL, 5, NULL);
  xTaskCreate(rotary_encoder_execute, "rotary_encoder_execute", 4096, NULL, 5, NULL);
}
