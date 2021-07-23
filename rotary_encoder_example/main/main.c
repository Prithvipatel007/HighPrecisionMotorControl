#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "rotary_encoder.h"

#define TAG "Rotary Encoder Example"

#define ROT_ENC_A_GPIO 34
#define ROT_ENC_B_GPIO 35

#define ENABLE_HALF_STEPS false // set to true to enable tracking of the rotary encoder at half setp resolution
#define RESET_AT 0 // Set to a positive non-zero number to reset the position if this value is exceeded
#define FLIP_DIRECTION false // Set to true to reverse the clockwise/counter-clockwise sense

void app_main()
{
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
