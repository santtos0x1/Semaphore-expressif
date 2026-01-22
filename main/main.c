#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "esp_log.h"

#define STACK_SIZE 1024

gpio_config_t red_led_io_conf = {
    // 00000...000000010000000000000000
    .pin_bit_mask = (1ULL << 16),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

gpio_config_t yel_led_io_conf = {
    // 00000...000000010000000000000000
    .pin_bit_mask = (1ULL << 21),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

gpio_config_t grn_led_io_conf = {
    // 00000...000000010000000000000000
    .pin_bit_mask = (1ULL << 19),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
};

gpio_config_t btn_io_conf = {
    .pin_bit_mask = (1ULL << 18),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_NEGEDGE
};

typedef struct {
    uint8_t red_led_pin;
    uint8_t yel_led_pin;
    uint8_t grn_led_pin;
    uint8_t btn_pin;
} gpio_pinout;

static const char* TAG = "runtime";

void vTaskCode( void * pvParameters )
{
    gpio_pinout* pins = ( gpio_pinout* ) pvParameters;
    const TickType_t xLdelay = 100;
    const TickType_t xHdelay = 3000;

    // Task
    for( ;; )
    {
        gpio_set_level( ( gpio_num_t )pins->grn_led_pin, 1 );

        if(gpio_get_level( ( gpio_num_t )pins->btn_pin ) == 0 )
        {
            gpio_set_level( ( gpio_num_t )pins->grn_led_pin, 0 );
            gpio_set_level( ( gpio_num_t )pins->red_led_pin, 1 );
            
            vTaskDelay( xHdelay );

            gpio_set_level( ( gpio_num_t )pins->red_led_pin, 0 );
            gpio_set_level( ( gpio_num_t )pins->grn_led_pin, 1 );
        }
    }
}

void taskBuilder( void )
{
    static gpio_pinout pinout;
    
    pinout.red_led_pin = 16;
    pinout.yel_led_pin = 21;
    pinout.grn_led_pin = 19;
    pinout.btn_pin = 18;

    TaskHandle_t xHandle = NULL;

    xTaskCreate( vTaskCode, "Led HIGH", STACK_SIZE, &pinout, tskIDLE_PRIORITY, &xHandle );
}

void app_main( void )
{
    gpio_config( &red_led_io_conf );
    gpio_config( &yel_led_io_conf );
    gpio_config( &grn_led_io_conf );
    gpio_config( &btn_io_conf );

    taskBuilder();
}
