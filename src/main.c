#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define ADC_UNIT ADC_UNIT_1
#define ADC_CHANNEL ADC_CHANNEL_3 // GPIO #4
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

#define LED_PIN 5
#define MOVING_AVERAGE_N 5

#define TIME_DIVIDER 1000
#define MOVING_AVERAGE_POLLING_MS 20
#define OPERATIONAL_STEP_MS 100
#define LOGGER_THRESHOLD_MS 500
#define SENSOR_THRESHOLD_MV 2400

uint64_t last_operation_at = 0;
uint64_t last_log_at = 0;

adc_oneshot_unit_handle_t adc_handle;
adc_oneshot_unit_init_cfg_t adc_init_config = {
    .unit_id = ADC_UNIT,
};
adc_oneshot_chan_cfg_t adc_chan_config = {
    .bitwidth = ADC_BITWIDTH,
    .atten = ADC_ATTEN,
};
adc_cali_handle_t adc_cali_handle;
adc_cali_curve_fitting_config_t adc_cali_config = {
    .unit_id = ADC_UNIT,
    .atten = ADC_ATTEN,
    .bitwidth = ADC_BITWIDTH,
    .chan = ADC_CHANNEL,
};

gpio_config_t gpio_config_led = {
    .pin_bit_mask = 1ULL << LED_PIN,
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
};

typedef struct
{
    float value_prev;
    int values[MOVING_AVERAGE_N];
    uint64_t updated_at;
    uint8_t length;
} moving_average_config;

moving_average_config mv_config = {
    .value_prev = 0,
    .values = {0},
    .length = MOVING_AVERAGE_N,
    .updated_at = 0,
};

void adc_sma_filter(moving_average_config* config) // simple moving average
{
    int sum = 0;

    for (uint8_t i = 0; i < config->length; i++)
    {
        sum = sum + config->values[i];
    }

    config->value_prev = (float)sum / config->length;
}

void init()
{
    adc_oneshot_new_unit(&adc_init_config, &adc_handle);
    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &adc_chan_config);
    adc_cali_create_scheme_curve_fitting(&adc_cali_config, &adc_cali_handle);

    gpio_config(&gpio_config_led);
}

void loop()
{
    uint64_t now = esp_timer_get_time() / TIME_DIVIDER;

    if ((now - mv_config.updated_at) >= MOVING_AVERAGE_POLLING_MS)
    {
        mv_config.updated_at = now;

        for (uint8_t i = 0; i < mv_config.length; i++)
        {
            if (i == (mv_config.length - 1))
            {
                adc_oneshot_read(adc_handle, ADC_CHANNEL, &mv_config.values[i]); // pre-writing it raw
                adc_cali_raw_to_voltage(adc_cali_handle, mv_config.values[i], &mv_config.values[i]);
                break;
            }
            mv_config.values[i] = mv_config.values[i + 1];
        }
    }

    if ((now - last_operation_at) >= OPERATIONAL_STEP_MS)
    {
        last_operation_at = now;

        adc_sma_filter(&mv_config);
        uint8_t led_level = mv_config.value_prev >= SENSOR_THRESHOLD_MV ? 0 : 1;
        gpio_set_level(LED_PIN, led_level);
    }

    if ((now - last_log_at) >= LOGGER_THRESHOLD_MS)
    {
        last_log_at = now;
        ESP_LOGI("ADC", "raw (mv): %d; cleaned (mv): %f", mv_config.values[MOVING_AVERAGE_N - 1], mv_config.value_prev);
    }
}

void app_main()
{
    init();

    while (1)
    {
        loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
