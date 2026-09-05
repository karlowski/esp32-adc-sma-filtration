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

#define TIME_DIVIDER        1000
#define LOGGER_THRESHOLD_MS 500
#define SENSOR_THRESHOLD_MV 2400

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

void handle_led(uint8_t pin)
{

}

void adc_filter(int vt_mv, int* cleaned_mv)
{
    *cleaned_mv = vt_mv;
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

    if ((now - last_log_at) > LOGGER_THRESHOLD_MS)
    {
        last_log_at = now;

        int raw = 0;
        int voltage_calibrated_mv = 0;
        int voltage_cleaned_mv = 0;

        adc_oneshot_read(adc_handle, ADC_CHANNEL, &raw);
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltage_calibrated_mv);

        // TODO: analog signal cleaning algorythm
        adc_filter(voltage_calibrated_mv, &voltage_cleaned_mv);

        uint8_t led_level = voltage_cleaned_mv >= SENSOR_THRESHOLD_MV ? 0 : 1;

        gpio_set_level(LED_PIN, led_level);

        ESP_LOGI("ADC", "calibrated (mv): %d; cleaned (mv): %d", voltage_calibrated_mv, voltage_cleaned_mv);
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
