#include <stdio.h>
#include <stdlib.h>   // calloc/free
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"   // for ESP_RETURN_ON_ERROR
#include "ledstrip.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us

static const char *TAG = "WS2812B";
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// WS2812B timing (ticks of 0.1us)
// Typical: 0 = 0.4H/0.85L, 1 = 0.8H/0.45L
#define T0H 4
#define T0L 9
#define T1H 8
#define T1L 5
#define RESET_TIME 800 // 80us (in ticks)

// Encoder implementation
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

enum {
    WS2812B_ENCODING_RGB,
    WS2812B_ENCODING_RESET_TRAIL,
    WS2812B_ENCODING_COMPLETE
};

static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                   const void *primary_data, size_t data_size,
                                   rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *enc = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = enc->bytes_encoder;
    rmt_encoder_handle_t copy_encoder  = enc->copy_encoder;
    rmt_encode_state_t session_state = 0;
    rmt_encode_state_t state = 0;
    size_t encoded_symbols = 0;

    switch (enc->state) {
    case WS2812B_ENCODING_RGB:
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel,
                                                 primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        } else if (session_state & RMT_ENCODING_COMPLETE) {
            enc->state = WS2812B_ENCODING_RESET_TRAIL;
        }
        break;

    case WS2812B_ENCODING_RESET_TRAIL:
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &enc->reset_code,
                                                sizeof(enc->reset_code), &session_state);
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
        } else {
            enc->state = WS2812B_ENCODING_COMPLETE;
            state |= RMT_ENCODING_COMPLETE;
        }
        break;

    case WS2812B_ENCODING_COMPLETE:
        state |= RMT_ENCODING_COMPLETE;
        break;
    }

    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *enc = __containerof(encoder, rmt_led_strip_encoder_t, base);
    if (enc->bytes_encoder) rmt_del_encoder(enc->bytes_encoder);
    if (enc->copy_encoder)  rmt_del_encoder(enc->copy_encoder);
    free(enc);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *enc = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(enc->bytes_encoder);
    rmt_encoder_reset(enc->copy_encoder);
    enc->state = WS2812B_ENCODING_RGB;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret;
    rmt_led_strip_encoder_t *enc = calloc(1, sizeof(*enc));
    if (!enc) return ESP_ERR_NO_MEM;

    enc->base.encode = rmt_encode_led_strip;
    enc->base.del    = rmt_del_led_strip_encoder;
    enc->base.reset  = rmt_led_strip_encoder_reset;

    // bytes encoder (MSB first)
    rmt_bytes_encoder_config_t bytes_cfg = {
        .bit0 = { .level0 = 1, .duration0 = T0H, .level1 = 0, .duration1 = T0L },
        .bit1 = { .level0 = 1, .duration0 = T1H, .level1 = 0, .duration1 = T1L },
        .flags.msb_first = 1
    };
    ret = rmt_new_bytes_encoder(&bytes_cfg, &enc->bytes_encoder);
    if (ret != ESP_OK) { free(enc); return ret; }

    // copy encoder for reset
    rmt_copy_encoder_config_t copy_cfg = {};
    ret = rmt_new_copy_encoder(&copy_cfg, &enc->copy_encoder);
    if (ret != ESP_OK) { rmt_del_encoder(enc->bytes_encoder); free(enc); return ret; }

    // Reset symbol: line low for RESET_TIME*2 ticks total
    enc->reset_code = (rmt_symbol_word_t){ .level0 = 0, .duration0 = RESET_TIME,
                                           .level1 = 0, .duration1 = RESET_TIME };
    enc->state = WS2812B_ENCODING_RGB;

    *ret_encoder = &enc->base;
    return ESP_OK;
}

esp_err_t ws2812b_init(int led_pin, int num_leds)
{
    ESP_LOGI(TAG, "Initializing WS2812B on pin %d for %d LEDs", led_pin, num_leds);

    rmt_tx_channel_config_t tx_cfg = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = led_pin,
        .mem_block_symbols = 64,
        .resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_cfg, &led_chan), TAG, "rmt_new_tx_channel failed");
    ESP_RETURN_ON_ERROR(rmt_new_led_strip_encoder(&led_encoder), TAG, "new_led_strip_encoder failed");
    ESP_RETURN_ON_ERROR(rmt_enable(led_chan), TAG, "rmt_enable failed");

    ESP_LOGI(TAG, "WS2812B initialization complete");
    return ESP_OK;
}

esp_err_t ws2812b_set_leds(const grb_t *grb_array, int num_leds)
{
    if (!grb_array) return ESP_ERR_INVALID_ARG;
    if (!led_chan || !led_encoder) return ESP_ERR_INVALID_STATE;

    uint8_t led_strip_pixels[num_leds * 3];
    for (int i = 0; i < num_leds; i++) {
        led_strip_pixels[i*3 + 0] = grb_array[i].g;
        led_strip_pixels[i*3 + 1] = grb_array[i].r;
        led_strip_pixels[i*3 + 2] = grb_array[i].b;
    }

    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };

    // Reset encoder state before each transmit
    rmt_encoder_reset(led_encoder);

    esp_err_t err = rmt_transmit(led_chan, led_encoder,
                                 led_strip_pixels, (num_leds * 3),
                                 &tx_cfg);
    if (err != ESP_OK) return err;

    return rmt_tx_wait_all_done(led_chan, portMAX_DELAY);
}

