#ifndef WS2812B_H
#define WS2812B_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GRB color structure (matches WS2812B native format)
 * 
 * WS2812B LEDs expect data in GRB order, not RGB.
 * This structure matches that requirement.
 */
typedef struct {
    uint8_t g;  ///< Green component (0-255)
    uint8_t r;  ///< Red component (0-255)  
    uint8_t b;  ///< Blue component (0-255)
} grb_t;

/**
 * @brief Initialize the WS2812B LED strip
 * 
 * Configures the RMT peripheral and creates the encoder for driving
 * WS2812B LEDs on the specified pin.
 * 
 * @return 
 *     - ESP_OK: Success
 *     - ESP_ERR_*: Various error codes on failure
 */
esp_err_t ws2812b_init(int led_pin, int num_leds);

/**
 * @brief Set LED strip colors
 * 
 * Updates all LEDs in the strip with the provided GRB color values.
 * The function blocks until transmission is complete.
 * 
 * @param grb_array Pointer to array of GRB color values
 *                  Must contain exactly NUM_LEDS elements
 * 
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: grb_array is NULL
 *     - ESP_ERR_INVALID_STATE: WS2812B not initialized
 *     - ESP_ERR_*: Other RMT transmission errors
 */
esp_err_t ws2812b_set_leds(const grb_t *grb_array, int num_leds);

/**
 * @brief Helper macro to create GRB color from RGB values
 * 
 * @param r Red component (0-255)
 * @param g Green component (0-255) 
 * @param b Blue component (0-255)
 * @return grb_t color structure
 */
static inline grb_t ws2812b_color(uint8_t r, uint8_t g, uint8_t b) {
    grb_t color = {.g = g, .r = r, .b = b};
    return color;
}

#define WS2812B_COLOR(r, g, b) ws2812b_color(r, g, b)

/**
 * @brief Predefined colors in GRB format
 */
#define WS2812B_BLACK    WS2812B_COLOR(0, 0, 0)
#define WS2812B_WHITE    WS2812B_COLOR(255, 255, 255)
#define WS2812B_RED      WS2812B_COLOR(255, 0, 0)
#define WS2812B_GREEN    WS2812B_COLOR(0, 255, 0)
#define WS2812B_BLUE     WS2812B_COLOR(0, 0, 255)
#define WS2812B_YELLOW   WS2812B_COLOR(255, 255, 0)
#define WS2812B_CYAN     WS2812B_COLOR(0, 255, 255)
#define WS2812B_MAGENTA  WS2812B_COLOR(255, 0, 255)

#ifdef __cplusplus
}
#endif

#endif // WS2812B_H
