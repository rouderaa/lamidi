/*
 * SPDX-FileCopyrightText: 2025 R vd Ouderaa
 *
 * SPDX-License-Identifier: MIT
 *
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include <math.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_mac.h"

#include "ledstrip.h"

#define NUM_LEDS 123
#define LED_PIN 13

static const char *TAG = "midi_bridge";

// UART configuration
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     31250  // Standard MIDI baud rate
#define UART_TX_PIN        6      // MIDI OUT
#define UART_RX_PIN        5      // MIDI IN
#define UART_BUF_SIZE      1024

// USB Endpoint numbers
enum usb_endpoints {
    EP_EMPTY = 0,
    EPNUM_MIDI = 1,
};

// Interface definitions - MIDI only
#define ITF_NUM_MIDI    0
#define ITF_NUM_MIDI_STREAMING 1
#define ITF_COUNT 2

/** TinyUSB descriptors **/
#define TUSB_DESCRIPTOR_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MIDI_DESC_LEN)

// forward declaration
static void show_note(int note, bool on);

// Helper function to determine MIDI message length
static int get_midi_message_length(uint8_t status_byte) {
    uint8_t status = status_byte & 0xF0;
    
    if (status_byte >= 0xF0) {
        // System messages
        switch (status_byte) {
            case 0xF1: // MTC Quarter Frame
            case 0xF3: // Song Select
                return 2;
            case 0xF2: // Song Position Pointer
                return 3;
            case 0xF6: // Tune Request
            case 0xF8: // Timing Clock
            case 0xFA: // Start
            case 0xFB: // Continue
            case 0xFC: // Stop
            case 0xFE: // Active Sensing
            case 0xFF: // System Reset
                return 1;
            default:
                return 1;
        }
    } else {
        // Channel messages
        switch (status) {
            case 0xC0: // Program Change
            case 0xD0: // Channel Pressure
                return 2;
            case 0x80: // Note Off
            case 0x90: // Note On
            case 0xA0: // Polyphonic Key Pressure
            case 0xB0: // Control Change
            case 0xE0: // Pitch Bend Change
                return 3;
            default:
                return 3; // Default to 3 bytes for unknown channel messages
        }
    }
}

/**
 * @brief String descriptor
 */
static const char* s_str_desc[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "ESP32 Audio",         // 1: Manufacturer
    "ESP32-S3 MIDI Bridge",// 2: Product
    "123456",              // 3: Serials, should use chip ID
    "MIDI UART Bridge",    // 4: MIDI
};

static const uint8_t s_cfg_desc[] = {
    // Configuration descriptor: 1 config, ITF_COUNT interfaces, string index 0, total length, attributes, 100 mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),

    // MIDI descriptor
    TUD_MIDI_DESCRIPTOR(
        ITF_NUM_MIDI,        // MIDI control interface
        4,                   // String index
        EPNUM_MIDI,          // MIDI OUT EP
        (0x80 | EPNUM_MIDI), // MIDI IN EP
        64                   // EP size
    ),
};

#if (TUD_OPT_HIGH_SPEED)
/**
 * @brief High Speed configuration descriptor
 */
static const uint8_t s_midi_hs_cfg_desc[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, TUSB_DESCRIPTOR_TOTAL_LEN, 0, 100),
    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MIDI_DESCRIPTOR(ITF_NUM_MIDI, 4, EPNUM_MIDI, (0x80 | EPNUM_MIDI), 512),
};
#endif // TUD_OPT_HIGH_SPEED


// Initialize UART for MIDI communication
static void uart_init(void)
{
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized - RX: GPIO%d, TX: GPIO%d, Baud: %d", UART_RX_PIN, UART_TX_PIN, UART_BAUD_RATE);
}


static void control_leds(uint8_t *packet) {
   uint8_t sb;
   uint8_t note;
   uint8_t velocity;
  
   packet++; // skip timestamp 
   sb = *packet; // status byte 
   packet++;
   note = *packet;
   packet++;
   velocity = *packet;

   if ((sb & 0xf0) == 0x90) {
        if (velocity == 0) {
		// Note off
		show_note((int) note, false);
	} else {
   		// Note on
		show_note((int) note, true);
	}
   } else
   if ((sb & 0xf0) == 0x80) {
   	// Note off
	show_note((int) note, false);
   }
}

static uint8_t last_status = 0;  // For running status tracking

// Task to read USB MIDI and forward to UART
static void usb_to_uart_task(void *arg)
{
    uint8_t packet[4];
    uint8_t midi_data[3];
    
    ESP_LOGI(TAG, "USB->UART - task activated");

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(30));
        
        while (tud_midi_available()) {
            if (tud_midi_packet_read(packet)) {
		// MIDI packet logging
                // ESP_LOGI(TAG, "USB->UART - Packet: %02X %02X %02X %02X", packet[0], packet[1], packet[2], packet[3]);

		control_leds(packet);
                
                // Extract MIDI data from USB MIDI packet (skip cable number and code index)
                // Standard MIDI messages are 3 bytes: status, data1, data2
                midi_data[0] = packet[1];  // Status byte
                midi_data[1] = packet[2];  // Data byte 1
                midi_data[2] = packet[3];  // Data byte 2
                
                // Determine message length based on status byte
                int msg_len = get_midi_message_length(packet[1]);
                
                // Handle running status for UART output
                if (packet[1] >= 0x80) {
                    // New status byte
                    last_status = packet[1];
                    // Send full message
                    uart_write_bytes(UART_PORT_NUM, midi_data, msg_len);
                } else {
                    // This could be a running status message
                    // But USB MIDI packets should always have status bytes, so this is unusual
                    // Send as-is but update running status tracking
                    uart_write_bytes(UART_PORT_NUM, midi_data, msg_len);
                }
            }
        }
    }
}

#ifdef NOTUSED
static void dump_midi_data(uint8_t *data, int len) {
   char line[80];

   // if not ((Timing clock) or (Active Sensing)) and  
   if ((*data != 0xf8) && (*data != 0xfe)) {
	   char *p = line;
	   for(int index = 0; ((index < len) && (index < 5)); index++) {
		sprintf(p, "%02X ", (*data & 0xff));
		p = p + 3;
		data++;
	   }
	   *p = '\0';
	   ESP_LOGI(TAG, "data : %s", line);
   }
}
#endif

// Task to read UART MIDI and forward to USB
static void uart_to_usb_task(void *arg)
{
    uint8_t data[UART_BUF_SIZE];
    static uint8_t cable_num = 0;
    
    // Running status state variables
    static uint8_t last_status = 0;
    static uint8_t midi_msg[3];
    static int msg_idx = 0;
    static int expected_len = 0;
    
    ESP_LOGI(TAG, "UART->USB - task activated");

    // Add a longer initial delay
    // vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second delay
    
    // ESP_LOGI(TAG, "UART->USB - entering main loop");

    for (;;) {
        int len = uart_read_bytes(UART_PORT_NUM, data, sizeof(data), pdMS_TO_TICKS(10));
        
        if (len > 0) {
            // ESP_LOGI(TAG, "UART->USB - Received %d bytes", len);
	    // dump_midi_data(data, len);
            
            // Process received MIDI data with running status support
            for (int i = 0; i < len; i++) {
                uint8_t byte = data[i];
                
                // Check if this is a status byte (MSB set)
                if (byte & 0x80) {
                    // New status byte received
                    
                    // System Real-Time messages can interrupt other messages
                    if (byte >= 0xF8) {
                        // Real-time message - send immediately without affecting running status
                        uint8_t realtime_msg[1] = {byte};
                        if (tud_midi_mounted()) {
                            tud_midi_stream_write(cable_num, realtime_msg, 1);
                        }
                        continue; // Don't reset the current message parsing
                    }
                    
                    // System Common messages cancel running status
                    if (byte >= 0xF0 && byte < 0xF8) {
                        last_status = 0; // Clear running status
                    } else {
                        last_status = byte; // Update running status
                    }
                    
                    // Reset message buffer for new message
                    msg_idx = 0;
                    midi_msg[msg_idx++] = byte;
                    expected_len = get_midi_message_length(byte);
                    
                } else {
                    // Data byte received
                    if (msg_idx == 0 && last_status != 0) {
                        // Running status: use the last status byte
                        midi_msg[msg_idx++] = last_status;
                        midi_msg[msg_idx++] = byte;
                        expected_len = get_midi_message_length(last_status);
                    } else if (msg_idx > 0 && msg_idx < 3) {
                        // Normal data byte for current message
                        midi_msg[msg_idx++] = byte;
                    } else {
                        // Unexpected data byte - ignore or handle error
                        ESP_LOGW(TAG, "Unexpected data byte: 0x%02X", byte);
                        continue;
                    }
                }
                
                // Send complete message to USB
                if (msg_idx == expected_len && tud_midi_mounted()) {
                    tud_midi_stream_write(cable_num, midi_msg, expected_len);
                    
                    // Reset message index but keep running status
                    if (midi_msg[0] < 0xF0) {
                        // Channel message - prepare for potential running status
                        msg_idx = 0;
                    } else {
                        // System message - fully reset
                        msg_idx = 0;
                        if (midi_msg[0] >= 0xF0 && midi_msg[0] < 0xF8) {
                            last_status = 0; // System common messages clear running status
                        }
                    }
                }
            }
        }
    }
}

// Function to convert MIDI note to 61-key keyboard position
// Standard 61-key keyboard range: C2 (MIDI 36) to C7 (MIDI 96)
int midi_to_61key_position(int midi_note) {
    const int MIN_MIDI = 36;  // C2 - lowest key on 61-key keyboard
    const int MAX_MIDI = 96;  // C7 - highest key on 61-key keyboard
    
    // Check if MIDI note is within 61-key range
    if (midi_note < MIN_MIDI || midi_note > MAX_MIDI) {
        return -1;  // Invalid: outside keyboard range
    }
    
    // Calculate position (1-based indexing)
    return midi_note - MIN_MIDI + 1;
}

// Helper function to get note name from MIDI number
void midi_to_note_name(int midi_note, char* note_name) {
    const char* notes[] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    int octave = (midi_note / 12) - 1;
    int note_index = midi_note % 12;
    
    sprintf(note_name, "%s%d", notes[note_index], octave);
}

static grb_t led_array[NUM_LEDS];

/**
 * @brief Simple Knight Rider effect with just one LED moving back and forth
 * 
 * @param delay_ms Delay between LED movements in milliseconds
 * @return ESP_OK on success, error code on failure
 */
esp_err_t knight_rider_simple(uint32_t delay_ms)
{
    // grb_t led_array[NUM_LEDS];
    esp_err_t ret;
    
    while (true) {
        // Move right (0 to NUM_LEDS-1)
        for (int pos = 0; pos < NUM_LEDS-1; pos++) {
            // Clear all LEDs
            memset(led_array, 0, sizeof(led_array));
	    // Mark end of leds start and finish
            led_array[0] = WS2812B_COLOR(0, 50, 0);
            led_array[NUM_LEDS-2] = WS2812B_COLOR(0, 50, 0);
            
            // Set current LED to red
            led_array[pos] = WS2812B_COLOR(50, 0, 0);
            
    	    ESP_LOGI(TAG, "set_leds 1");
            ret = ws2812b_set_leds(led_array, NUM_LEDS);
            if (ret != ESP_OK) {
    	        ESP_LOGI(TAG, "ret error %d", ret);
    		ESP_LOGE(TAG, "ws2812b_set_leds failed");
                return ret;
            }
    	    ESP_LOGI(TAG, "set_leds 2");
            
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
        
        // Move left (NUM_LEDS-1 down to 0)
        for (int pos = NUM_LEDS - 2; pos > 0; pos--) {
            // Clear all LEDs  
            memset(led_array, 0, sizeof(led_array));
            
            // Set current LED to red
            led_array[pos] = WS2812B_COLOR(50, 0, 0);
            
    	    // ESP_LOGI(TAG, "set_leds 3");
            ret = ws2812b_set_leds(led_array, NUM_LEDS);
            if (ret != ESP_OK) {
    		ESP_LOGE(TAG, "ws2812b_set_leds failed");
                return ret;
            }
    	    // ESP_LOGI(TAG, "set_leds 4");
            
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
    
    return ESP_OK;
}

/**
 * Get the note number within an octave from MIDI note number
 * 
 * @param midiNote MIDI note number (0-127)
 * @return Note number in octave (0-11), where 0=C, 1=C#, 2=D, etc.
 */
uint8_t getNoteInOctave(uint8_t midiNote) {
    // Validate MIDI note range
    if (midiNote > 127) {
        return 0; // Default to C if invalid
    }
    
    return midiNote % 12;
}

bool isSemitoneFast(uint8_t midiNote) {
    if (midiNote > 127) {
        return false;
    }
    
    // Bitmask for semitone positions: 1010100010100 (binary)
    // Positions 1,3,6,8,10 are set to 1
    const uint16_t semitoneMask = 0x054A; // 0000010101001010
    
    uint8_t noteInOctave = midiNote % 12;
    return (semitoneMask >> noteInOctave) & 1;
}

// Color ratios for each chromatic note (multiply by brightness)
static float colorRatios[12][3] = {
        {1.0, 0.0, 0.0},  // C - Red
        {1.0, 0.5, 0.0},  // C# - Red-Orange
        {1.0, 1.0, 0.0},  // D - Yellow
        {0.5, 1.0, 0.0},  // D# - Yellow-Green
        {0.0, 1.0, 0.0},  // E - Green
        {0.0, 1.0, 0.5},  // F - Green-Cyan
        {0.0, 1.0, 1.0},  // F# - Cyan
        {0.0, 0.5, 1.0},  // G - Cyan-Blue
        {0.0, 0.0, 1.0},  // G# - Blue
        {0.5, 0.0, 1.0},  // A - Blue-Purple
        {1.0, 0.0, 1.0},  // A# - Purple
        {1.0, 0.0, 0.5}   // B - Purple-Red
};
    
static void setNoteColorArray(int pos, uint8_t midiNote, uint8_t brightness, bool on) {
    uint8_t noteInOctave = getNoteInOctave(midiNote);
    
    uint8_t red = (uint8_t)(colorRatios[noteInOctave][0] * brightness);
    uint8_t green = (uint8_t)(colorRatios[noteInOctave][1] * brightness);
    uint8_t blue = (uint8_t)(colorRatios[noteInOctave][2] * brightness);
  
    pos = pos + 1; 
    if ((pos > 0) && (pos < NUM_LEDS)) {
	if (pos < 73) pos = pos + 1; // Skip missing led 
	if (on) 
    		led_array[pos] = WS2812B_COLOR(red, green, blue);
	else
    		led_array[pos] = WS2812B_COLOR(0, 0, 0);
	if (pos > 1) {
		if (on) 
    			led_array[pos-1] = WS2812B_COLOR(red, green, blue);
		else
    			led_array[pos-1] = WS2812B_COLOR(0, 0, 0);
	}
    } 
}

static void show_note(int note, bool on) {
    int pos = 120+((36-note)*2);
    // sanity check
    if ((pos >= 0) && (pos <= (NUM_LEDS-2)) ) {
	setNoteColorArray(pos, note, 40, on);
   
    	int ret = ws2812b_set_leds(led_array, NUM_LEDS);
    	if (ret != ESP_OK) {
    		ESP_LOGE(TAG, "ws2812b_set_leds failed");
        	return;
    	}
    	// ESP_LOGI(TAG, "show_note: %d %d", note, pos);
    } else {
    	ESP_LOGI(TAG, "show_note: pos out of range %d %d", note, pos);
    }
}

static void clear_all_leds() {
    // Clear all LEDs  
    memset(led_array, 0, sizeof(led_array));
    ws2812b_set_leds(led_array, NUM_LEDS);
}

#ifdef NOTUSED
static void show_all_notes() {
    // Clear all LEDs  
    memset(led_array, 0, sizeof(led_array));

    // for(;;) {
    	for(int note = 0x24; note <= 0x60; note++) {

    	    show_note(note, true);

            vTaskDelay(pdMS_TO_TICKS(500));
    	}
    // }
}
#endif

static char serial_str[13];

void app_main(void)
{
    ESP_LOGI(TAG, "USB MIDI initialization");

    // Generate unique serial number from MAC address
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(serial_str, sizeof(serial_str), "%02X%02X%02X%02X%02X%02X", 
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    ESP_LOGI(TAG, "MAC Address: %s", serial_str);
    
    // Update serial number in string descriptor
    s_str_desc[3] = serial_str;

    tinyusb_config_t const tusb_cfg = {
        .device_descriptor = NULL, // If device_descriptor is NULL, tinyusb_driver_install() will use Kconfig
        .string_descriptor = s_str_desc,
        .string_descriptor_count = sizeof(s_str_desc) / sizeof(s_str_desc[0]),
        .external_phy = false,
#if (TUD_OPT_HIGH_SPEED)
        .fs_configuration_descriptor = s_cfg_desc, 
        .hs_configuration_descriptor = s_midi_hs_cfg_desc,
        .qualifier_descriptor = NULL,
#else
        .configuration_descriptor = s_cfg_desc,
#endif // TUD_OPT_HIGH_SPEED
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB MIDI initialization DONE");

    // Initialize UART for MIDI
    uart_init();

    // Create tasks for bidirectional MIDI bridging
    ESP_LOGI(TAG, "Creating MIDI bridge tasks");
    ESP_LOGI(TAG, "Creating usb_to_uart_task");
    BaseType_t result1 = xTaskCreate(usb_to_uart_task, "usb_to_uart_task", 8 * 1024, NULL, 5, NULL);
    if (result1 != pdPASS) {
        ESP_LOGE(TAG, "Failed to create usb_to_uart_task: %d", result1);
    }
    ESP_LOGI(TAG, "Creating uart_to_usb_task");
    BaseType_t result2 = xTaskCreate(uart_to_usb_task, "uart_to_usb_task", 8 * 1024, NULL, 5, NULL);
    if (result2 != pdPASS) {
    	ESP_LOGE(TAG, "Failed to create uart_to_usb_task: %d", result2);
    }

    // Setup ledstrip
    ws2812b_init(LED_PIN, NUM_LEDS);
    clear_all_leds();

    // knight_rider_simple(100);
    // show_all_notes(); 

    ESP_LOGI(TAG, "MIDI UART bridge initialized");
}
