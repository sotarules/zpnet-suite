#include "config.h"
#include "transport.h"
#include "debug.h"

#include <string.h>
#include <stdlib.h>

// --------------------------------------------------------------
// Parser state machine
// --------------------------------------------------------------
enum RxState {
    RX_IDLE,
    RX_STX,          // reading "<STX="
    RX_LEN,          // reading decimal length
    RX_PAYLOAD,      // reading payload bytes
    RX_ETX           // reading "<ETX>"
};

static RxState rx_state = RX_IDLE;

// Length parsing
static size_t expected_len = 0;
static size_t received_len = 0;

// Payload buffer
static char payload_buf[TRANSPORT_MAX_PAYLOAD];

// ETX matcher
static const char ETX_SEQ[] = "<ETX>";
static size_t etx_pos = 0;

// RX callback
static transport_rx_callback_t rx_cb = nullptr;

// --------------------------------------------------------------
// Helpers
// --------------------------------------------------------------
static void rx_reset() {
    rx_state     = RX_IDLE;
    expected_len = 0;
    received_len = 0;
    etx_pos      = 0;
}

static bool is_digit(char c) {
    return (c >= '0' && c <= '9');
}

// --------------------------------------------------------------
// Public API
// --------------------------------------------------------------
void transport_init(transport_rx_callback_t cb) {
    rx_cb = cb;
    rx_reset();
}

void transport_ingest_byte(char c) {
    // ------------------------------------------------------------------
    // Optional raw transport ingress echo (DEBUG ONLY)
    //
    // Echoes every received transport byte to the debug UART.
    // This is diagnostic scaffolding and MUST NOT be enabled in production.
    // ------------------------------------------------------------------

    switch (rx_state) {

        case RX_IDLE:
            if (c == '<') {
                rx_state = RX_STX;
                etx_pos = 0;
            }
            break;

        case RX_STX:
            // Expect "STX="
            if ((etx_pos == 0 && c == 'S') ||
                (etx_pos == 1 && c == 'T') ||
                (etx_pos == 2 && c == 'X') ||
                (etx_pos == 3 && c == '=')) {
                etx_pos++;
                if (etx_pos == 4) {
                    rx_state = RX_LEN;
                    expected_len = 0;
                }
            } else {
                rx_reset();
            }
            break;

        case RX_LEN:
            if (is_digit(c)) {
                expected_len = expected_len * 10 + (c - '0');
                if (expected_len > TRANSPORT_MAX_PAYLOAD) {
                    rx_reset();
                }
            } else if (c == '>') {
                received_len = 0;
                rx_state = RX_PAYLOAD;
            } else {
                rx_reset();
            }
            break;

        case RX_PAYLOAD:
            payload_buf[received_len++] = c;
            if (received_len == expected_len) {
                rx_state = RX_ETX;
                etx_pos = 0;
            }
            break;

        case RX_ETX:
            if (c == ETX_SEQ[etx_pos]) {
                etx_pos++;
                if (etx_pos == sizeof(ETX_SEQ) - 1) {
                    // Valid frame
                    if (rx_cb) {
                        rx_cb(payload_buf, expected_len);
                    }
                    rx_reset();
                }
            } else {
                rx_reset();
            }
            break;
    }
}

// --------------------------------------------------------------
// Outbound framing
// --------------------------------------------------------------
void transport_send_frame(const char* payload, size_t length) {
    if (!payload || length == 0) return;
    if (length > TRANSPORT_MAX_PAYLOAD) return;

    ZPNET_SERIAL.print("<STX=");
    ZPNET_SERIAL.print(length);
    ZPNET_SERIAL.print(">");

    ZPNET_SERIAL.write((const uint8_t*)payload, length);

    ZPNET_SERIAL.print("<ETX>");
}

void transport_send_frame(const char* payload) {
    if (!payload) return;
    transport_send_frame(payload, strlen(payload));
}
